#include "dds_idl.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <spawn.h>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;

namespace {
namespace fs = std::filesystem;

// 以二进制形式读取 path。打开失败或文件大小超过 limit 字节时抛出异常。
std::string readFile(const fs::path& path, size_t limit = 16 * 1024 * 1024) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("无法读取文件：" + path.string());
  if (fs::file_size(path) > limit) throw std::runtime_error("IDL 生成文件过大：" + path.string());
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

// 返回工具日志开头最多 8192 字节，用于限制异常消息长度。
std::string diagnostics(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  std::string text(8192, '\0');
  input.read(text.data(), static_cast<std::streamsize>(text.size()));
  text.resize(static_cast<size_t>(input.gcount()));
  return text;
}

// 执行 args 中的工具和参数，将标准输出写入 output、标准错误写入 log。
// 参数直接传给进程，含空格的路径可原样传入。启动、运行或等待失败时抛出异常。
void run(const std::vector<std::string>& args, const fs::path& output, const fs::path& log) {
  struct SpawnState {
    posix_spawn_file_actions_t files;
    posix_spawnattr_t attrs;
    // 初始化进程的文件操作和启动属性，第二步失败时释放已初始化的文件操作。
    SpawnState() {
      const int error = posix_spawn_file_actions_init(&files);
      if (error) throw std::runtime_error(std::strerror(error));
      const int attr_error = posix_spawnattr_init(&attrs);
      if (attr_error) {
        posix_spawn_file_actions_destroy(&files);
        throw std::runtime_error(std::strerror(attr_error));
      }
    }
    // 离开作用域时释放进程启动所用的两组配置。
    ~SpawnState() {
      posix_spawnattr_destroy(&attrs);
      posix_spawn_file_actions_destroy(&files);
    }
  } state;
  auto check = [](int error) { if (error) throw std::runtime_error(std::strerror(error)); };
  check(posix_spawn_file_actions_addopen(&state.files, STDIN_FILENO, "/dev/null", O_RDONLY, 0));
  check(posix_spawn_file_actions_addopen(&state.files, STDOUT_FILENO, output.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600));
  check(posix_spawn_file_actions_addopen(&state.files, STDERR_FILENO, log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600));
  // 给工具创建独立进程组，超时或等待失败时可终止工具及同组子进程。
  check(posix_spawnattr_setflags(&state.attrs, POSIX_SPAWN_SETPGROUP));
  check(posix_spawnattr_setpgroup(&state.attrs, 0));
  std::vector<char*> argv;
  for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);
  pid_t child = -1;
  const int error = posix_spawnp(&child, argv[0], &state.files, &state.attrs, argv.data(), environ);
  if (error) throw std::runtime_error("无法启动 " + args.front() + "：" + std::strerror(error) +
                                     "；请安装匹配 CycloneDDS 的 idlc 和本机 C 编译器");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  int status = 0;
  for (;;) {
    const auto result = waitpid(child, &status, WNOHANG);
    if (result == child) break;
    if (result < 0 && errno != EINTR) {
      const int wait_error = errno;
      kill(-child, SIGKILL);
      while (waitpid(child, &status, 0) < 0 && errno == EINTR) {}
      throw std::runtime_error("等待 IDL 工具失败：" + std::string(std::strerror(wait_error)));
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      kill(-child, SIGKILL);
      while (waitpid(child, &status, 0) < 0 && errno == EINTR) {}
      throw std::runtime_error("IDL 处理超时（30 秒）：" + args.front());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
    throw std::runtime_error("IDL 处理失败：" + args.front() + "\n" + diagnostics(log) + diagnostics(output));
}

// 优先返回可执行的构建时工具路径，否则返回供 posix_spawnp 按 PATH 查找的名称。
std::string tool(const char* configured, const char* fallback) {
  return access(configured, X_OK) == 0 ? configured : fallback;
}
}  // namespace

struct DdsIdlLibrary::Impl {
  std::string source;
  fs::path directory;
  void* library = nullptr;
  std::map<std::string, const dds_topic_descriptor_t*> types;
  // 先卸载包含描述符的共享库，再尝试删除该次导入的构建产物。
  ~Impl() {
    if (library) dlclose(library);
    if (!directory.empty()) {
      std::error_code ignored;
      fs::remove_all(directory, ignored);
    }
  }
};

// 在独立临时目录中预处理、生成并编译 IDL，再按 DDS 类型全名登记可用描述符。
std::shared_ptr<const DdsIdlLibrary> DdsIdlLibrary::load(const std::string& path) {
  auto impl = std::make_unique<Impl>();
  impl->source = fs::canonical(path).string();
  if (!fs::is_regular_file(impl->source)) throw std::runtime_error("请选择有效的 IDL 文件");
  std::string pattern = (fs::temp_directory_path() / "plotjuggler-dds-idl-XXXXXX").string();
  if (!mkdtemp(pattern.data())) throw std::runtime_error("无法创建 IDL 临时目录：" + std::string(std::strerror(errno)));
  impl->directory = pattern;
  const auto& dir = impl->directory;
  const auto idlc = tool(PJ_DDS_IDLC, "idlc");
  const auto include_dir = fs::path(impl->source).parent_path().string();
  run({idlc, "-E", "-I", include_dir, impl->source}, dir / "preprocessed.idl", dir / "preprocess.log");

  // 将预处理展开的依赖合成一个 IDL 文件。只去掉源文件行号标记，
  // 保留 pragma 和注解，让 idlc 一次生成主文件及其依赖的 C 定义。
  std::istringstream preprocessed(readFile(dir / "preprocessed.idl"));
  std::ofstream flattened(dir / "schema.idl");
  if (!flattened) throw std::runtime_error("无法写入 IDL 临时文件");
  const std::regex line_marker(R"(^\s*#\s*(line\s+)?[0-9]+(\s|$))");
  for (std::string line; std::getline(preprocessed, line);)
    if (!std::regex_search(line, line_marker)) flattened << line << '\n';
  flattened.close();
  if (!flattened) throw std::runtime_error("写入 IDL 临时文件失败");
  run({idlc, "-l", "c", "-n", "false", "-x", "final", "-o", dir.string(), (dir / "schema.idl").string()},
      dir / "generate.out", dir / "generate.log");

  // 使用插件构建时选定的 CycloneDDS 头文件和库编译描述符，并将符号留在局部作用域。
  std::vector<std::string> command{tool(PJ_DDS_CC, "cc"), "-std=c11", "-shared", "-fPIC", "-O2", "-I", dir.string()};
  std::istringstream includes(PJ_DDS_INCLUDE_DIRS);
  for (std::string include; std::getline(includes, include, '|');)
    if (!include.empty()) { command.push_back("-I"); command.push_back(include); }
  command.insert(command.end(), {(dir / "schema.c").string(), PJ_DDS_LIBRARY,
                                "-Wl,-rpath," + fs::path(PJ_DDS_LIBRARY).parent_path().string(),
                                "-o", (dir / "schema.so").string()});
  run(command, dir / "compile.out", dir / "compile.log");
  impl->library = dlopen((dir / "schema.so").c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!impl->library) throw std::runtime_error("加载 IDL 类型失败：" + std::string(dlerror()));

  // 从生成的 C 文件提取描述符符号，并检查每个描述符都含有 XTypes 类型信息和映射。
  const std::string generated = readFile(dir / "schema.c");
  const std::regex descriptor(R"(\bconst\s+dds_topic_descriptor_t\s+([A-Za-z_][A-Za-z0-9_]*)\s*=)");
  for (std::sregex_iterator it(generated.begin(), generated.end(), descriptor), end; it != end; ++it) {
    const auto symbol = (*it)[1].str();
    auto* type = static_cast<const dds_topic_descriptor_t*>(dlsym(impl->library, symbol.c_str()));
    if (!type || !type->m_typename || !(type->m_flagset & DDS_TOPIC_XTYPES_METADATA) ||
        !type->type_information.sz || !type->type_mapping.sz)
      throw std::runtime_error("IDL 没有生成完整类型信息：" + symbol);
    impl->types[type->m_typename] = type;
  }
  if (impl->types.empty()) throw std::runtime_error("IDL 中没有可订阅的消息类型，请检查 @topic / @nested 注解");
  return std::shared_ptr<const DdsIdlLibrary>(new DdsIdlLibrary(std::move(impl)));
}

// 接管本次导入生成的库和临时目录。
DdsIdlLibrary::DdsIdlLibrary(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
// 由 Impl 统一清理动态库和临时文件。
DdsIdlLibrary::~DdsIdlLibrary() = default;
// 返回加载时保存的规范化源路径。
const std::string& DdsIdlLibrary::sourcePath() const { return impl_->source; }
// 按索引中的类型名顺序复制名称列表。
std::vector<std::string> DdsIdlLibrary::typeNames() const {
  std::vector<std::string> result;
  for (const auto& [name, descriptor] : impl_->types) { (void)descriptor; result.push_back(name); }
  return result;
}
// 查询已加载的类型，命中时返回共享库中的描述符地址。
const dds_topic_descriptor_t* DdsIdlLibrary::find(const std::string& type_name) const {
  const auto it = impl_->types.find(type_name);
  return it == impl_->types.end() ? nullptr : it->second;
}
