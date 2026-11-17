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

std::string readFile(const fs::path& path, size_t limit = 16 * 1024 * 1024) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("无法读取文件：" + path.string());
  if (fs::file_size(path) > limit) throw std::runtime_error("IDL 生成文件过大：" + path.string());
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::string diagnostics(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  std::string text(8192, '\0');
  input.read(text.data(), static_cast<std::streamsize>(text.size()));
  text.resize(static_cast<size_t>(input.gcount()));
  return text;
}

// Use argv directly, including paths with spaces; no shell interprets user paths
// or IDL contents. A process group bounds cleanup even if the compiler forks.
void run(const std::vector<std::string>& args, const fs::path& output, const fs::path& log) {
  struct SpawnState {
    posix_spawn_file_actions_t files;
    posix_spawnattr_t attrs;
    SpawnState() {
      const int error = posix_spawn_file_actions_init(&files);
      if (error) throw std::runtime_error(std::strerror(error));
      const int attr_error = posix_spawnattr_init(&attrs);
      if (attr_error) {
        posix_spawn_file_actions_destroy(&files);
        throw std::runtime_error(std::strerror(attr_error));
      }
    }
    ~SpawnState() {
      posix_spawnattr_destroy(&attrs);
      posix_spawn_file_actions_destroy(&files);
    }
  } state;
  auto check = [](int error) { if (error) throw std::runtime_error(std::strerror(error)); };
  check(posix_spawn_file_actions_addopen(&state.files, STDIN_FILENO, "/dev/null", O_RDONLY, 0));
  check(posix_spawn_file_actions_addopen(&state.files, STDOUT_FILENO, output.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600));
  check(posix_spawn_file_actions_addopen(&state.files, STDERR_FILENO, log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600));
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

std::string tool(const char* configured, const char* fallback) {
  return access(configured, X_OK) == 0 ? configured : fallback;
}
}  // namespace

struct DdsIdlLibrary::Impl {
  std::string source;
  fs::path directory;
  void* library = nullptr;
  std::map<std::string, const dds_topic_descriptor_t*> types;
  ~Impl() {
    if (library) dlclose(library);
    if (!directory.empty()) {
      std::error_code ignored;
      fs::remove_all(directory, ignored);
    }
  }
};

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

  // Flatten preprocessed includes into one translation unit. Stripping only
  // source-line markers keeps pragmas/annotations and gives idlc all dependency
  // definitions, so users need not generate included .h/.c files themselves.
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

DdsIdlLibrary::DdsIdlLibrary(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
DdsIdlLibrary::~DdsIdlLibrary() = default;
const std::string& DdsIdlLibrary::sourcePath() const { return impl_->source; }
std::vector<std::string> DdsIdlLibrary::typeNames() const {
  std::vector<std::string> result;
  for (const auto& [name, descriptor] : impl_->types) { (void)descriptor; result.push_back(name); }
  return result;
}
const dds_topic_descriptor_t* DdsIdlLibrary::find(const std::string& type_name) const {
  const auto it = impl_->types.find(type_name);
  return it == impl_->types.end() ? nullptr : it->second;
}
