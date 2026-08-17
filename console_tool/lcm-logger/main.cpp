#include "cmdline.hpp"
#include "typesingle.hpp"
#include "logsink.hpp"
#include <lcm/lcm-cpp.hpp>

using namespace lmc;

class HandleMessage {
public:
    HandleMessage() = default;

    void set_lcm_log(std::shared_ptr<rotating_file_sink> log) {
        lcm_log = log;
    }
    void handleMessage(const lcm::ReceiveBuffer* recvbuf, const std::string& topic_name) {
        if (lcm_log)
            lcm_log->write(topic_name, recvbuf);
    }

private:
    std::shared_ptr<rotating_file_sink> lcm_log;
};

int main(int argc, char* argv[]) {
    CmdLine *cmd = TypeSingle<CmdLine>::getInstance();
    HandleMessage handle_message;

    std::list<std::shared_ptr<lcm::LCM>> lcm_list;
    std::string file_path = "";
    int max_file_size = 0;
    int max_files_num = 0;
    cmd->add("-p", "--port", "set listen port(s), support single or multiple ports", [&lcm_list, &handle_message] (std::list<int> port_list) {
        for (auto port : port_list) {
            std::string lcm_url = std::string("udpm://239.255.76.67:") + std::to_string(port);
            auto lcm_ptr = std::make_shared<lcm::LCM>(lcm_url);
            lcm_ptr->subscribe(".*", &HandleMessage::handleMessage, &handle_message);
            lcm_list.emplace_back(lcm_ptr);
        }
    }, {"-m", "-n", "-f"}, {100, 65536});
    
    cmd->add("-m", "--set_max_file_size", "set max file size(unit: byte)", [&max_file_size] (int max_file_size_) {
        if (max_file_size_ < 1024 * 1024) {
            std::cout << "max file size must be greater than 1MB" << std::endl;
            exit(0);
        }
        
        max_file_size = max_file_size_;
    });

    cmd->add("-n", "--set_max_files_num", "set max files num", [&max_files_num] (int max_files_num_) {
        max_files_num = max_files_num_;
    });

    cmd->add("-f", "--set_file_path", "set file path", [&file_path] (std::string file_path_) {
        file_path = file_path_;
    });

    cmd->parse(false, argc, argv);

    handle_message.set_lcm_log(std::make_shared<rotating_file_sink>(file_path, max_file_size, max_files_num));

    while (1) {
      fd_set fds;
      FD_ZERO(&fds);
      int max_fd = 0;
      for (auto lcm_ptr: lcm_list) {
        int fd = lcm_ptr->getFileno();
        FD_SET(fd, &fds);
        if (fd > max_fd) {
          max_fd = fd;
        }
      }

      int status = select(max_fd + 1, &fds, 0, 0, 0);
      if (status) {
        for (auto lcm_ptr: lcm_list) {
          int fd = lcm_ptr->getFileno();
          if (FD_ISSET(fd, &fds)) {
            if (lcm_ptr) {
              lcm_ptr->handle();
            }
          }
        }
      }
    }

    TypeSingle<CmdLine>::destory();
    return 0;
}