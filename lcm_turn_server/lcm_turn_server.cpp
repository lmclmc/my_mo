#include <iostream>
#include <thread>
#include <queue>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <vector>
#include <lcm/lcm-cpp.hpp>
#include <netinet/tcp.h>
#include <fcntl.h>
#include "singleton.hpp"
#include "spinmutex.hpp"

// 线程安全的消息队列
template<typename T>
class SafeQueue {
public:
    void push(const T& item) {
        std::lock_guard<lmc::SpinMutex> lock(mutex_);
        queue_.push(item);
    }

    T pop() {
        std::lock_guard<lmc::SpinMutex> lock(mutex_);
        T item = queue_.front();
        queue_.pop();
        return item;
    }

    bool empty() {
        std::lock_guard<lmc::SpinMutex> lock(mutex_);
        return queue_.empty();
    }

    int size() {
        std::lock_guard<lmc::SpinMutex> lock(mutex_);
        return queue_.size();
    }

private:
    std::queue<T> queue_;
    lmc::SpinMutex mutex_;
};

// LCM消息结构体
struct LcmMessage {
    std::string channel;
    std::vector<uint8_t> data;
};

// 全局变量
SafeQueue<LcmMessage> g_message_queue;
lmc::SpinMutex g_clients_mutex;
std::vector<int> g_client_sockets;

// LCM消息处理类
class LcmHandler : public lcm::LCM {
public:
    LcmHandler() : lcm::LCM("udpm://239.255.76.67:7667") {
        if (!good()) {
            std::cerr << "Failed to initialize LCM" << std::endl;
            exit(1);
        }
    }

    void subscribeAll() {
        std::cout << "Subscribing to all LCM channels..." << std::endl;
        // 订阅所有通道的消息
        subscribe(".*", &LcmHandler::handleMessage, this);
    }

private:
    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
        LcmMessage msg;
        msg.channel = channel;
        msg.data.assign(reinterpret_cast<const uint8_t*>(rbuf->data), reinterpret_cast<const uint8_t*>(rbuf->data) + rbuf->data_size);
        g_message_queue.push(msg);
    }
};

// TCP服务器类
class TcpServer {
public:
    TcpServer(int port) : port_(port), epoll_fd_(-1), server_fd_(-1) {}

    bool start() {
        // 创建socket
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }

        // 设置socket选项
        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
            std::cerr << "Failed to set socket options" << std::endl;
            close(server_fd_);
            return false;
        }

        // 禁用 Naggle 算法
        int nodelay = 1;
        if (setsockopt(server_fd_, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay)) < 0) {
            std::cerr << "Failed to set TCP_NODELAY" << std::endl;
        }

        // 绑定地址
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "Failed to bind socket" << std::endl;
            close(server_fd_);
            return false;
        }

        // 开始监听
        if (listen(server_fd_, 32) < 0) {
            std::cerr << "Failed to listen on socket" << std::endl;
            close(server_fd_);
            return false;
        }

        // 创建epoll实例
        epoll_fd_ = epoll_create1(0);
        if (epoll_fd_ < 0) {
            std::cerr << "Failed to create epoll" << std::endl;
            close(server_fd_);
            return false;
        }

        // 将服务器socket添加到epoll
        struct epoll_event event;
        event.events = EPOLLIN;
        event.data.fd = server_fd_;
        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, server_fd_, &event) < 0) {
            std::cerr << "Failed to add server socket to epoll" << std::endl;
            close(epoll_fd_);
            close(server_fd_);
            return false;
        }

        std::cout << "TCP server started on port " << port_ << std::endl;
        return true;
    }

    void run() {
        if (epoll_fd_ < 0 || server_fd_ < 0) {
            std::cerr << "TCP server not started" << std::endl;
            return;
        }

        struct epoll_event events[1024];

        while (true) {
            // 等待事件
            int nfds = epoll_wait(epoll_fd_, events, 1024, 1); // 1ms超时
            if (nfds < 0) {
                std::cerr << "Epoll wait failed" << std::endl;
                break;
            }

            // 处理事件
            for (int i = 0; i < nfds; i++) {
                if (events[i].data.fd == server_fd_) {
                    // 新连接
                    handleNewConnection();
                } else {
                    // 现有连接的事件
                    handleClientEvent(events[i].data.fd, events[i].events);
                }
            }

            // 发送队列中的消息
            sendQueuedMessages();
        }
    }

    void stop() {
        if (epoll_fd_ >= 0) {
            close(epoll_fd_);
        }
        if (server_fd_ >= 0) {
            close(server_fd_);
        }
        
        // 关闭所有客户端连接
        std::lock_guard<lmc::SpinMutex> lock(g_clients_mutex);
        for (int sock : g_client_sockets) {
            close(sock);
        }
        g_client_sockets.clear();
    }

private:
    void handleNewConnection() {
        struct sockaddr_in client_address;
        socklen_t client_addrlen = sizeof(client_address);
        int client_fd = accept(server_fd_, (struct sockaddr*)&client_address, &client_addrlen);
        if (client_fd < 0) {
            std::cerr << "Failed to accept connection" << std::endl;
            return;
        }

        // 设置为非阻塞
        int flags = fcntl(client_fd, F_GETFL, 0);
        fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);

        // 添加到epoll
        struct epoll_event event;
        event.events = EPOLLIN | EPOLLET;
        event.data.fd = client_fd;
        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, client_fd, &event) < 0) {
            std::cerr << "Failed to add client socket to epoll" << std::endl;
            close(client_fd);
            return;
        }

        // 添加到客户端列表
        { 
            std::lock_guard<lmc::SpinMutex> lock(g_clients_mutex);
            g_client_sockets.push_back(client_fd);
        }

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(client_address.sin_addr), client_ip, INET_ADDRSTRLEN);
        std::cout << "New client connected: " << client_ip << ":" << ntohs(client_address.sin_port) << std::endl;
    }

    void handleClientEvent(int client_fd, uint32_t events) {
        if (events & (EPOLLRDHUP | EPOLLHUP | EPOLLERR)) {
            // 连接关闭或错误
            closeClient(client_fd);
        } else if (events & EPOLLIN) {
            // 有数据可读
            char buffer[1024];
            int read_size = read(client_fd, buffer, sizeof(buffer));
            if (read_size <= 0) {
                closeClient(client_fd);
                return;
            }

            // 解析客户端数据（这里只处理心跳包）
            // 注意：实际应用中可能需要更复杂的协议解析
            // 这里简化处理，假设客户端发送的是心跳包
            handleHeartbeat(client_fd);
        }
    }

    void handleHeartbeat(int client_fd) {
        // 回复心跳包
        sendHeartbeatResponse(client_fd);
    }

    void sendHeartbeatResponse(int client_fd) {
        // 构建心跳响应
        std::string channel = "__heartbeat__";
        std::string data = "pong";

        uint32_t channel_len = channel.size();
        uint32_t data_len = data.size();
        uint32_t total_len = sizeof(channel_len) + channel_len + sizeof(data_len) + data_len;

        // 分配缓冲区
        std::vector<uint8_t> buffer(total_len);
        uint32_t offset = 0;

        // 写入通道长度
        memcpy(buffer.data() + offset, &channel_len, sizeof(channel_len));
        offset += sizeof(channel_len);
        // 写入通道名
        memcpy(buffer.data() + offset, channel.data(), channel_len);
        offset += channel_len;

        // 写入数据长度
        memcpy(buffer.data() + offset, &data_len, sizeof(data_len));
        offset += sizeof(data_len);

        // 写入数据
        memcpy(buffer.data() + offset, data.data(), data_len);

        // 发送响应
        ssize_t sent = send(client_fd, buffer.data(), buffer.size(), 0);
        if (sent < 0) {
            // 发送失败，关闭连接
            closeClient(client_fd);
        }
    }

    void closeClient(int client_fd) {
        // 从epoll中移除
        epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, client_fd, nullptr);
        // 关闭socket
        close(client_fd);
        // 从客户端列表中移除
        { 
            std::lock_guard<lmc::SpinMutex> lock(g_clients_mutex);
            auto it = std::find(g_client_sockets.begin(), g_client_sockets.end(), client_fd);
            if (it != g_client_sockets.end()) {
                g_client_sockets.erase(it);
            }
        }
        std::cout << "Client disconnected: " << client_fd << std::endl;
    }

    void sendQueuedMessages() {
        // 检查队列是否有消息
        if (g_message_queue.empty()) {
            return;
        }
        int queue_size = g_message_queue.size();

        for (size_t i =0; i < queue_size; i++) {
            LcmMessage msg = g_message_queue.pop();
            // 构建消息格式：通道长度 + 通道名 + 数据长度 + 数据
            uint32_t channel_len = msg.channel.size();
            uint32_t data_len = msg.data.size();
            uint32_t total_len = sizeof(channel_len) + channel_len + sizeof(data_len) + data_len;

            // 分配缓冲区
            std::vector<uint8_t> buffer(total_len);
            uint32_t offset = 0;

            // 写入通道长度
            memcpy(buffer.data() + offset, &channel_len, sizeof(channel_len));
            offset += sizeof(channel_len);
            // 写入通道名
            memcpy(buffer.data() + offset, msg.channel.data(), channel_len);
            offset += channel_len;

            // 写入数据长度
            memcpy(buffer.data() + offset, &data_len, sizeof(data_len));
            offset += sizeof(data_len);

            // 写入数据
            memcpy(buffer.data() + offset, msg.data.data(), data_len);

            // 发送给所有客户端
            std::lock_guard<lmc::SpinMutex> lock(g_clients_mutex);
            for (auto it = g_client_sockets.begin(); it != g_client_sockets.end();) {
                int client_fd = *it;
                ssize_t sent = send(client_fd, buffer.data(), buffer.size(), 0);
                if (sent < 0) {
                    // 区分不同类型的错误
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // 非致命错误，暂时跳过
                        ++it;
                    } else {
                        // 发送失败，关闭连接
                        std::cout << "Failed to send message to client " << client_fd << " " << strerror(errno) << std::endl;
                        closeClient(client_fd);
                        it = g_client_sockets.erase(it);
                    }
                } else {
                    ++it;
                }
            }
        }
    }

    int port_;
    int epoll_fd_;
    int server_fd_;
};

int main() {
    std::cout << "LCM to TCP forwarder starting..." << std::endl;

    // 启动LCM订阅
    LcmHandler* lcm = lmc::TypeSingle<LcmHandler>::getInstance();
    lcm->subscribeAll();

    // 启动TCP服务器
    TcpServer* server = lmc::TypeSingle<TcpServer>::getInstance(58888); // 使用8080端口
    if (!server->start()) {
        std::cerr << "Failed to start TCP server" << std::endl;
        return 1;
    }

    // 启动TCP服务器线程
    std::thread tcp_thread(&TcpServer::run, server);

    // 主线程运行LCM事件循环
    while (true) {
        lcm->handle();
    }

    // 清理资源
    server->stop();
    tcp_thread.join();

    return 0;
}