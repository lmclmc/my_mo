#!/usr/bin/env python3
"""
LCM 转发服务器的 Python 客户端类
带有独立线程，可由主界面调用
"""
import lcm
import socket
import struct
import threading
import time

class LcmTcpClient:
    """LCM TCP 客户端类，带有独立线程处理消息接收"""
    
    def __init__(self, server_ip='192.168.54.110', server_port=58888, message_callback=None, error_callback=None):
        """
        初始化客户端
        
        Args:
            server_ip (str): 服务器 IP 地址
            server_port (int): 服务器端口
            message_callback (callable): 消息回调函数，格式为 callback(channel, data)
        """
        self.server_ip = server_ip
        self.server_port = server_port
        self.message_callback = message_callback
        self.error_callback = error_callback
        
        self.client_socket = None
        self.thread = None
        self.running = False
        self.connected = False
        self.lock = threading.Lock()
        self.ping_timestamp = 0 # 上次心跳时间
        self.ping_timeout = 0 # 心跳超时时间
    
        lcm_url = f"udpm://239.255.76.67:56666"
        self.lc = lcm.LCM(lcm_url) # 创建 LCM 实例用于发送数据
    
    def start(self):
        """启动客户端，开始接收消息"""
        with self.lock:
            if self.running:
                if self.error_callback:
                    self.error_callback("客户端已经在运行")
                return False
            
            try:
                # 创建 TCP 套接字
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.server_ip, self.server_port))
                self.connected = True
                
                # 禁用 Naggle 算法，提高实时性
                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                
                # 启动接收线程
                self.running = True
                self.thread_receive = threading.Thread(target=self._receive_thread, daemon=True)
                self.thread_receive.start()
                
                self.thread_ping = threading.Thread(target=self._ping_thread, daemon=True)
                self.thread_ping.start()
                
                if self.error_callback:
                    self.error_callback(f"客户端已启动，连接到 {self.server_ip}:{self.server_port}")
                return True
                
            except socket.error as e:
                if self.error_callback:
                    self.error_callback(f"连接服务器时出错: {e}")
                self.connected = False
                return False
    
    def stop(self):
        """停止客户端，关闭连接"""
        with self.lock:
            if not self.running:
                return
            
            self.running = False
            
            # 等待线程结束
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=2.0)
            
            # 关闭套接字
            if self.client_socket:
                try:
                    self.client_socket.close()
                except:
                    pass
                self.client_socket = None
            
            self.connected = False
            if self.error_callback:
                self.error_callback("客户端已停止")
    
    def is_connected(self):
        """检查客户端是否已连接"""
        return self.connected
    
    def is_running(self):
        """检查客户端是否正在运行"""
        return self.running
    
    def _ping_thread(self):
        "心跳"
        while self.running:
            time.sleep(10)
            # 发送心跳包
            self.client_socket.sendall(b'ping')
            self.ping_timestamp = time.time()

    def _receive_thread(self):
        """接收线程函数，持续接收和处理消息"""
        while self.running:
            try:
                if not self.client_socket:
                    time.sleep(0.1)
                    continue
                # 接收通道长度（4字节，小端）
                channel_len_data = self._recv_all(4)
                if not channel_len_data:
                    break
                channel_len = struct.unpack('<I', channel_len_data)[0]
                if channel_len > 128:
                    if self.error_callback:
                        self.error_callback(f"通道名长度异常: {channel_len}")
                    continue
                # 接收通道名
                channel_name = self._recv_all(channel_len).decode('utf-8')
                if channel_name == '__heartbeat__':
                    if self.message_callback:
                        print(f"收到心跳包，时间差: {time.time() - self.ping_timestamp}")
                        self.message_callback(time.time() - self.ping_timestamp)
                    
                # 接收数据长度（4字节，小端）
                data_len_data = self._recv_all(4)
                if not data_len_data:
                    break
                data_len = struct.unpack('<I', data_len_data)[0]
                if data_len > 3200:
                    if self.error_callback:
                        self.error_callback(f"数据长度异常: {data_len}")
                    continue
                # 接收数据
                data = self._recv_all(data_len)
                if not data:
                    break
                self.lc.publish(channel_name, data)
                # # 调用回调函数处理消息
                # if self.message_callback:
                #     try:
                #         self.message_callback(channel_name, data)
                        
                #     except Exception as e:
                #         print(f"回调函数执行出错: {e}")
                
            except socket.error as e:
                if self.error_callback:
                    self.error_callback(f"接收数据时出错: {e}")
                break
            except Exception as e:
                if self.error_callback:
                    self.error_callback(f"处理消息时出错: {e}")
                continue
        
        # 连接断开
        with self.lock:
            self.connected = False
            self.running = False
            if self.client_socket:
                try:
                    self.client_socket.close()
                except:
                    pass
                self.client_socket = None
            if self.error_callback:
                self.error_callback("连接已关闭")
    
    def _recv_all(self, size):
        """
        接收指定大小的数据
        
        Args:
            size (int): 需要接收的数据大小
        
        Returns:
            bytes: 接收到的数据，如果连接关闭则返回空 bytes
        """
        data = b''
        while len(data) < size:
            try:
                chunk = self.client_socket.recv(size - len(data))
                if not chunk:
                    return b''
                data += chunk
            except:
                return b''
        return data


# 示例用法
if __name__ == "__main__":
    def message_handler(channel, data):
        """消息处理回调函数"""
        print(f"收到消息:")
        print(f"  通道: {channel}")
        print(f"  数据长度: {len(data)}")
        print(f"  数据: {data.hex()[:50]}...")
        print()
    
    # 创建客户端实例
    client = LcmTcpClient(
        server_ip='192.168.54.1',
        server_port=58888,
        message_callback=message_handler
    )
    
    # 启动客户端
    if client.start():
        print("按 Ctrl+C 停止客户端...")
        try:
            while client.is_running():
                time.sleep(1)
        except KeyboardInterrupt:
            print("正在停止客户端...")
            client.stop()
    else:
        print("客户端启动失败")