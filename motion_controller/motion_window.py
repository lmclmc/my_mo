#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于 Python Qt 的主界面应用程序
"""

import subprocess
import asyncio
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QLineEdit,
    QWidget, QFileDialog, QMessageBox, QStatusBar, QTextEdit, QSplitter
)
from PySide6.QtCore import Qt, QTime, Signal
import sys
import os
from lcm_player_thread import LCMPlayerThread
import threading
import asyncio

class MainWindow(QMainWindow):
    """主窗口类"""
    log_message_updated = Signal(str)
    message_updated = Signal(int, str)  # 新增：更新消息的信号
    ping_timeout_updated = Signal(float)  # 新增：更新消息的信号
    
    def __init__(self):
        super().__init__()
        self.ros2_bag_process = None  # 用于存储 ros2 bag 录制进程
        self.init_ui()
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        self.log_message_updated.connect(self.log_message, Qt.QueuedConnection)
        self.message_updated.connect(self.update_message, Qt.QueuedConnection)
        self.ping_timeout_updated.connect(self.ping_timeout, Qt.QueuedConnection)
        
    def init_ui(self):
        """初始化用户界面"""
        # 设置窗口标题和大小
        self.setWindowTitle('主界面')
        self.setGeometry(100, 100, 600, 400)
        
        # 创建状态栏
        status_bar = QStatusBar()
        status_bar.showMessage('就绪')
        self.setStatusBar(status_bar)
        
        self.lcm_player_thread = LCMPlayerThread(None)  # 初始化线程对象，但不启动
        
        # 创建日志窗口
        self.create_log_window()
        
        # 创建中央部件
        self.create_central_widget()
    
    def create_log_window(self):
        """创建日志窗口"""
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setLineWrapMode(QTextEdit.NoWrap)
        self.log_text.setStyleSheet("QTextEdit { background-color: #f5f5f5; font-family: monospace; font-size: 9pt; line-height: 1.1; padding: 5px; margin: 2px; }")
    
    def create_central_widget(self):
        """创建中央部件"""
        central_widget = QWidget()
        main_layout = QVBoxLayout()
        
        # 创建按钮区域
        button_widget = QWidget()
        button_layout = QVBoxLayout()
        
        # 创建打开日志按钮
        self.open_log_button = QPushButton('播放lcm日志', self)
        self.open_log_button.setFixedSize(200, 50)
        self.open_log_button.clicked.connect(self.on_play_lcm_log)
        button_layout.addWidget(self.open_log_button, 0, Qt.AlignCenter)
        
        # 创建在线调试按钮
        self.online_debug_button = QPushButton('开始在线监听', self)
        self.online_debug_button.setFixedSize(200, 50)
        self.online_debug_button.clicked.connect(self.on_online_watch)
        button_layout.addWidget(self.online_debug_button, 0, Qt.AlignCenter)
        
        # 创建IP地址输入框
        self.ip_input = QLineEdit('192.168.54.1', self)
        self.ip_input.setFixedSize(200, 30)
        self.ip_input.setPlaceholderText('请输入IPv4地址')
        button_layout.addWidget(self.ip_input, 0, Qt.AlignCenter)
        
        # 创建 ROS2 bag 录制按钮
        self.ros2_bag_button = QPushButton('开始录制ROS2 bag', self)
        self.ros2_bag_button.setFixedSize(200, 50)
        self.ros2_bag_button.clicked.connect(self.on_ros2_bag_record)
        button_layout.addWidget(self.ros2_bag_button, 0, Qt.AlignCenter)
        
        # 设置按钮布局间距
        button_layout.setSpacing(20)  # 减少按钮间距，使整体更紧凑
        button_widget.setLayout(button_layout)
        
        # 创建分割器，将按钮区域和日志区域分开
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(button_widget)
        splitter.addWidget(self.log_text)
        splitter.setSizes([200, 150])  # 设置初始大小，减少日志区域高度
        
        # 添加分割器到主布局
        main_layout.addWidget(splitter)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # 记录初始日志
        self.log_message('应用程序已启动')
    
    def log_message(self, message):
        """记录消息到日志窗口"""
        current_time = QTime.currentTime().toString('HH:mm:ss')
        log_entry = f"[{current_time}] {message}"
        self.log_text.append(log_entry)
        # 滚动到最新消息
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
    
    def execute_script(self, script_path, mode, value):
        """
        执行指定脚本并获取其输出和返回值
        
        Args:
            script_path (str): 脚本路径
            mode (str): 模式参数 (play/record/playback/watch)
            value (int): 值参数 (0/1)
        
        Returns:
            dict: 包含脚本输出和返回值的字典，格式为 {"output": str, "return_code": int}
        """
        # 构建完整的命令行
        command = [script_path, mode, str(value)]
        
        try:
            # 执行命令并捕获输出
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # 将标准错误也捕获到标准输出
                text=True,  # 以文本模式获取输出
                check=False  # 不自动抛出异常
            )
            
            # 构建返回结果
            return {
                "output": result.stdout,
                "return_code": result.returncode
            }
        
        except Exception as e:
            # 处理异常情况
            return {
                "output": f"Error: {str(e)}",
                "return_code": -1
            }
        
    def on_ros2_bag_record(self):
        """ROS2 bag 录制槽函数"""
        if self.ros2_bag_process is None:
            # 开始录制
            result = self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "record", 1)
            if result["return_code"] != 0:
                QMessageBox.critical(self, "错误", result["output"])
                return
            # 弹出文件保存对话框
            file_name, _ = QFileDialog.getSaveFileName(
                self, '保存ROS2 bag文件', '', 'ROS2 bag文件 (*)'
            )
            if file_name:
                try:
                    # 确保文件路径存在
                    os.makedirs(os.path.dirname(file_name), exist_ok=True)
                    
                    # 启动 ros2 bag 录制进程
                    self.ros2_bag_process = subprocess.Popen([
                        'ros2', 'bag', 'record', '-o', file_name, '-a'
                    ])
                    self.statusBar().showMessage(f'已开始录制ROS2 bag: {file_name}')
                    self.log_message(f'已开始录制ROS2 bag: {file_name}')
                    QMessageBox.information(self, '信息', f'已开始录制ROS2 bag: {file_name}')
                    # 更改按钮文本
                    self.ros2_bag_button.setText('停止录制ROS2 bag')
                except Exception as e:
                    self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "record", 0)
                    QMessageBox.critical(self, '错误', f'启动ROS2 bag录制失败: {str(e)}')
                    self.statusBar().showMessage('启动ROS2 bag录制失败')
                    self.log_message(f'启动ROS2 bag录制失败: {str(e)}')
            else:
                self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "record", 0)
                self.log_message('用户取消了ROS2 bag录制')
        else:
            # 停止录制
            self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "record", 0)
            try:
                self.ros2_bag_process.terminate()
                self.ros2_bag_process.wait(timeout=5)
                self.statusBar().showMessage('已停止ROS2 bag录制')
                self.log_message('已停止ROS2 bag录制')
                QMessageBox.information(self, '信息', '已停止ROS2 bag录制')
                # 重置进程变量和按钮文本
                self.ros2_bag_process = None
                self.ros2_bag_button.setText('开始录制ROS2 bag')
            except Exception as e:
                QMessageBox.critical(self, '错误', f'停止ROS2 bag录制失败: {str(e)}')
                self.statusBar().showMessage('停止ROS2 bag录制失败')
                self.log_message(f'停止ROS2 bag录制失败: {str(e)}')
          
    def ping_timeout(self, timeout):
        """心跳超时槽函数"""
        print(f"心跳超时: {timeout:.4f} 秒")
        self.statusBar().showMessage(f'心跳超时: {timeout:.4f} 秒')
        self.log_message(f'心跳超时: {timeout:.4f} 秒')
        
    def update_message(self, mode, message):
        """更新消息显示槽函数"""
        if mode == 0:
            QMessageBox.information(self, '信息', message)
        else:
            QMessageBox.critical(self, '错误', message)
      
    def on_play_lcm_log(self):
        """打开日志槽函数"""
        # 检查是否正在播放
        if self.lcm_player_thread and self.lcm_player_thread.is_alive():
            # 停止播放
            self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "play_lcm", 0)

            try:
                self.lcm_player_thread.stop()
                self.lcm_player_thread.join()
                self.statusBar().showMessage('已停止 LCM 日志回放')
                self.log_message('已停止 LCM 日志回放')
            except Exception as e:
                QMessageBox.critical(self, '错误', f'停止 LCM 日志回放失败: {str(e)}')
                self.statusBar().showMessage('停止 LCM 日志回放失败')
                self.log_message(f'停止 LCM 日志回放失败: {str(e)}')
            self.open_log_button.setText('播放lcm日志')
        else:
            # 开始播放
            # 弹出文件选择对话框
            result = self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "play_lcm", 1)
            if result["return_code"] != 0:
                QMessageBox.critical(self, "错误", result["output"])
                return
            
            file_name, _ = QFileDialog.getOpenFileName(
                self, '打开日志文件', '', '所有文件 (*)'
            )
            if file_name:
                self.statusBar().showMessage(f'已选择日志文件: {file_name}')
                self.log_message(f'已选择日志文件: {file_name}')
                QMessageBox.information(self, '信息', f'已选择日志文件: {file_name}')
                
                # 启动LCM播放器线程
                try:
                    self.lcm_player_thread = LCMPlayerThread(file_name)
                    self.lcm_player_thread.start()
                    self.statusBar().showMessage('已启动 LCM 日志回放')
                    self.log_message('已启动 LCM 日志回放')
                    self.open_log_button.setText('停止播放lcm')
                except Exception as e:
                    self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "play_lcm", 0)
                    QMessageBox.critical(self, '错误', f'启动 LCM 日志回放失败: {str(e)}')
                    self.statusBar().showMessage('启动 LCM 日志回放失败')
                    self.log_message(f'启动 LCM 日志回放失败: {str(e)}')
            else:
                self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "play_lcm", 0)
                self.log_message('用户取消了打开LCM日志文件')
    
    def ping_callback(self, timeout):
        """心跳回调函数"""
        self.ping_timeout_updated.emit(timeout)
        
    def error_callback(self, message):
        """错误回调函数"""
        self.log_message_updated.emit(message)
                
    async def on_excute_watch(self):
        # 开始监听
        self.statusBar().showMessage('在线监听模式')
        self.log_message_updated.emit('进入在线监听模式')
        try:
            ip_address = self.ip_input.text().strip()
            # 1. 传输lcm_turn_server到远程机器
            self.log_message_updated.emit('正在传输lcm_turn_server到远程机器...')
            import subprocess
            
            # 检查lcm_turn_server可执行文件是否存在
            lcm_turn_server_path = '/opt/ros/lcm_turn_server'
            import os
            if not os.path.exists(lcm_turn_server_path):
                self.log_message_updated.emit('错误：lcm_turn_server可执行文件不存在')
                self.message_updated.emit(1, '错误：lcm_turn_server可执行文件不存在')
                return
            
            # 无脑删除lcm_turn_server文件
            scp_cmd = f'sshpass -p "123" ssh eame@{ip_address} "pkill -9 -f lcm_turn_server"'
            self.log_message_updated.emit(f'执行命令: {scp_cmd}')
            result = subprocess.run(scp_cmd, timeout=5, shell=True, capture_output=True, text=True)
            
            # 无脑删除lcm_turn_server文件
            scp_cmd = f'sshpass -p "123" ssh eame@{ip_address} "rm -rf /home/eame/lcm_turn_server"'
            self.log_message_updated.emit(f'执行命令: {scp_cmd}')
            result = subprocess.run(scp_cmd, timeout=5, shell=True, capture_output=True, text=True)
            
            self.log_message_updated.emit('lcm_turn_server垃圾清理成功')

            # 使用scp传输文件，设置10秒超时
            scp_cmd = f'sshpass -p "123" scp -o ConnectTimeout=10 {lcm_turn_server_path} eame@{ip_address}:/home/eame/lcm_turn_server'
            self.log_message_updated.emit(f'执行命令: {scp_cmd}')
            result = subprocess.run(scp_cmd, timeout=5, shell=True, capture_output=True, text=True)
            
            if result.returncode != 0:
                self.log_message_updated.emit(f'传输失败: {result.stderr}')
                self.message_updated.emit(1, f'传输lcm_turn_server失败: {result.stderr}')
                return
            self.log_message_updated.emit('lcm_turn_server传输成功')
            
            # 2. 给lcm_turn_server添加可执行权限
            self.log_message_updated.emit('正在远程启动lcm_turn_server...')
            ssh_cmd = f'sshpass -p "123" ssh -o ConnectTimeout=10 eame@{ip_address} "chmod +x /home/eame/lcm_turn_server"'
            self.log_message_updated.emit(f'执行命令: {ssh_cmd}')
            result = subprocess.run(ssh_cmd, timeout=5, shell=True, capture_output=True, text=True)
            if result.returncode != 0:
                self.log_message_updated.emit(f'添加可执行权限失败: {result.stderr}')
                self.message_updated.emit(1, f'添加可执行权限失败: {result.stderr}')
                return
            
            self.log_message_updated.emit('lcm_turn_server添加可执行权限')
            
            # 3. 远程启动lcm_turn_server
            self.log_message_updated.emit('正在远程启动lcm_turn_server...')
            ssh_cmd = f'sshpass -p "123" ssh -o ConnectTimeout=10 eame@{ip_address} "nohup /home/eame/lcm_turn_server > /home/eame/lcm_turn_server.log 2>&1 &"'
            self.log_message_updated.emit(f'执行命令: {ssh_cmd}')
            result = subprocess.run(ssh_cmd, timeout=15, shell=True, capture_output=True, text=True)
            if result.returncode != 0:
                self.log_message_updated.emit(f'启动失败: {result.stderr}')
                self.message_updated.emit(1, f'启动lcm_turn_server失败: {result.stderr}')
                return
            
            self.log_message_updated.emit('lcm_turn_server启动成功')
            
            # 3. 启动lcm_tcp_client客户端
            self.log_message_updated.emit('正在启动LCM TCP客户端...')
            from lcm_tcp_client import LcmTcpClient
            # 定义消息回调函数

            import time
            time.sleep(5)
            # 创建并启动客户端
            self.lcm_client = LcmTcpClient(
                server_ip=ip_address,
                server_port=58888,
                message_callback=self.ping_callback,
                error_callback=self.error_callback,
            )
            
            if self.lcm_client.start():
                self.log_message_updated.emit('LCM TCP客户端启动成功')
                # 修改按钮文本为停止监听
                self.online_debug_button.setText('停止在线监听')
                self.message_updated.emit(0, '在线监听功能已启动')
            else:
                self.log_message_updated.emit('LCM TCP客户端启动失败')
                self.message_updated.emit(1, 'LCM TCP客户端启动失败')
                
        except Exception as e:
            self.log_message_updated.emit(f'在线监听功能启动失败: {str(e)}')
            self.message_updated.emit(1, f'在线监听功能启动失败: {str(e)}')
            
        self.online_debug_button.setEnabled(True)
        
    def on_online_watch(self):
        """在线监听槽函数"""
        # 检查是否已经在监听
        if hasattr(self, 'lcm_client') and self.lcm_client and self.lcm_client.is_running():
            self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "watch", 0)
            # 停止监听
            self.log_message('正在停止在线监听...')
            try:
                # 停止LCM客户端
                self.lcm_client.stop()
                self.log_message('LCM TCP客户端已停止')
                # 远程停止lcm_turn_server（可选）
                import subprocess
                ssh_cmd = f'sshpass -p "123" ssh -o ConnectTimeout=10 eame@192.168.54.1 "pkill -f lcm_turn_server"'
                self.log_message(f'执行命令: {ssh_cmd}')
                result = subprocess.run(ssh_cmd, shell=True, capture_output=True, text=True)
                
                if result.returncode == 0:
                    self.log_message('lcm_turn_server已停止')
                else:
                    self.log_message('停止lcm_turn_server时出现错误（可能已停止）')
                
                # 恢复按钮文本
                self.online_debug_button.setText('开始在线监听')
                self.statusBar().showMessage('就绪')
                self.log_message('在线监听已停止')
                QMessageBox.information(self, '信息', '在线监听已停止')
                
            except Exception as e:
                self.log_message(f'停止在线监听失败: {str(e)}')
                QMessageBox.critical(self, '错误', f'停止在线监听失败: {str(e)}')
                
        else:
            result = self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "watch", 1)
            if result["return_code"] != 0:
                QMessageBox.critical(self, "错误", result["output"])
                return
            self.online_debug_button.setEnabled(False)
            def run_async():
                try:
                    asyncio.run(self.on_excute_watch())
                except Exception as e:
                    self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "watch", 0)
                    self.log_message(f'在线监听功能执行失败: {str(e)}')
                    # 重新启用按钮
                    self.online_debug_button.setEnabled(True)
                    self.online_debug_button.setText('开始在线监听')
            
            loop_thread = threading.Thread(target=run_async, daemon=True)
            loop_thread.start()
                
def main():
    """主函数"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()