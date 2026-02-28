#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于 Python Qt 的主界面应用程序
"""

import subprocess

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, 
    QWidget, QFileDialog, QMessageBox, QStatusBar, QTextEdit, QSplitter
)
from PySide6.QtCore import Qt, QTime
from PySide6.QtCore import Qt
import sys
import os
from lcm_player_thread import LCMPlayerThread

class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self):
        super().__init__()
        self.ros2_bag_process = None  # 用于存储 ros2 bag 录制进程
        self.init_ui()
    
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
        online_debug_button = QPushButton('在线监听', self)
        online_debug_button.setFixedSize(200, 50)
        online_debug_button.clicked.connect(self.on_online_watch)
        button_layout.addWidget(online_debug_button, 0, Qt.AlignCenter)
        
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
    
    def check_ros2_mode(self, to_enable=""):
        mode_file = "/opt/ros2_mode"
        ros2_mode = "idle"
        try:
            # 检查文件是否存在
            if not os.path.exists(mode_file):
                # 文件不存在，创建并写入 'record'
                with open(mode_file, 'w') as f:
                    if to_enable == "record":
                        ros2_mode = "record"
                        f.write('record')
                    else:
                        ros2_mode = "idle"
                        f.write('idle')
                self.statusBar().showMessage(f'当前ros2消息模式为 {ros2_mode}')
                self.log_message(f'设置ROS2消息模式为 {ros2_mode}')
            else:
                # 文件存在，读取内容
                with open(mode_file, 'r') as f:
                    content = f.read().strip()
                
                # 检查内容是否为 'replay'
                if content == 'replay':
                    QMessageBox.critical(self, '错误', '当前 ros2 消息模式为重放模式，无法开启此功能，请关闭plotjuggler的ros2消息重发，再试一次')
                    self.statusBar().showMessage(f'当前ros2消息模式为 {ros2_mode}')
                    ros2_mode = "replay"
                    self.log_message('ROS2消息模式为重放模式，操作被拒绝')
                else:
                    # 内容不是 'record'，更新为 'record'
                    with open(mode_file, 'w') as f:
                        if to_enable == "record":
                            ros2_mode = "record"
                            f.write('record')
                        elif to_enable == "idle":
                            ros2_mode = "idle"
                            f.write('idle')
                        else:
                            ros2_mode = content  # 保持原有模式
                            f.write(ros2_mode)
                    self.statusBar().showMessage(f'当前ros2消息模式为 {ros2_mode}')
                    self.log_message(f'设置ROS2消息模式为 {ros2_mode}')
        except Exception as e:
            QMessageBox.critical(self, '错误', f'处理 ros2 消息模式文件失败: {str(e)}')
            with open(mode_file, 'w') as f:
                f.write('idle')
            self.statusBar().showMessage('处理 ros2 消息模式文件失败')
            self.log_message(f'处理ROS2消息模式文件失败: {str(e)}')
            ros2_mode = "idle"
        return ros2_mode
        
    def on_ros2_bag_record(self):
        """ROS2 bag 录制槽函数"""
        if self.ros2_bag_process is None:
            # 开始录制
            if self.check_ros2_mode("record") == "replay":
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
                    QMessageBox.critical(self, '错误', f'启动ROS2 bag录制失败: {str(e)}')
                    self.statusBar().showMessage('启动ROS2 bag录制失败')
                    self.log_message(f'启动ROS2 bag录制失败: {str(e)}')
            else:
                self.check_ros2_mode("idle")
                self.log_message('用户取消了ROS2 bag录制')
        else:
            # 停止录制
            if self.check_ros2_mode("idle") == "replay":
                return
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
                
    def on_play_lcm_log(self):
        """打开日志槽函数"""
        # 检查是否正在播放
        if self.lcm_player_thread and self.lcm_player_thread.is_alive():
            # 停止播放
            if self.check_ros2_mode() == "replay":
                return
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
            if self.check_ros2_mode() == "replay":
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
                    QMessageBox.critical(self, '错误', f'启动 LCM 日志回放失败: {str(e)}')
                    self.statusBar().showMessage('启动 LCM 日志回放失败')
                    self.log_message(f'启动 LCM 日志回放失败: {str(e)}')
            else:
                self.log_message('用户取消了打开LCM日志文件')
    
    def on_online_watch(self):
        """在线监听槽函数"""
        self.statusBar().showMessage('在线监听模式')
        self.log_message('进入在线监听模式')
        QMessageBox.information(self, '信息', '在线监听功能')
        # 这里可以添加在线监听的代码

def main():
    """主函数"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()