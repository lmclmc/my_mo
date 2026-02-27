#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于 Python Qt 的主界面应用程序
"""

import subprocess

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, 
    QWidget, QFileDialog, QMessageBox, QStatusBar
)
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
        self.setGeometry(100, 100, 400, 200)
        
        # 创建状态栏
        status_bar = QStatusBar()
        status_bar.showMessage('就绪')
        self.setStatusBar(status_bar)
        
        self.lcm_player_thread = LCMPlayerThread(None)  # 初始化线程对象，但不启动
        
        # 创建中央部件
        self.create_central_widget()
    
    def create_central_widget(self):
        """创建中央部件"""
        central_widget = QWidget()
        layout = QVBoxLayout()
        
        # 创建打开日志按钮
        self.open_log_button = QPushButton('播放lcm日志', self)
        self.open_log_button.setFixedSize(200, 50)
        self.open_log_button.clicked.connect(self.on_play_lcm_log)
        layout.addWidget(self.open_log_button, 0, Qt.AlignCenter)
        
        # 创建在线调试按钮
        online_debug_button = QPushButton('在线调试', self)
        online_debug_button.setFixedSize(200, 50)
        online_debug_button.clicked.connect(self.on_online_debug)
        layout.addWidget(online_debug_button, 0, Qt.AlignCenter)
        
        # 创建 ROS2 bag 录制按钮
        self.ros2_bag_button = QPushButton('开始录制ROS2 bag', self)
        self.ros2_bag_button.setFixedSize(200, 50)
        self.ros2_bag_button.clicked.connect(self.on_ros2_bag_record)
        layout.addWidget(self.ros2_bag_button, 0, Qt.AlignCenter)
        
        # 设置布局间距
        layout.setSpacing(30)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
    
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
            else:
                # 文件存在，读取内容
                with open(mode_file, 'r') as f:
                    content = f.read().strip()
                
                # 检查内容是否为 'replay'
                if content == 'replay':
                    QMessageBox.critical(self, '错误', '当前 ros2 消息模式为重放模式，无法开启此功能，请关闭plotjuggler的ros2消息重发，再试一次')
                    self.statusBar().showMessage(f'当前ros2消息模式为 {ros2_mode}')
                    ros2_mode = "replay"
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
        except Exception as e:
            QMessageBox.critical(self, '错误', f'处理 ros2 消息模式文件失败: {str(e)}')
            with open(mode_file, 'w') as f:
                f.write('idle')
            self.statusBar().showMessage('处理 ros2 消息模式文件失败')
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
                    QMessageBox.information(self, '信息', f'已开始录制ROS2 bag: {file_name}')
                    # 更改按钮文本
                    self.ros2_bag_button.setText('停止录制ROS2 bag')
                except Exception as e:
                    QMessageBox.critical(self, '错误', f'启动ROS2 bag录制失败: {str(e)}')
                    self.statusBar().showMessage('启动ROS2 bag录制失败')
            else:
                self.check_ros2_mode("idle")
        else:
            # 停止录制
            if self.check_ros2_mode("idle") == "replay":
                return
            try:
                self.ros2_bag_process.terminate()
                self.ros2_bag_process.wait(timeout=5)
                self.statusBar().showMessage('已停止ROS2 bag录制')
                QMessageBox.information(self, '信息', '已停止ROS2 bag录制')
                # 重置进程变量和按钮文本
                self.ros2_bag_process = None
                self.ros2_bag_button.setText('开始录制ROS2 bag')
            except Exception as e:
                QMessageBox.critical(self, '错误', f'停止ROS2 bag录制失败: {str(e)}')
                self.statusBar().showMessage('停止ROS2 bag录制失败')
                
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
            except Exception as e:
                QMessageBox.critical(self, '错误', f'停止 LCM 日志回放失败: {str(e)}')
                self.statusBar().showMessage('停止 LCM 日志回放失败')
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
                QMessageBox.information(self, '信息', f'已选择日志文件: {file_name}')
                
                # 启动LCM播放器线程
                try:
                    self.lcm_player_thread = LCMPlayerThread(file_name)
                    self.lcm_player_thread.start()
                    self.statusBar().showMessage('已启动 LCM 日志回放')
                    self.open_log_button.setText('停止播放lcm')
                except Exception as e:
                    QMessageBox.critical(self, '错误', f'启动 LCM 日志回放失败: {str(e)}')
                    self.statusBar().showMessage('启动 LCM 日志回放失败')
    
    def on_online_debug(self):
        """在线调试槽函数"""
        self.statusBar().showMessage('在线调试模式')
        QMessageBox.information(self, '信息', '在线调试功能')
        # 这里可以添加在线调试的代码

def main():
    """主函数"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()