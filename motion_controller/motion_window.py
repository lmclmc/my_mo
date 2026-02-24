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

from lcm_player_thread import LCMPlayerThread

class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self):
        super().__init__()
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
        open_log_button = QPushButton('打开日志', self)
        open_log_button.setFixedSize(200, 50)
        open_log_button.clicked.connect(self.on_open_log)
        layout.addWidget(open_log_button, 0, Qt.AlignCenter)
        
        # 创建在线调试按钮
        online_debug_button = QPushButton('在线调试', self)
        online_debug_button.setFixedSize(200, 50)
        online_debug_button.clicked.connect(self.on_online_debug)
        layout.addWidget(online_debug_button, 0, Qt.AlignCenter)
        
        # 设置布局间距
        layout.setSpacing(30)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
    
    def on_open_log(self):
        """打开日志槽函数"""
        # 弹出文件选择对话框
        file_name, _ = QFileDialog.getOpenFileName(
            self, '打开日志文件', '', '所有文件 (*)'
        )
        if file_name:
            self.statusBar().showMessage(f'已选择日志文件: {file_name}')
            QMessageBox.information(self, '信息', f'已选择日志文件: {file_name}')
            
            # 停止之前的线程（如果存在）
            if self.lcm_player_thread and self.lcm_player_thread.is_alive():
                self.lcm_player_thread.stop()
                self.lcm_player_thread.join()
            
            # 启动LCM播放器线程
            try:
                self.lcm_player_thread = LCMPlayerThread(file_name)
                self.lcm_player_thread.start()
                self.statusBar().showMessage('已启动 LCM 日志回放')
            except Exception as e:
                QMessageBox.critical(self, '错误', f'启动 LCM 日志回放失败: {str(e)}')
                self.statusBar().showMessage('启动 LCM 日志回放失败')
            
        # 运行 ros2 run plotjuggler plotjuggler
        try:
            subprocess.Popen(['ros2', 'run', 'plotjuggler', 'plotjuggler'])
            self.statusBar().showMessage('已启动 PlotJuggler')
        except Exception as e:
            QMessageBox.critical(self, '错误', f'启动 PlotJuggler 失败: {str(e)}')
            self.statusBar().showMessage('启动 PlotJuggler 失败')
    
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