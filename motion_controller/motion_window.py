#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于 Python Qt 的主界面应用程序
"""

import subprocess
import asyncio
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QLineEdit, QComboBox, QHBoxLayout,
    QWidget, QFileDialog, QMessageBox, QStatusBar, QTextEdit, QSplitter, QLabel, QGroupBox,
    QSlider
)
from PySide6.QtCore import Qt, QTime, Signal, QTimer, QEventLoop
import sys
import os
from lcm_player_thread import LCMPlayerThread
import threading
import asyncio
from lcm_tcp_client import LcmTcpClient

class MainWindow(QMainWindow):
    """主窗口类"""
    log_message_updated = Signal(str)
    message_updated = Signal(int, str)  # 新增：更新消息的信号
    ping_timeout_updated = Signal(float)  # 新增：更新消息的信号
    
    def __init__(self):
        super().__init__()
        self.ros2_bag_process = None  # 用于存储 ros2 bag 录制进程
        self.ros2_bag_timer = QTimer()  # 用于ROS2 bag录制计时
        self.ros2_bag_time = 0  # 录制时间（秒）
        self.init_ui()
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        # 连接计时器信号
        self.ros2_bag_timer.timeout.connect(self.update_ros2_bag_timer)
        
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
        # 创建监听客户端
        self.lcm_client = LcmTcpClient()
        
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
        button_widget.setStyleSheet("""
            QWidget {
                background-color: #f8f9fa;
                border-radius: 12px;
                border: 1px solid #e0e0e0;
            }
        """)
        #button_layout = QVBoxLayout(button_widget)
        
        # 主按钮布局 - 使用垂直布局
        button_layout = QVBoxLayout(button_widget)
        button_layout.setSpacing(12)
        button_layout.setContentsMargins(20, 30, 20, 30)
        
        # ========== LCM 日志操作组 ==========
        lcm_group = QGroupBox('LCM 日志操作')
        lcm_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #333;
                border: 1px solid #d0d0d0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 8px 0 8px;
            }
        """)
       # lcm_group.setMinimumHeight(0)  # 单独拉高 LCM 日志播放组的高度
        lcm_layout = QVBoxLayout(lcm_group)
        lcm_layout.setSpacing(12)
        
        # LCM播放按钮
        self.open_log_button = QPushButton('播放 LCM 日志')
        self.open_log_button.setFixedSize(200, 45)
        self.open_log_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        self.open_log_button.clicked.connect(self.on_play_lcm_log)
        lcm_layout.addWidget(self.open_log_button, 0, Qt.AlignCenter)
        
        # 播放控制区域
        control_layout = QHBoxLayout()
        control_layout.setSpacing(8)
        
        # 播放控制按钮
        self.play_button = QPushButton('播放')
        self.play_button.setFixedSize(60, 28)
        self.play_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #ccc;
            }
        """)
        self.play_button.clicked.connect(self.on_play)
        
        self.pause_button = QPushButton('暂停')
        self.pause_button.setFixedSize(60, 28)
        self.pause_button.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
            QPushButton:pressed {
                background-color: #E65100;
            }
            QPushButton:disabled {
                background-color: #ccc;
            }
        """)
        self.pause_button.clicked.connect(self.on_pause)
        self.pause_button.setEnabled(False)
        
        control_layout.addWidget(self.play_button)
        control_layout.addWidget(self.pause_button)
        
        # 播放进度滑块（可手动拖动）
        self.playback_progress = QSlider(Qt.Horizontal)
        self.playback_progress.setFixedHeight(22)
        self.playback_progress.setRange(0, 100)
        self.playback_progress.setValue(0)
        self.playback_progress.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 8px;
                background: #e0e0e0;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #2196F3;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QSlider::sub-page:horizontal {
                background: #2196F3;
                border-radius: 4px;
            }
        """)
        self.playback_progress.sliderMoved.connect(self.on_slider_moved)
        self.playback_progress.sliderPressed.connect(self.on_slider_pressed)
        self.playback_progress.sliderReleased.connect(self.on_slider_released)
        self.is_playback_progress_released = True
        control_layout.addWidget(self.playback_progress)
        
        # 倍速选择
        speed_layout = QHBoxLayout()
        speed_layout.setSpacing(8)
        speed_label = QLabel('播放倍速:')
        speed_label.setStyleSheet('color: #666; font-size: 13px;')
        self.playback_speed_combo = QComboBox()
        self.playback_speed_combo.addItems(['0.25', '0.5', '0.75', '1', '1.25', '1.5', '2', '3'])
        self.playback_speed_combo.setCurrentText('1')
        self.playback_speed_combo.setFixedSize(80, 28)
        self.playback_speed_combo.currentIndexChanged.connect(self.on_speed_changed)
        self.playback_speed_combo.setStyleSheet("""
            QComboBox {
                border: 1px solid #ccc;
                border-radius: 4px;
                padding: 4px;
                font-size: 13px;
            }
        """)
        control_layout.addWidget(speed_label)
        control_layout.addWidget(self.playback_speed_combo)
        
        lcm_layout.addLayout(control_layout)
        
        # LCM转ROS2 Bag按钮（在进度条下方）
        self.lcm_to_bag_button = QPushButton('LCM快速转ROS2 Bag')
        self.lcm_to_bag_button.setFixedSize(200, 45)
        self.lcm_to_bag_button.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
            QPushButton:pressed {
                background-color: #6A1B9A;
            }
        """)
        self.lcm_to_bag_button.clicked.connect(self.on_lcm_convert_ros2_bag)
        lcm_layout.addWidget(self.lcm_to_bag_button, 0, Qt.AlignCenter)
        
        # 输入框区域
        input_layout = QVBoxLayout()
        input_layout.setSpacing(8)
        
        # LCM文件输入框
        lcm_file_layout = QHBoxLayout()
        lcm_file_label = QLabel('LCM文件:')
        lcm_file_label.setFixedWidth(60)
        lcm_file_label.setStyleSheet('color: #555; font-size: 12px;')
        self.lcm_file_edit = QLineEdit()
        self.lcm_file_edit.setPlaceholderText('请输入LCM日志文件路径')
        self.lcm_file_edit.setFixedHeight(28)
        self.lcm_file_edit.setStyleSheet("""
            QLineEdit {
                border: 1px solid #ddd;
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 12px;
            }
            QLineEdit:focus {
                border-color: #9C27B0;
                outline: none;
            }
        """)
        lcm_file_browse = QPushButton('浏览')
        lcm_file_browse.setFixedSize(55, 28)
        lcm_file_browse.setStyleSheet("""
            QPushButton {
                background-color: #f0f0f0;
                color: #333;
                border: 1px solid #ddd;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        lcm_file_browse.clicked.connect(self.on_browse_lcm_file)
        lcm_file_layout.addWidget(lcm_file_label)
        lcm_file_layout.addWidget(self.lcm_file_edit)
        lcm_file_layout.addWidget(lcm_file_browse)
        input_layout.addLayout(lcm_file_layout)
        
        # ROS2 Bag输出路径输入框
        bag_path_layout = QHBoxLayout()
        bag_path_label = QLabel('输出路径:')
        bag_path_label.setFixedWidth(60)
        bag_path_label.setStyleSheet('color: #555; font-size: 12px;')
        self.bag_path_edit = QLineEdit()
        self.bag_path_edit.setPlaceholderText('请输入ROS2 Bag输出路径')
        self.bag_path_edit.setFixedHeight(28)
        self.bag_path_edit.setStyleSheet("""
            QLineEdit {
                border: 1px solid #ddd;
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 12px;
            }
            QLineEdit:focus {
                border-color: #9C27B0;
                outline: none;
            }
        """)
        bag_path_browse = QPushButton('浏览')
        bag_path_browse.setFixedSize(55, 28)
        bag_path_browse.setStyleSheet("""
            QPushButton {
                background-color: #f0f0f0;
                color: #333;
                border: 1px solid #ddd;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        bag_path_browse.clicked.connect(self.on_browse_bag_path)
        bag_path_layout.addWidget(bag_path_label)
        bag_path_layout.addWidget(self.bag_path_edit)
        bag_path_layout.addWidget(bag_path_browse)
        input_layout.addLayout(bag_path_layout)
        
        lcm_layout.addLayout(input_layout)
        
        
        # # 添加底部间距
        # lcm_layout.addStretch(1)
        
        button_layout.addWidget(lcm_group)
        
        # ========== 在线监听组 ==========
        online_group = QGroupBox('在线监听')
        online_group.setMinimumHeight(150)
        online_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #333;
                border: 1px solid #d0d0d0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 8px 0 8px;
            }
        """)
        online_layout = QVBoxLayout(online_group)
        online_layout.setSpacing(0)
        
        # 在线调试按钮
        self.online_debug_button = QPushButton('开始在线监听')
        self.online_debug_button.setFixedSize(200, 45)
        self.online_debug_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
        """)
        self.online_debug_button.clicked.connect(self.on_online_watch)
        online_layout.addWidget(self.online_debug_button, 0, Qt.AlignCenter)
        
        # IP输入框
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText('请输入运控板IP地址')
        self.ip_input.setFixedSize(200, 32)
        self.ip_input.setStyleSheet("""
            QLineEdit {
                border: 1px solid #ccc;
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 13px;
            }
            QLineEdit:focus {
                border-color: #2196F3;
                outline: none;
            }
        """)
        online_layout.addWidget(self.ip_input, 0, Qt.AlignCenter)
        
        # 在在线监听组上方添加间距，使其往下拉
        button_layout.addSpacing(10)
        button_layout.addWidget(online_group)
        
        # ========== ROS2 Bag 操作组 ==========
        ros2_group = QGroupBox('ROS2 Bag 操作')
        ros2_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #333;
                border: 1px solid #d0d0d0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 8px 0 8px;
            }
        """)
        ros2_layout = QVBoxLayout(ros2_group)
        ros2_layout.setSpacing(12)
        
        # CSV快速转换按钮
        self.csv_convert_button = QPushButton('CSV融合LCM快转 ROS2Bag')
        self.csv_convert_button.setFixedSize(200, 45)
        self.csv_convert_button.setStyleSheet("""
            QPushButton {
                background-color: #f87171;
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #ef4444;
            }
            QPushButton:pressed {
                background-color: #dc2626;
            }
        """)
        self.csv_convert_button.clicked.connect(self.on_csv_convert_ros2_bag)
        ros2_layout.addWidget(self.csv_convert_button, 0, Qt.AlignCenter)
        
        # CSV文件加载区域
        csv_files_layout = QVBoxLayout()
        csv_files_layout.setSpacing(12)
        
        # 加载quat文件
        quat_layout = QHBoxLayout()
        self.quat_button = QPushButton('加载quat文件')
        self.quat_button.setFixedSize(100, 28)
        self.quat_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        self.quat_button.clicked.connect(self.on_browse_quat_file)
        self.quat_file_label = QLabel('未选择')
        self.quat_file_label.setStyleSheet('color: #666; font-size: 12px;')
        quat_layout.addWidget(self.quat_button)
        quat_layout.addWidget(self.quat_file_label)
        csv_files_layout.addLayout(quat_layout)
        
        # 加载pos文件
        pos_layout = QHBoxLayout()
        self.pos_button = QPushButton('加载pos文件')
        self.pos_button.setFixedSize(100, 28)
        self.pos_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        self.pos_button.clicked.connect(self.on_browse_pos_file)
        self.pos_file_label = QLabel('未选择')
        self.pos_file_label.setStyleSheet('color: #666; font-size: 12px;')
        pos_layout.addWidget(self.pos_button)
        pos_layout.addWidget(self.pos_file_label)
        csv_files_layout.addLayout(pos_layout)
        
        # 加载vel文件
        vel_layout = QHBoxLayout()
        self.vel_button = QPushButton('加载vel文件')
        self.vel_button.setFixedSize(100, 28)
        self.vel_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        self.vel_button.clicked.connect(self.on_browse_vel_file)
        self.vel_file_label = QLabel('未选择')
        self.vel_file_label.setStyleSheet('color: #666; font-size: 12px;')
        vel_layout.addWidget(self.vel_button)
        vel_layout.addWidget(self.vel_file_label)
        csv_files_layout.addLayout(vel_layout)
        
        # 加载lcmlog文件
        lcmlog_layout = QHBoxLayout()
        self.lcmlog_button = QPushButton('加载lcmlog文件')
        self.lcmlog_button.setFixedSize(100, 28)
        self.lcmlog_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        self.lcmlog_button.clicked.connect(self.on_browse_lcmlog_file)
        self.lcmlog_file_label = QLabel('未选择')
        self.lcmlog_file_label.setStyleSheet('color: #666; font-size: 12px;')
        lcmlog_layout.addWidget(self.lcmlog_button)
        lcmlog_layout.addWidget(self.lcmlog_file_label)
        csv_files_layout.addLayout(lcmlog_layout)
        
        # 状态机编号输入框
        state_machine_layout = QHBoxLayout()
        self.state_machine_label = QLabel('状态机编号:')
        self.state_machine_label.setStyleSheet('color: #666; font-size: 12px;')
        self.state_machine_edit = QLineEdit()
        self.state_machine_edit.setFixedSize(100, 28)
        self.state_machine_edit.setStyleSheet("""
            QLineEdit {
                border: 1px solid #ccc;
                border-radius: 4px;
                padding: 2px 5px;
                font-size: 12px;
            }
        """)
        self.state_machine_edit.setPlaceholderText('输入编号')
        state_machine_layout.addWidget(self.state_machine_label)
        state_machine_layout.addWidget(self.state_machine_edit)
        
        csv_files_layout.addLayout(state_machine_layout)
        # 添加底部拉伸，防止 CSV 文件加载区域的按钮被拉伸
        csv_files_layout.addStretch(1)
        ros2_layout.addLayout(csv_files_layout)
        
        # ROS2录制按钮
        self.ros2_bag_button = QPushButton('开始录制 ROS2 Bag')
        self.ros2_bag_button.setFixedSize(200, 45)
        self.ros2_bag_button.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
            QPushButton:pressed {
                background-color: #E65100;
            }
        """)
        self.ros2_bag_button.clicked.connect(self.on_ros2_bag_record)
        ros2_layout.addWidget(self.ros2_bag_button, 0, Qt.AlignCenter)
        
        # 录制时间标签
        self.ros2_bag_timer_label = QLabel('录制时间: 00:00:00')
        self.ros2_bag_timer_label.setFixedSize(200, 28)
        self.ros2_bag_timer_label.setAlignment(Qt.AlignCenter)
        self.ros2_bag_timer_label.setStyleSheet('color: #666; font-size: 13px;')
        ros2_layout.addWidget(self.ros2_bag_timer_label, 0, Qt.AlignCenter)
        # 添加底部拉伸，防止 ros2_layout 内部部件被拉伸
        ros2_layout.addStretch(1)
        
        button_layout.addWidget(ros2_group)
        # 添加底部拉伸，防止各个 group 在窗口拉伸时自动变高
        button_layout.addStretch(1)
        
        # ========== 创建分割器 ==========
        splitter = QSplitter(Qt.Vertical)
        button_widget.setMinimumHeight(850)  # 增加按钮区域最小高度，避免分组重叠
        button_widget.setMaximumHeight(850)  # 增加最大高度
        splitter.addWidget(button_widget)
        
        # 日志区域样式
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #d4d4d4;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 11pt;
                line-height: 1.4;
                padding: 10px;
                border: 1px solid #3c3c3c;
                border-radius: 8px;
            }
        """)
        splitter.addWidget(self.log_text)
        
        # 设置分割器
        splitter.setSizes([1, 200])  # 同步增加按钮区域初始大小
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        
        # 添加到主布局
        main_layout.addWidget(splitter)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # 记录初始日志
        self.log_message('应用程序已启动')
        
    def on_browse_lcmlog_file(self):
        """浏览lcmlog文件"""
        file_name, _ = QFileDialog.getOpenFileName(
            self, '选择lcmlog文件', '', 'LCM日志文件 (*.log); 所有文件 (*)'
        )
        if file_name:
            self.lcmlog_file_label.setText(os.path.basename(file_name))
            self.lcmlog_file_path = file_name
    
    def on_lcm_convert_ros2_bag(self):
        """将LCM日志文件转换为ROS2 Bag"""
        if not self.lcm_file_edit.text():
            self.log_message('请先选择LCM日志文件')
            return
        
        if not self.bag_path_edit.text():
            self.log_message('请先选择ros2 bag输出目录')
            return
        
        # ========== 完全禁用所有控件槽函数 ==========
        # 1. 处理所有待处理事件，清空事件队列
        QApplication.processEvents(QEventLoop.AllEvents)
        
        # 2. 设置应用程序级别的等待光标
        QApplication.setOverrideCursor(Qt.WaitCursor)
        
        # 3. 禁用主窗口及其所有子控件（这会阻止所有控件接收用户输入事件）
        self.setDisabled(True)
        
        # 4. 再次处理事件确保禁用状态完全生效
        QApplication.processEvents(QEventLoop.AllEvents)

        # 启动lcm 转换 ros2 bag 进程
        self.ros2_bag_process = subprocess.Popen([
            'ros2', 'run', 'lcm_to_ros2', 'lcm_convert_ros2bag', self.lcm_file_edit.text(), self.bag_path_edit.text()],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # 将 stderr 合并到 stdout
            text=True)  # 以文本模式读取
        
        # 等待进程完成并获取输出（会阻塞）
        stdout, _ = self.ros2_bag_process.communicate()

        # 检查返回码
        if self.ros2_bag_process.returncode == 0:
            self.statusBar().showMessage(f'已转换ROS2 bag完成: : { stdout }')
            self.log_message(f'已转换ROS2 bag完成: : { stdout }')
        else:
            self.statusBar().showMessage(f'转换失败: : { stdout }')
            self.log_message(f'转换失败: : { stdout }')
        
        # ========== 恢复所有控件槽函数 ==========
        # 1. 恢复光标
        QApplication.restoreOverrideCursor()
        
        # 2. 启用主窗口及其所有子控件
        self.setDisabled(False)
        
        # 3. 清除禁用期间积累的用户输入事件（关键！确保不会触发延迟的槽函数）
        QApplication.processEvents(QEventLoop.ExcludeUserInputEvents)
        
        # 4. 处理所有剩余事件
        QApplication.processEvents(QEventLoop.AllEvents)
        return
    
    
    def on_browse_lcm_file(self):
        """浏览LCM日志文件"""
        file_name, _ = QFileDialog.getOpenFileName(
            self, '选择LCM日志文件', '', 'LCM日志文件; 所有文件 (*)'
        )
        if file_name:
            self.lcm_file_edit.setText(file_name)
            
    def on_browse_quat_file(self):
        """浏览quat文件"""
        file_name, _ = QFileDialog.getOpenFileName(
            self, '选择quat文件', '', 'CSV文件 (*.csv); 所有文件 (*)'
        )
        if file_name:
            self.quat_file_label.setText(os.path.basename(file_name))
            self.quat_file_path = file_name
    
    def on_browse_pos_file(self):
        """浏览pos文件"""
        file_name, _ = QFileDialog.getOpenFileName(
            self, '选择pos文件', '', 'CSV文件 (*.csv); 所有文件 (*)'
        )
        if file_name:
            self.pos_file_label.setText(os.path.basename(file_name))
            self.pos_file_path = file_name
    
    def on_browse_vel_file(self):
        """浏览vel文件"""
        file_name, _ = QFileDialog.getOpenFileName(
            self, '选择vel文件', '', 'CSV文件 (*.csv); 所有文件 (*)'
        )
        if file_name:
            self.vel_file_label.setText(os.path.basename(file_name))
            self.vel_file_path = file_name
    
    def on_csv_convert_ros2_bag(self):
        """CSV快速转换为ROS2 Bag"""
        # 检查是否已选择文件
        if not hasattr(self, 'quat_file_path') or not self.quat_file_path:
            self.log_message('请先选择quat文件')
            QMessageBox.warning(self, '警告', '请先选择quat文件')
            return
        
        if not hasattr(self, 'pos_file_path') or not self.pos_file_path:
            self.log_message('请先选择pos文件')
            QMessageBox.warning(self, '警告', '请先选择pos文件')
            return
        
        if not hasattr(self, 'vel_file_path') or not self.vel_file_path:
            self.log_message('请先选择vel文件')
            QMessageBox.warning(self, '警告', '请先选择vel文件')
            return
        
        if not hasattr(self, 'lcmlog_file_path') or not self.lcmlog_file_path:
            self.log_message('请先选择LCM日志文件')
            QMessageBox.warning(self, '警告', '请先选择LCM日志文件')
            return
        
        if not self.state_machine_edit or not self.state_machine_edit.text():
            self.log_message('请先输入csv状态机编号')
            QMessageBox.warning(self, '警告', '请先输入csv状态机编号')
            return
        
        # 弹出保存对话框
        file_name, _ = QFileDialog.getSaveFileName(
            self, '保存ROS2 Bag文件', '', 'ROS2 Bag文件 (*)'
        )
        
        if file_name:
            self.log_message(f'开始转换CSV到ROS2 Bag: {file_name}')
            QMessageBox.information(self, '信息', f'开始转换CSV到ROS2 Bag: {file_name}')
            # 这里可以添加实际的转换逻辑
            # ========== 完全禁用所有控件槽函数 ==========
            # 1. 处理所有待处理事件，清空事件队列
            QApplication.processEvents(QEventLoop.AllEvents)
            
            # 2. 设置应用程序级别的等待光标
            QApplication.setOverrideCursor(Qt.WaitCursor)
            
            # 3. 禁用主窗口及其所有子控件（这会阻止所有控件接收用户输入事件）
            self.setDisabled(True)
            
            # 4. 再次处理事件确保禁用状态完全生效
            QApplication.processEvents(QEventLoop.AllEvents)

            # 启动lcm 转换 ros2 bag 进程
            self.ros2_bag_process = subprocess.Popen([
                'ros2', 'run', 'lcm_to_ros2', 'lcm_convert_ros2bag', self.lcmlog_file_path, file_name, self.quat_file_path, self.pos_file_path, self.vel_file_path, self.state_machine_edit.text()],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # 将 stderr 合并到 stdout
                text=True)  # 以文本模式读取
            
            # 等待进程完成并获取输出（会阻塞）
            stdout, _ = self.ros2_bag_process.communicate()

            # 检查返回码
            if self.ros2_bag_process.returncode == 0:
                self.statusBar().showMessage(f'已转换ROS2 bag完成: : { stdout }')
                self.log_message(f'已转换ROS2 bag完成: : { stdout }')
            else:
                self.statusBar().showMessage(f'转换失败: : { stdout }')
                self.log_message(f'转换失败: : { stdout }')
            
            # ========== 恢复所有控件槽函数 ==========
            # 1. 恢复光标
            QApplication.restoreOverrideCursor()
            
            # 2. 启用主窗口及其所有子控件
            self.setDisabled(False)
            
            # 3. 清除禁用期间积累的用户输入事件（关键！确保不会触发延迟的槽函数）
            QApplication.processEvents(QEventLoop.ExcludeUserInputEvents)
            
            # 4. 处理所有剩余事件
            QApplication.processEvents(QEventLoop.AllEvents)
    
    def on_browse_bag_path(self):
        """浏览ROS2 Bag输出路径"""
        file_name, _ = QFileDialog.getSaveFileName(
            self, '选择输出文件', '', 'ROS2 Bag文件 (*)'
        )
 
        if file_name:
            self.bag_path_edit.setText(file_name)
            
    def update_ros2_bag_timer(self):
        """更新ROS2 bag录制时间"""
        self.ros2_bag_time += 1
        hours = self.ros2_bag_time // 3600
        minutes = (self.ros2_bag_time % 3600) // 60
        seconds = self.ros2_bag_time % 60
        self.ros2_bag_timer_label.setText(f'录制时间: {hours:02d}:{minutes:02d}:{seconds:02d}')
        
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
                        'ros2', 'bag', 'record', '-o', file_name, '-e', '^/robot_lcm*'
                    ])
                    self.statusBar().showMessage(f'已开始录制ROS2 bag: {file_name}')
                    self.log_message(f'已开始录制ROS2 bag: {file_name}')
                    QMessageBox.information(self, '信息', f'已开始录制ROS2 bag: {file_name}')
                    # 更改按钮文本
                    self.ros2_bag_button.setText('停止录制ROS2 bag')
                    # 重置并启动计时器
                    self.ros2_bag_time = 0
                    self.ros2_bag_timer_label.setText('录制时间: 00:00:00')
                    self.ros2_bag_timer.start(1000)  # 每1秒更新一次
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
                # 停止计时器
                self.ros2_bag_timer.stop()
                self.statusBar().showMessage('已停止ROS2 bag录制')
                self.log_message('已停止ROS2 bag录制')
                QMessageBox.information(self, '信息', '已停止ROS2 bag录制')
                # 重置进程变量和按钮文本
                self.ros2_bag_process = None
                self.ros2_bag_button.setText('开始录制ROS2 bag')
                # 清零时间显示
                self.ros2_bag_time = 0
                self.ros2_bag_timer_label.setText('录制时间: 00:00:00')
            except Exception as e:
                QMessageBox.critical(self, '错误', f'停止ROS2 bag录制失败: {str(e)}')
                self.statusBar().showMessage('停止ROS2 bag录制失败')
                self.log_message(f'停止ROS2 bag录制失败: {str(e)}')
                # 即使出错也要停止计时器并清零
                self.ros2_bag_timer.stop()
                self.ros2_bag_time = 0
                self.ros2_bag_timer_label.setText('录制时间: 00:00:00')
          
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
      
    def on_lcm_playback_finished(self):
        """LCM日志播放完成槽函数"""
        # 执行模式切换脚本，停止播放模式
        self.execute_script("/opt/ros/motion_studio_mode_switch.sh", "play_lcm", 0)
        # 更新按钮文本
        self.open_log_button.setText('播放lcm日志')
        # 更新播放控制按钮状态
        self.play_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        # 重置进度条
        self.playback_progress.setValue(0)
        # 更新状态栏和日志
        self.statusBar().showMessage('LCM 日志回放已完成')
        self.log_message('LCM 日志回放已完成')
        
    def on_speed_changed(self, index):
        """播放倍速选择框槽函数"""
        self.lcm_player_thread.speed = float(self.playback_speed_combo.currentText())
        self.lcm_player_thread.first_timestamp = 0
        
    def on_slider_pressed(self):
        """进度条按下槽函数"""
        self.is_playback_progress_released = False
        
    def on_slider_moved(self, position):
        """进度条移动槽函数"""
        print(f"进度条位置: {position}")
        self.lcm_player_thread.file_seek = int(position * self.lcm_player_thread.log.size() / 100)
        self.lcm_player_thread.first_timestamp = 0
        self.lcm_player_thread.speed = float(self.playback_speed_combo.currentText())
     #   self.lcm_player_thread.first_timestamp = position
        
    def on_slider_released(self):
        """进度条释放槽函数"""
        self.is_playback_progress_released = True
        
    def on_lcm_progress_updated(self, progress):
        """LCM日志播放进度槽函数"""
     #   print(f"当前进度: {progress:.2f}%")
        if self.is_playback_progress_released:
            self.playback_progress.setValue(int(progress))
        
    def on_play(self):
        """播放按钮槽函数"""
        if self.lcm_player_thread and hasattr(self.lcm_player_thread, 'resume'):
            self.lcm_player_thread.resume()
            self.play_button.setEnabled(False)
            self.pause_button.setEnabled(True)
            self.statusBar().showMessage('LCM 日志回放中')
            self.log_message('LCM 日志继续播放')
    
    def on_pause(self):
        """暂停按钮槽函数"""
        if self.lcm_player_thread and hasattr(self.lcm_player_thread, 'pause'):
            self.lcm_player_thread.pause()
            self.play_button.setEnabled(True)
            self.pause_button.setEnabled(False)
            self.statusBar().showMessage('LCM 日志回放已暂停')
            self.log_message('LCM 日志回放已暂停')
        
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
                    self.lcm_player_thread = LCMPlayerThread(file_name, speed = float(self.playback_speed_combo.currentText()))
                    # 连接播放完成信号
                    self.lcm_player_thread.playback_finished.connect(self.on_lcm_playback_finished, Qt.QueuedConnection)
                    self.lcm_player_thread.progress_updated.connect(self.on_lcm_progress_updated, Qt.QueuedConnection)
                    self.lcm_player_thread.start()
                    self.statusBar().showMessage('已启动 LCM 日志回放')
                    self.log_message('已启动 LCM 日志回放')
                    self.open_log_button.setText('停止播放lcm')
                    # 更新播放控制按钮状态
                    self.play_button.setEnabled(False)
                    self.pause_button.setEnabled(True)
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
            scp_cmd = f'sshpass -p "123" scp -o StrictHostKeyChecking=no -o ConnectTimeout=10 {lcm_turn_server_path} eame@{ip_address}:/home/eame/lcm_turn_server'
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

            #延时五秒等待服务器启动完成
            import time
            time.sleep(5)
            # 创建并启动客户端
            if self.lcm_client.start(server_ip=ip_address, server_port=58888,
                                     message_callback=self.ping_callback,
                                     error_callback=self.error_callback):
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