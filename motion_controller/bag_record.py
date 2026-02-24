#!/usr/bin/env python3
"""
使用 Python 代码录制 ROS 2 话题到 bag 文件
"""

import rclpy
from rclpy.node import Node
from rosbag2_py import Recorder, RecordOptions


class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        self.get_logger().info('启动 bag 录制器')
        
        # 配置录制选项
        self.options = RecordOptions()
#        self.options.all = False  # 不录制所有话题
        self.options.is_discovery_disabled = False
        self.options.topic_names = ['/cmd_vel', '/odom']  # 指定要录制的话题
        self.options.output = 'my_python_recording'  # 输出文件名前缀
        
        # 创建录制器
        self.recorder = Recorder()
    
    def start_recording(self):
        """开始录制"""
        try:
            self.recorder.open(self.options)
            self.recorder.record()
            self.get_logger().info('开始录制话题: {}'.format(self.options.topic_names))
        except Exception as e:
            self.get_logger().error('录制失败: {}'.format(str(e)))
    
    def stop_recording(self):
        """停止录制"""
        try:
            self.recorder.close()
            self.get_logger().info('停止录制，文件已保存')
        except Exception as e:
            self.get_logger().error('停止录制失败: {}'.format(str(e)))


def main(args=None):
    rclpy.init(args=args)
    
    recorder = BagRecorder()
    recorder.start_recording()
    
    try:
        # 运行 10 秒后停止录制（可根据实际需求调整）
        import time
        time.sleep(10)
    finally:
        recorder.stop_recording()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()