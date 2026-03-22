# -*- coding: utf-8 -*-

import lcm
import time
import sys
import threading
from PySide6.QtCore import QObject, Signal, Qt

# 导入你自定义的 LCM 类型 (确保 PYTHONPATH 包含这些类型所在路径)
# try:
#     from my_robot_msgs import p5_humanoid_t
# except ImportError:
#     print("警告: 未能导入自定义 LCM 类型，将以原始二进制格式回放")

class LCMPlayerThread(threading.Thread, QObject):
    """
    LCM 日志播放线程类
    """
    
    # 定义播放完成信号
    playback_finished = Signal()
    
    def __init__(self, file_path, speed=1.0):
        """
        初始化线程
        :param file_path: .lcm 文件路径
        :param speed: 播放倍速 (1.0 是原速)
        """
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self.file_path = file_path
        self.speed = speed
        self._stop_event = threading.Event()
        self.log = None
        lcm_url = f"udpm://239.255.76.67:56666"
        self.lc = lcm.LCM(lcm_url) # 创建 LCM 实例用于发送数据
    
    def run(self):
        """
        线程运行方法
        """
        try:
            self.log = lcm.EventLog(self.file_path, "r")
        except IOError:
            print(f"无法打开文件: {self.file_path}")
            return

        print(f"开始回放: {self.file_path} (倍速: {self.speed})")
        
        first_timestamp = None
        start_wall_time = time.time()

        try:
            for event in self.log:
                # 检查是否需要停止
                if self._stop_event.is_set():
                    print("\n播放已停止")
                    break
                
                # 初始化起始时间
                if first_timestamp is None:
                    first_timestamp = event.timestamp
                    start_wall_time = time.time()

                # 计算应该等待的时间 (基于日志里的时间戳差值)
                elapsed_log_time = (event.timestamp - first_timestamp) / 1e6 # 微秒转秒
                current_wall_time = time.time() - start_wall_time
                
                # 计算等待时长
                sleep_time = (elapsed_log_time / self.speed) - current_wall_time
                
                if sleep_time > 0:
                    # 在等待时检查是否需要停止
                    if self._stop_event.wait(sleep_time):
                        print("\n播放已停止")
                        break

                # 将数据重新发布到网络
                # event.channel 是原通道名，event.data 是原始 bytes 字节流
                #print(event.channel, event.timestamp)
                # if event.channel == "leg_control_command":
                #     print(event.data)
                # if event.channel == "leg_control_data":
                #     print(event.data)
                #print(event.channel, event.timestamp)
                self.lc.publish(event.channel, event.data)
                
                # # 打印进度 (可选)
                # sys.stdout.write(f"\r正在播放通道: {event.channel:<30} 时间戳: {event.timestamp}")
                # sys.stdout.flush()

        except KeyboardInterrupt:
            print("\n回放已由用户中止")
        finally:
            print("\n播放结束")
            if self.log:
                self.log.close()
            
            # 发送播放完成信号
            self.playback_finished.emit()
    
    def stop(self):
        """
        停止线程
        """
        self._stop_event.set()


def play_lcm_log(file_path, speed=1.0):
    """
    播放 LCM 日志文件
    :param file_path: .lcm 文件路径
    :param speed: 播放倍速 (1.0 是原速)
    """
    player = LCMPlayerThread(file_path, speed)
    player.start()
    return player

if __name__ == "__main__":
    # 使用方式: python3 lcm_player.py my_data.lcm 2.0
    log_file = sys.argv[1] # lcm日志
    play_speed = 1.0
    
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    if len(sys.argv) > 2:
        play_speed = float(sys.argv[2])

    player = play_lcm_log(log_file, play_speed)
    try:
        player.join()
    except KeyboardInterrupt:
        player.stop()
        player.join()