#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import csv
import os
import time
import math

class OdomRecorder:
    def __init__(self):
        rospy.init_node('odom_recorder', anonymous=True)
        
        # 参数配置
        self.csv_path = rospy.get_param('~csv_path', 'odom_data.csv')
        self.odom_topic = rospy.get_param('~odom_topic', '/iris_0/mavros/vision_odom/odom')
        
        # 初始化CSV文件
        self.init_csv()
        
        # 创建订阅者
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        rospy.loginfo(f"开始录制Odometry数据到 {self.csv_path}...")

    def init_csv(self):
        # 始终以写入模式打开，覆盖旧文件
        self.csv_file = open(self.csv_path, 'w')  # 修改为'w'模式
        self.writer = csv.writer(self.csv_file)
        
        # 始终写入表头（覆盖模式不需要检查文件存在性）
        header = [
            'timestamp', 'speed_magnitude'  # 保持与之前的列名一致
        ]
        self.writer.writerow(header)
        self.csv_file.flush()
        rospy.loginfo(f"已创建新数据文件: {self.csv_path}")

    def odom_callback(self, msg):
        try:
            # 提取位置信息

            
            # 提取线速度和角速度（保持m/s单位）
            linear_vel = msg.twist.twist.linear
 
            
            # 计算三轴速度矢量模（m/s）
            speed_magnitude = math.sqrt(
                linear_vel.x**2 + 
                linear_vel.y**2 + 
                linear_vel.z**2
            )
            
            # 获取时间戳
            timestamp = msg.header.stamp.to_sec()
            
            # 写入CSV
            row = [
                timestamp,speed_magnitude  # 写入速度矢量模
            ]
            self.writer.writerow(row)
            self.csv_file.flush()
            
        except Exception as e:
            rospy.logerr(f"数据记录错误: {str(e)}")

    def shutdown_hook(self):
        self.csv_file.close()
        rospy.loginfo("文件已安全关闭")

if __name__ == '__main__':
    try:
        recorder = OdomRecorder()
        rospy.on_shutdown(recorder.shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass