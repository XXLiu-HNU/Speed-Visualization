#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import csv
import os
import time

class OdomRecorder:
    def __init__(self):
        rospy.init_node('odom_recorder', anonymous=True)
        
        # 参数配置
        self.csv_path = rospy.get_param('~csv_path', 'odom_data.csv')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        
        # 初始化CSV文件
        self.init_csv()
        
        # 创建订阅者
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        rospy.loginfo(f"开始录制Odometry数据到 {self.csv_path}...")

    def init_csv(self):
        file_exists = os.path.isfile(self.csv_path)
        self.csv_file = open(self.csv_path, 'a')
        self.writer = csv.writer(self.csv_file)
        
        if not file_exists:
            header = [
                'timestamp', 
                'pos_x', 'pos_y', 'pos_z',
                'vel_x', 'vel_y', 'vel_z',
                'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                'vel_x_kmh'
            ]
            self.writer.writerow(header)
            self.csv_file.flush()

    def odom_callback(self, msg):
        try:
            # 提取位置信息
            position = msg.pose.pose.position
            
            # 提取线速度和角速度（m/s）
            linear_vel = msg.twist.twist.linear
            angular_vel = msg.twist.twist.angular
            
            # 转换线速度为km/h
            vel_x_kmh = linear_vel.x * 3.6  # m/s -> km/h
            
            # 获取时间戳
            timestamp = msg.header.stamp.to_sec()
            
            # 写入CSV
            row = [
                timestamp,
                position.x, position.y, position.z,
                linear_vel.x, linear_vel.y, linear_vel.z,
                angular_vel.x, angular_vel.y, angular_vel.z,
                vel_x_kmh
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