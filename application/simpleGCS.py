#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import subprocess
import time
import re
import csv
import signal
import sys

class GcsSimulator:
    def __init__(self):
        # 初始化ROS節點
        rospy.init_node('gcs_simple_simulator')
        rospy.loginfo("GCS simulation start")

        self.start_time = time.time()
        self.basic_time = float(sys.argv[1])

        # 創建發布者
        self.pose_pub = rospy.Publisher('/gcs/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.heartbeat_pub = rospy.Publisher('/gcs/heartbeat', String, queue_size=1, tcp_nodelay=True)
        self.heartbeat_ack_pub = rospy.Publisher('/gcs/heartbeat_ack', String, queue_size=1, tcp_nodelay=True)
        
        self.heartbeat_with_delay_sub = rospy.Subscriber(
            '/drone/heartbeat_with_delay',
            String,
            self.heartbeat_callback,
            queue_size=1, 
            tcp_nodelay=True
        )
        # 設置發布頻率
        self.rate = rospy.Rate(100)  # 1Hz
        
        # 軌跡參數
        self.radius = 2.0  # 圓形軌跡半徑
        self.z_height = 2.0  # 飛行高度
        self.angular_speed = 0.5  # 角速度 (rad/s)
        

    def heartbeat_callback(self, msg):
        """Process the received heartbeat message"""
        current_time = rospy.get_time()
            
        sim_time = current_time - self.start_time + self.basic_time 
        # transmit heartbeat ack
        # heartbeat_count = re.search(r'Heartbeat:(\d+)', msg.data).group(1);
        heartbeat_ack_msg = String()
        heartbeat_ack_msg.data = f"{sim_time:.7f}:Heartbeat_ACK"
        self.heartbeat_ack_pub.publish(heartbeat_ack_msg)
        

    def shutdown_handler(self, sig, frame):
        rospy.loginfo("GCS simulation completed")
        rospy.signal_shutdown("timeout")

    def main(self):
        signal.signal(signal.SIGTERM, self.shutdown_handler)
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        simulator = GcsSimulator()
        simulator.main()
        
    except rospy.ROSInterruptException:
        pass