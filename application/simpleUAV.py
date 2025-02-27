#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import subprocess
import time
import signal
import sys

class DroneSimulator:
    def __init__(self):

        rospy.init_node('drone_simple_simulator')
        rospy.loginfo("Drone simulation start")

        self.start_time = time.time()
        self.basic_time = float(sys.argv[1])

        self.last_heartbeat_time = self.start_time
        self.heartbeat_interval = 1.0

        self.pose_pub = rospy.Publisher('/drone/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.heartbeat_pub = rospy.Publisher('/drone/heartbeat', String, queue_size=1, tcp_nodelay=True)

        # 設置發布頻率
        self.rate = rospy.Rate(100)  # Hz
        
        # 軌跡參數
        self.radius = 2.0  # 圓形軌跡半徑
        self.z_height = 2.0  # 飛行高度
        self.angular_speed = 0.5  # 角速度 (rad/s)
        
        # Heartbeat 計數器
        self.heartbeat_count = 0
        
    def publish_heartbeat(self):
        """Publish heartbeat message with precise timing"""

        current_time = time.time()

        # Check if it's time to send the next heartbeat
        time_interval = current_time - self.last_heartbeat_time
        if time_interval >= self.heartbeat_interval:
            self.heartbeat_count += 1
            heartbeat_msg = String()
            sim_time = current_time - self.start_time + self.basic_time 
            heartbeat_msg.data = f"{sim_time:.7f}:Heartbeat"
            self.heartbeat_pub.publish(heartbeat_msg)
            
            self.last_heartbeat_time = current_time  # Reset the timer

        
    def generate_circular_trajectory(self):
        time = 0.0
        
        while not rospy.is_shutdown():

            x = self.radius * math.cos(self.angular_speed * time)
            y = self.radius * math.sin(self.angular_speed * time)
            z = self.z_height
            
            # Create and fill a PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            
            yaw = math.atan2(y, x)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = math.sin(yaw/2)
            pose_msg.pose.orientation.w = math.cos(yaw/2)
            

            self.pose_pub.publish(pose_msg)

            self.publish_heartbeat()
            
            time += 0.1
            self.rate.sleep()

    def shutdown_handler(self, sig, frame):
        rospy.loginfo("Drone simulation completed")
        rospy.signal_shutdown("timeout")
        
    def main(self):
        
        signal.signal(signal.SIGTERM, self.shutdown_handler)
        self.generate_circular_trajectory()

if __name__ == '__main__':
    try:
        simulator = DroneSimulator()
        simulator.main()
        
    except rospy.ROSInterruptException:
        pass
