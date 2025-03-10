#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo

import sys
import os
import csv
import json
import signal
import functools
import time
import argparse

class ROSClient:

    def __init__(self, args):
        rospy.init_node('ros_client')
        
        self.msg_count = -1

        self.start_time = time.time()
        self.timeout_threshold = 1.0  # 1s

        self.basic_time = args.basic_time
        self.round = args.round
        self.time_slot = 0 + int(self.basic_time)
        self.updateGranularity = 0
        self.exp_path = args.exp_path
        
        self.hasInitStatic = 0 # static.json is the ns3 configuration file 
        
        # define CSV path
        if not os.path.exists(self.exp_path):
            os.makedirs(self.exp_path)
            
        self.log_file = f"{self.exp_path}/message_transmit_log{self.time_slot}_{self.round}.csv"

        # Initalize CSV header
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'msg_count',
                'time_diff',      
                'msg_type',       
                'content',
                'sender',
                'receiver'
            ])
            # write open record
            writer.writerow([
                '-1',           # msg_count
                '0',            # time diff between last msg
                'STATUS',       # msg_type (HEARTBEAT/STATUS)
                'START',       # content
                'null',
                'null'
            ])
        rospy.loginfo(f"CSV save to: {self.log_file}")
        
        self.heartbeat_with_delay_pub = rospy.Publisher(
            '/drone/heartbeat_with_delay',
            String,
            queue_size=1,
            tcp_nodelay=True
        )


        # ros subscriber
        self.heartbeat_sub = rospy.Subscriber(
            '/drone/heartbeat',
            String,
            self.heartbeat_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        self.heartbeat_ack_sub = rospy.Subscriber(
            '/gcs/heartbeat_ack', 
            String, 
            self.heartbeat_ack_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        self.heartbeat_ack_sub = rospy.Subscriber(
            '/drone/pose', 
            PoseStamped, 
            self.pose_callback,
            queue_size=1,
            tcp_nodelay=True
        )
        
        self.firefly_pose_sub = rospy.Subscriber(
            "/firefly/transform_stamped", 
            TransformStamped, 
            self.firefly_pose_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        self.firefly_image_sub = rospy.Subscriber(
            f"/nbv/uav_image", 
            Image, 
            self.firefly_image_callback,
            queue_size=1,
            tcp_nodelay=True
        )
        # ros node run rate
        self.rate = rospy.Rate(1000)  


        
    def write_to_csv(self, msg_count=None, time_diff=None, message_type=None, content=None, sender=None, receiver= None):
        """Writing data to a CSV file"""
        try:
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    msg_count,
                    time_diff,
                    message_type,
                    content,       
                    sender,
                    receiver
                ])
        except Exception as e:
            rospy.logerr(f"Failed to write CSV file: {str(e)}")
        
    def heartbeat_ack_callback(self, msg):
        current_time = time.time()
        
        self.msg_count += 1

        # msg_count = re.search(r'Heartbeat_ACK:(\d+)', msg.data).group(1)
        msg_timestamp, msg_content = msg.data.split(":", 1)
        self.write_to_csv(
            msg_count=self.msg_count,
            time_diff=msg_timestamp,
            message_type='HEARTBEAT_ACK',
            content=msg_content,
            sender="gcs",
            receiver="drone1"
        )
        # rospy.loginfo(f"Published {msg_content}, Sim time: {msg_timestamp} seconds")

    
    def heartbeat_callback(self, msg):
        """Process the received heartbeat message"""
        current_time = time.time() 
        self.msg_count += 1

        msg_timestamp, msg_content = msg.data.split(":", 1)
        
        self.write_to_csv(
            msg_count=self.msg_count,
            time_diff=msg_timestamp,
            message_type='HEARTBEAT',
            content=msg_content,
            sender="drone1",
            receiver="gcs"
        )
        # rospy.loginfo(f"Published {msg_content}, Sim time: {msg_timestamp}")

    def pose_callback(self, msg): 
        pass # [cyw]: need to rewrite
    #     """Initialize the static.json to set up the position of drone"""
        
    #     if self.hasInitStatic == 0:
    #         file_path = self.exp_path
    #         with open(file_path, 'r') as file:
    #             data = json.load(file)

    #         # Extract position
    #         position = msg.pose.position
    #         x, y, z = position.x, position.y, position.z
    #         data['uavsPositions'][0] = [x, y, z]
    #         # self.updateGranularity = data['updateGranularity']
            
    #         rospy.loginfo(f"Initialize the position to {data['uavsPositions'][0]}")
            
    #         with open(file_path, 'w') as file:
    #             json.dump(data, file, indent=4)

    #         self.hasInitStatic = 1

    def firefly_pose_callback(self, msg):
        print(msg)

        self.msg_count += 1

        msg_content = {"position_x": msg.transform.translation.x,
                        "position_y": msg.transform.translation.y,
                        "position_z": msg.transform.translation.z,
                        "orientation_x": msg.transform.rotation.x,
                        "orientation_y": msg.transform.rotation.y,
                        "orientation_z": msg.transform.rotation.z,
                        "orientation_w": msg.transform.rotation.w}
        
        msg_timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        self.write_to_csv(
            msg_count=self.msg_count,
            time_diff=msg_timestamp,
            message_type='ODOMETRY',
            content=msg_content,
            sender="drone1",
            receiver="gcs"
        )

    def shutdown_handler(self, sig, frame):
        """Processor program shutdown"""

        rospy.signal_shutdown("Ctrl+C pressed")
        try:
            time_diff = time.time() - self.start_time + self.basic_time
            
            # Write Close Record
            self.write_to_csv(
                msg_count = '-1',
                time_diff=time_diff,
                message_type='STATUS',
                content='END',
                sender="null",
                receiver="null"
            )
            rospy.loginfo("The monitoring program is closed and the last status has been recorded.")
        except Exception as e:
            rospy.logerr(f"Failed to write record on close: {str(e)}")   


    def send_heartbeat(self, msg, event):
        self.heartbeat_with_delay_pub.publish(msg)

    def firefly_image_callback(self,msg):
        pass

    def schedule_message_event(self):
        file_path = f"{self.exp_path}/messageDelayLog{self.time_slot}_{self.round-1}.csv"
        logData = self.read_message_dealy_log(file_path)
        for msg in logData:
            if msg['msg_type'] == 0: # heartbeat, msg_type = 0
                heartbeat_msg = String()
                heartbeat_msg.data = f"Heartbeat:{msg['msg_count']}"
                # self.heartbeat_with_delay_pub.publish(heartbeat_msg)
                # waiting_time = msg['recvTime']
                waiting_time = msg['recvTime'] - self.basic_time
                # print(f"----------------------------{waiting_time}")
                timer = rospy.Timer(
                    rospy.Duration(waiting_time),
                    functools.partial(self.send_heartbeat, heartbeat_msg),
                    oneshot=True
                )



    def read_message_dealy_log(self, file_path):
        data = []
        with open(file_path, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                # 將數值型欄位轉換為對應的類型
                row['msg_count'] = int(row['msg_count'])
                row['msg_type'] = int(row['msg_type'])
                row['sendTime'] = float(row['sendTime'])
                row['recvTime'] = float(row['recvTime'])
                row['delay'] = float(row['delay'])
                data.append(row)
        return data

    def main(self):
        """Turnning on the monitoring program"""
        signal.signal(signal.SIGTERM, self.shutdown_handler)
        
        if self.round > 0:
            self.schedule_message_event()
        
        while not rospy.is_shutdown():
            self.rate.sleep()

def get_parser():
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-r', '--round', default=0, type=int)
    parser.add_argument('-t', '--basic_time', default=0, type=float)
    parser.add_argument('-p', '--exp_path', default='./msg_logs', type=str)
    return parser

if __name__ == '__main__':
    parser = get_parser()
    args = parser.parse_args()
    try:
        monitor = ROSClient(args)
        monitor.main()
    except rospy.ROSInterruptException:
        pass