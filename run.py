import subprocess
import threading
import time
import json
import os


file_path = './static.json'
current_folder = os.getcwd()
with open(file_path, 'r') as file:
    data = json.load(file)
    updateGranularity = data['updateGranularity']
    endTime = data['endTime']
    ros_path = os.path.join(current_folder, data['ros_path'])
    ns3_path = os.path.join(current_folder, data['ns3_path'])
    exp_path = os.path.join(current_folder, "./msg_logs")
    data['exp_path'] = exp_path
with open(file_path, 'w') as file:
    json.dump(data, file, indent=4)

startTime = 0

while startTime < endTime:
    round = 0
    while 1:

        p1 = subprocess.Popen(['python', ros_path + 'simpleUAV.py', f'-t {startTime}'])
        p2 = subprocess.Popen(['python', ros_path + 'simpleGCS.py', f'-t {startTime}'])
        p3 = subprocess.Popen(['python', ros_path + 'client.py', '-t', str(startTime), '-r', str(round), '-p', exp_path], cwd=ros_path)


        round_start_time = time.monotonic()
        while time.monotonic() - round_start_time < updateGranularity:
            time.sleep(0.1) 

        # 發送 SIGTRAM 強制終止
        p1.terminate()
        p2.terminate()
        p3.terminate()

        p1.wait()
        p2.wait()
        p3.wait()

        
        print("ROS round end")
        

        if round > 0:
            file_path1 = f"{exp_path}/message_transmit_log{int(startTime)}_{round}.csv"
            file_path2 = f"{exp_path}/message_transmit_log{int(startTime)}_{round-1}.csv"
            with open(file_path1, 'r') as file:
                row_count1 = sum(1 for line in file)
            with open(file_path2, 'r') as file:
                row_count2 = sum(1 for line in file)
            if row_count1 == row_count2:
                break

        p_ns3 = subprocess.Popen(['./ns3', 'run', 'scratch/network/main.cc', '--', f'--timeSlot={int(startTime)}', f'--round={round}', f'--expPath={exp_path}'], cwd=ns3_path)
        p_ns3.wait()

        print("NS3 round end")

        round +=1
    startTime += updateGranularity


