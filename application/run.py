import subprocess
import threading
import time
import json


file_path = '/home/nmsl/python_test/syncFromFile/static.json'
with open(file_path, 'r') as file:
    data = json.load(file)
    updateGranularity = data['updateGranularity']
    endTime = data['endTime']
startTime = 0

ns3_path = "/home/nmsl/ns-3.40/ns-allinone-3.40/ns-3.40/"

while startTime < endTime:
    round = 1
    while 1:

        p1 = subprocess.Popen(['python', 'simpleUAV.py', f'{startTime}'])
        p2 = subprocess.Popen(['python', 'simpleGCS.py', f'{startTime}'])
        p3 = subprocess.Popen(['python', 'client.py', f'{startTime}', f"{round}"])


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
        

        if round > 1:
            file_path1 = f"/home/nmsl/python_test/syncFromFile/msg_logs/message_transmit_log{int(startTime)}_{round}.csv"
            file_path2 = f"/home/nmsl/python_test/syncFromFile/msg_logs/message_transmit_log{int(startTime)}_{round-1}.csv"
            with open(file_path1, 'r') as file:
                row_count1 = sum(1 for line in file)
            with open(file_path2, 'r') as file:
                row_count2 = sum(1 for line in file)
            if row_count1 == row_count2:
                break

        p_ns3 = subprocess.Popen(['./ns3', 'run', 'scratch/syncFormFile/main.cc', '--', f'--timeSlot={int(startTime)}',f'--round={round}'], cwd=ns3_path)
        p_ns3.wait()

        print("NS3 round end")

        round +=1
    startTime += updateGranularity


