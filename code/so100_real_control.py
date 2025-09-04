import mujoco
import time
import mujoco_viewer
import numpy as np
import math
import zmq
import json

# # ZMQ配置
# ZMQ_IP = "127.0.0.1"  # 绑定到所有可用网络接口
# ZMQ_PORT = "5555"
# context = zmq.Context()
# socket = context.socket(zmq.PUB)
# socket.bind(f"tcp://{ZMQ_IP}:{ZMQ_PORT}")

# print(f"仿真端发布者启动，绑定到：tcp://{ZMQ_IP}:{ZMQ_PORT}")

# ZMQ IPC配置 - 使用进程间通信协议
ZMQ_IPC_PATH = "ipc:///tmp/robot_arm_comm.ipc"  # IPC文件路径
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(ZMQ_IPC_PATH)  # 绑定到IPC路径

print(f"仿真端发布者启动，绑定到：{ZMQ_IPC_PATH}")

def send_data_to_robot(processed_q_deg):
    """将预处理后的关节角通过ZMQ发布"""
    # 数据封装为JSON
    data = {
        "joint_pos": processed_q_deg,  # 关节角（角度）
        "timestamp": time.time(),      # 时间戳（用于同步）
        "control_mode": "position"     # 控制模式（位置控制/速度控制）
    }
    # 转换为JSON字符串并发布
    json_data = json.dumps(data)
    socket.send_string(json_data)

class Test(mujoco_viewer.CustomViewer):
    def __init__(self, path):
        super().__init__(path, 1.5, azimuth=135, elevation=-30)
        self.path = path
    
    def runBefore(self):
        pass
    
    def runFunc(self):
        sim_joint_rad = self.data.qpos[:6].copy()
        sim_joint_deg = [math.degrees(q) for q in sim_joint_rad]
        send_data_to_robot(sim_joint_deg)
        time.sleep(0.01)  # 控制发送频率

try:
    test = Test("./model/trs_so_arm100/scene.xml")
    test.run_loop()
except KeyboardInterrupt:
    print("仿真程序被用户中断")
finally:
    # 关闭ZMQ连接
    socket.close()
    context.term()
