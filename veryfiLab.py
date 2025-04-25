from global_config import CommsConfig, DroneConfig, EnvConfig
from uav import DroneModel
from tqdm import tqdm
import numpy as np
from attacker import AttackerModel
from draw_collect import plot_all_with_comms, record_flight_info_in_csv, record_metrics_in_csv, plot_2d_trajectory, plot_metrics
import os

env = EnvConfig()
# 初始化
attacker = AttackerModel("yaw_attack",
                         pos=env.attacker_pos,
                         hijack_pos=env.attacker_hijack_pos,
                         attack_radius=0,
                         comms_config=CommsConfig())
drone = DroneModel(uav_config=DroneConfig(),
                   comms_config=CommsConfig(),
                   target_pos=[0, 0, 80],  # 初始目标点：悬停高度
                   takeoff_pos=env.drone_takeoff_pos,
                   attacker=attacker)

dt = 0.1  # 按照毫秒级别进行仿真
# 直线航迹
task1 = [
    np.array([00.0, 0.0, 80.0]),  # 悬停高度
    np.array([100.0, 0.0, 20.0]),  # 飞往目标点
    np.array([100.0, 0.0, 0.0]),  # 降落
    # np.array([0.0, 0.0, 0.0]) # 降落
]
# 折线航迹
task2 = [
    np.array([00.0, 0.0, 20.0]),  # 悬停高度
    np.array([50.0, 0.0, 20.0]),  # 飞往目标点
    np.array([100.0, 50.0, 20.0]),  # 飞往目标点
    np.array([100.0, 50.0, 0.0]),  # 降落
    # np.array([0.0, 0.0, 0.0]) # 降落
]
# 三角航迹
task3 = [
    np.array([0.0, 0.0, 20.0]),  # 悬停高度
    np.array([-70.0, 200.0, 20.0]),  # 飞往目标点
    np.array([30.0, 140.0, 20.0]),  # 飞往目标点
    np.array([0.0, 0.0, 20.0]),  # 降落
    np.array([0.0, 0.0, 0.0]),  # 降落
    # np.array([0.0, 0.0, 0.0]) # 降落
]
# 矩形形航迹
task4 = [
    np.array([0.0, 0.0, 20.0]),  # 悬停高度
    np.array([20.0, 100.0, 20.0]),  # 飞往目标点
    np.array([70.0, 90.0, 20.0]),  # 飞往目标点
    np.array([50.0, -10.0, 20.0]),  # 降落
    np.array([0.0, 0.0, 20.0]),  # 降落
    np.array([0.0, 0.0, 0.0]),  # 降落
    # np.array([0.0, 0.0, 0.0]) # 降落
]

tasks = {
    1: task1,
    2: task2,
    3: task3,
    4: task4
}

num_repeats = 50
base_dir = "VerifyLab"

for task_num, task in tasks.items():
    
    task_dir = os.path.join(base_dir, f"Task{task_num}")
    os.makedirs(task_dir, exist_ok=True)

    for repeat in tqdm(range(1, num_repeats + 1), desc=f"Running Task {task_num}"):
        # 重置无人机状态
        drone = DroneModel(uav_config=DroneConfig(),
                           comms_config=CommsConfig(),
                           target_pos=[0, 0, 20],  # 初始目标点：悬停高度
                           takeoff_pos=env.drone_takeoff_pos,
                           attacker=attacker)
        waypoint_index = 0
        drone.target_pos = task[waypoint_index]

        # 运行任务
        for t in range(1200):
            drone.smart_update(dt)

            # 判断是否到达当前目标点
            if drone.judge_position() == 0:
                waypoint_index += 1
                if waypoint_index < len(task):
                    drone.target_pos = task[waypoint_index]  # 设置新的目标点
                else:
                    break  # 完成所有航点，退出循环
            elif drone.judge_position() == 2:
                break
            elif drone.judge_position() == 1:
                break

        # 记录飞行数据
        filename = f"Task{task_num}_No{repeat}.csv"
        filepath = os.path.join(task_dir, filename)
        record_flight_info_in_csv(
            np.array(drone.actual_pos_log),
            np.array(drone.rpm_log),
            np.array(drone.velocity_log),
            np.array(drone.F_total_log),
            np.array(drone.mov_energy_log),
            np.array(drone.comms_energy_log),
            np.array(drone.total_energy_log),
            save_path=filepath
        )

print("实验完成！")