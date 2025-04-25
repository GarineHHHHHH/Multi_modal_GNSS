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
                         attacker_radius=env.attacker_radius,
                         comms_config=CommsConfig())

dt = 0.1  # 按照毫秒级别进行仿真
# 直线航迹
task1 = [
    np.array([00.0, 0.0, 0.0]),  # 起飞点
    np.array([00.0, 0.0, 80.0]),  # 悬停高度
    np.array([3000.0, 4000.0, 80.0]),  # 飞往目标点
    # np.array([3000.0, 4000.0, 0.0]),  # 降落
]

tasks = {
    1: task1,
}

num_repeats = 50
base_dir = "yawLab3"
os.makedirs(base_dir, exist_ok=True)

for repeat in tqdm(range(1, num_repeats + 1), desc=f"Running Experiment"):
    # 初始化无人机状态
    drone = DroneModel(uav_config=DroneConfig(),
                       comms_config=CommsConfig(),
                       target_pos=tasks[1][1],  # 初始目标点：悬停高度
                       takeoff_pos=env.drone_takeoff_pos,
                       attacker=attacker,
                       way_points=tasks[1][2:])  # waypoints设为除起飞点和悬停高度外的点
    waypoint_index = 0
    # drone.target_pos = task[waypoint_index]

    # 运行任务
    for t in range(10000):
        drone.smart_update(dt)

        # 判断是否到达目标点
        if drone.judge_position() == 0:
            if waypoint_index < len(drone.way_points):
                waypoint_index += 1
                if waypoint_index < len(drone.way_points):
                    drone.target_pos = drone.way_points[waypoint_index]
                else:
                    drone.target_pos = tasks[1][-1]  # 设置最终目标点
            else:
                break  # 完成所有航点，退出循环
        elif drone.judge_position() == 2:
            break
        elif drone.judge_position() == 1:
            break

    # 记录飞行数据
    flight_records_filename = f"yaw_No{repeat}.csv"
    
    metrics_reords_filename = f"metrics_No{repeat}.csv"
    plot_records_filename = f"plot_No{repeat}.png"
    plot_2d_trajectory_filename = f"2d_No{repeat}.png"
    
    flight_records_filepath = os.path.join(base_dir, "flight_records",flight_records_filename)
    metrics_records_filepath = os.path.join(base_dir, "metrics_records",metrics_reords_filename)
    plot_records_filepath = os.path.join(base_dir, "plot_records",plot_records_filename)
    plot_2d_trajectory_filepath = os.path.join(base_dir, "plot2d_records",plot_2d_trajectory_filename)
    # 创建文件夹
    os.makedirs(os.path.dirname(flight_records_filepath), exist_ok=True)
    os.makedirs(os.path.dirname(metrics_records_filepath), exist_ok=True)
    os.makedirs(os.path.dirname(plot_records_filepath), exist_ok=True)
    os.makedirs(os.path.dirname(plot_2d_trajectory_filepath), exist_ok=True)

    
    
    #  记录和绘制
    record_flight_info_in_csv(
        np.array(drone.actual_pos_log),
        np.array(drone.rpm_log),
        np.array(drone.velocity_log),
        np.array(drone.F_total_log),
        np.array(drone.mov_energy_log),
        np.array(drone.comms_energy_log),
        np.array(drone.total_energy_log),
        save_path=flight_records_filepath
    )
    record_metrics_in_csv(
        np.array(drone.dest_error_log),
        np.array(drone.drift_error_log),
        np.array(drone.hijack_dist_log),
        save_path=metrics_records_filepath
    )

    plot_all_with_comms(
        np.array(drone.takeoff_pos),
        np.array(drone.target_pos),
        np.array(attacker.hijack_pos),
        np.array(drone.actual_pos_log),
        np.array(drone.rpm_log),
        np.array(drone.velocity_log),
        np.array(drone.F_total_log),
        np.array(drone.mov_energy_log),
        np.array(drone.comms_energy_log),
        np.array(drone.total_energy_log),
        save_path=plot_records_filepath
    )

    plot_2d_trajectory(
        np.array(drone.takeoff_pos),
        np.array(drone.target_pos),
        np.array(attacker.hijack_pos),
        np.array(drone.actual_pos_log), save_path=plot_2d_trajectory_filepath
    )

    # plot_metrics(
    #     np.array(drone.dest_error_log),
    #     np.array(drone.drift_error_log),
    #     np.array(drone.hijack_dist_log),
    #     save_path="drone_hijack.png"
    # )
print("实验完成！")