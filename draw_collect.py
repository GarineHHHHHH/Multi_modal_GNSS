import matplotlib.pyplot as plt
import csv
# 设置字体为Times New Roman
# plt.rcParams['font.family'] = 'Times New Roman'
# plt.rcParams['mathtext.fontset'] = 'cm'  # 确保数学公式也使用Times New Roman的风格

def plot_2d_trajectory(takeoff_point, target_point,hijack_point, trajectory, save_path=None, is_show=False):
    """
    绘制二维飞行轨迹，并标注关键点和参考线
    :param trajectory: 飞行轨迹 (Nx3 的 NumPy 数组)
    :param takeoff_point: 起飞点坐标 (可选)
    :param target_point: 目标点坐标 (可选)
    :param hijack_point: 劫持点坐标 (可选)
    :param save_path: 保存路径 (可选)
    """
    plt.figure(figsize=(8, 6))
    plt.plot(trajectory[:, 0], trajectory[:, 1], label="Trajectory")
    plt.title("Drone 2D Flight Trajectory")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")

    # 标注关键点
    if takeoff_point is not None:
        plt.scatter(takeoff_point[0], takeoff_point[1], marker='p', color='blue', s=100, label="Takeoff Point")
    if target_point is not None:
        plt.scatter(target_point[0], target_point[1], marker='o', color='green', s=100, label="Target Point")
    if hijack_point is not None:
        plt.scatter(hijack_point[0], hijack_point[1], marker='^', color='red', s=100, label="Hijack Point")

    # 绘制参考线
    if takeoff_point is not None and target_point is not None:
        plt.plot([takeoff_point[0], target_point[0]], [takeoff_point[1], target_point[1]],
                 linestyle='--', color='green', label="Reference Line")
    
    plt.legend()
    plt.grid()
    if save_path:
        plt.savefig(save_path)
    if is_show:
        plt.show()
def plot_3d_trajectory(trajectory, save_path=None, is_show=False):
    """
    绘制三维飞行轨迹
    :param trajectory: 飞行轨迹 (Nx3 的 NumPy 数组)
    :param save_path: 保存路径 (可选)
    """
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label="Trajectory")
    ax.set_title("Drone 3D Flight Trajectory")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Z Position (m)")
    ax.legend()
    ax.grid()
    if save_path:
        plt.savefig(save_path)
    if is_show:
        plt.show()

def plot_velocity_changes(velocity_log, save_path=None, is_show=False):
    """
    绘制速度变化
    :param velocity_log: 速度记录 (Nx3 的 NumPy 数组)
    :param save_path: 保存路径 (可选)
    """
    time_steps = range(len(velocity_log))
    plt.figure(figsize=(8, 6))
    plt.plot(time_steps, velocity_log[:, 0], label="Vx (m/s)", color="blue")
    plt.plot(time_steps, velocity_log[:, 1], label="Vy (m/s)", color="green")
    plt.plot(time_steps, velocity_log[:, 2], label="Vz (m/s)", color="orange")
    plt.title("Velocity Changes Over Time")
    plt.xlabel("Time Steps")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.grid()
    if save_path:
        plt.savefig(save_path)
    if is_show:
        plt.show()

def plot_energy_consumption(energy_log, save_path=None, is_show=False):
    """
    绘制能耗变化
    :param energy_log: 能耗记录 (N 的 NumPy 数组)
    :param save_path: 保存路径 (可选)
    """
    time_steps = range(len(energy_log))
    plt.figure(figsize=(8, 6))
    plt.plot(time_steps, energy_log, label="Energy Consumption (J)", color="red")
    plt.title("Energy Consumption Over Time")
    plt.xlabel("Time Steps")
    plt.ylabel("Energy (J)")
    plt.legend()
    plt.grid()
    if save_path:
        plt.savefig(save_path)
    if is_show:
        plt.show()

def plot_all(trajectory, velocity_log, energy_log, save_path=None, is_show=False):
    """
    绘制三维飞行轨迹、速度变化和能耗变化在同一个画布中
    :param trajectory: 飞行轨迹 (Nx3 的 NumPy 数组)
    :param velocity_log: 速度记录 (Nx3 的 NumPy 数组)
    :param energy_log: 能耗记录 (N 的 NumPy 数组)
    :param save_path: 保存路径 (可选)
    """
    fig = plt.figure(figsize=(12, 15))

    # 绘制三维飞行轨迹
    ax1 = fig.add_subplot(3, 1, 1, projection='3d')
    ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label="Trajectory")
    ax1.set_title("Drone 3D Flight Trajectory")
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    ax1.set_zlabel("Z Position (m)")
    ax1.legend()
    ax1.grid()

    # 绘制速度变化
    ax2 = fig.add_subplot(3, 1, 2)
    time_steps = range(len(velocity_log))
    ax2.plot(time_steps, velocity_log[:, 0], label="Vx (m/s)", color="blue")
    ax2.plot(time_steps, velocity_log[:, 1], label="Vy (m/s)", color="green")
    ax2.plot(time_steps, velocity_log[:, 2], label="Vz (m/s)", color="orange")
    ax2.set_title("Velocity Changes Over Time")
    ax2.set_xlabel("Time Steps")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.legend()
    ax2.grid()

    # 绘制能耗变化
    ax3 = fig.add_subplot(3, 1, 3)
    ax3.plot(time_steps, energy_log, label="Energy Consumption (J)", color="red")
    ax3.set_title("Energy Consumption Over Time")
    ax3.set_xlabel("Time Steps")
    ax3.set_ylabel("Energy (J)")
    ax3.legend()
    ax3.grid()

    # 显示图像
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path)
    if is_show:
        plt.show()
def record_flight_info_in_csv(trajectory, rpm_log, velocity_log, F_total_log, mov_energy_log, comms_energy_log, total_energy_log, save_path):
    """
    将飞行轨迹、速度、能耗、推力和 RPM 记录保存到 CSV 文件
    :param trajectory: 飞行轨迹 (Nx3 的 NumPy 数组)
    :param rpm_log: RPM 记录 (N 的 NumPy 数组)
    :param velocity_log: 速度记录 (Nx3 的 NumPy 数组)
    :param F_total_log: 推力记录 (Nx3 的 NumPy 数组)
    :param mov_energy_log: 运动能耗记录 (N 的 NumPy 数组)
    :param comms_energy_log: 通信能耗记录 (N 的 NumPy 数组)
    :param total_energy_log: 总能耗记录 (N 的 NumPy 数组)
    :param save_path: 保存路径
    """
    with open(save_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z", "rpm", "vx", "vy", "vz", "F_total_x", "F_total_y", "F_total_z", "move_energy", "comms_energy", "total_energy"])
        for pos, rpm, vel, F_total, move_energy, comms_energy, total_energy in zip(
            trajectory, rpm_log, velocity_log, F_total_log, mov_energy_log, comms_energy_log, total_energy_log
        ):
            writer.writerow([pos[0], pos[1], pos[2], rpm, vel[0], vel[1], vel[2], F_total[0], F_total[1], F_total[2], move_energy, comms_energy, total_energy])
def record_metrics_in_csv(dest_error_log,drift_error_log,hijack_dist_log, save_path):
    """
    记录指标
    :param dest_error_log: 目的地误差
    :param drift_error_log: 漂移误差
    :param hijack_dist_log: 劫持距离
    """
    with open(save_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["dest_error", "drift_error", "hijack_dist"])
        for dest_error, drift_error, hijack_dist in zip(dest_error_log, drift_error_log, hijack_dist_log):
            writer.writerow([dest_error, drift_error, hijack_dist])
    
def plot_all_with_comms(takeoff_point, target_point,hijack_point, trajectory, rpm_log, velocity_log, F_total_log, mov_energy_log, comms_energy_log, total_energy_log, save_path=None, is_show=False):
    """
    绘制三维飞行轨迹、速度变化、总能耗、通信路径总损耗和通信能耗变化
    :param trajectory: 飞行轨迹 (Nx3 的 NumPy 数组)
    :param velocity_log: 速度记录 (Nx3 的 NumPy 数组)
    :param energy_log: 总能耗记录 (N 的 NumPy 数组)
    :param path_loss_log: 通信路径总损耗记录 (N 的 NumPy 数组)
    :param comms_energy_log: 通信能耗记录 (N 的 NumPy 数组)
    :param save_path: 保存路径 (可选)
    :param is_show: 是否显示图像
    """
    fig = plt.figure(figsize=(20, 24))  # 增加画布尺寸

    # 绘制三维飞行轨迹
    ax1 = fig.add_subplot(7, 1, 1, projection='3d')  # 增加画布尺寸
    ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label="Trajectory")
    ax1.set_title("Drone 3D Flight Trajectory", fontsize=16)  # 增加标题字体大小
    ax1.set_xlabel("X Position (m)", fontsize=12)  # 增加轴标签字体大小
    ax1.set_ylabel("Y Position (m)", fontsize=12)
    ax1.set_zlabel("Z Position (m)", fontsize=12)
    ax1.figure.set_size_inches(20, 24) # 设置3D图的尺寸

    # 标注关键点
    if takeoff_point is not None:
        ax1.scatter(takeoff_point[0], takeoff_point[1], takeoff_point[2], marker='p', color='blue', s=100, label="Takeoff Point")
    if target_point is not None:
        ax1.scatter(target_point[0], target_point[1], target_point[2], marker='o', color='green', s=100, label="Target Point")
    if hijack_point is not None:
        ax1.scatter(hijack_point[0], hijack_point[1], hijack_point[2], marker='^', color='red', s=100, label="Hijack Point")

    ax1.legend(fontsize=12)  # 增加图例字体大小
    ax1.grid(True)

    # 绘制速度变化
    ax2 = fig.add_subplot(7, 1, 2)
    time_steps = range(len(velocity_log))
    ax2.plot(time_steps, velocity_log[:, 0], label="Vx (m/s)", color="blue")
    ax2.plot(time_steps, velocity_log[:, 1], label="Vy (m/s)", color="green")
    ax2.plot(time_steps, velocity_log[:, 2], label="Vz (m/s)", color="orange")
    ax2.set_title("Velocity Changes Over Time")
    ax2.set_xlabel("Time Steps")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.legend()
    ax2.grid()

    # 绘制总能耗变化
    ax3 = fig.add_subplot(7, 1, 3)
    ax3.plot(time_steps, mov_energy_log, label="Total Energy Consumption (J)", color="red")
    ax3.set_title("Total Mov Energy Consumption Over Time")
    ax3.set_xlabel("Time Steps")
    ax3.set_ylabel("Energy (J)")
    ax3.legend()
    ax3.grid()


    # 绘制通信能耗变化
    ax5 = fig.add_subplot(7, 1, 4)
    ax5.plot(time_steps, comms_energy_log, label="Communication Energy (J)", color="brown")
    ax5.set_title("Communication Energy Consumption Over Time")
    ax5.set_xlabel("Time Steps")
    ax5.set_ylabel("Energy (J)")
    ax5.legend()
    ax5.grid()
        # 绘制推力变化
    # 绘制 F_desired 变化
    ax6 = fig.add_subplot(7, 1, 5)
    time_steps = range(len(F_total_log))
    ax6.plot(time_steps, F_total_log[:, 0], label="F_total_x (N)", color="blue")
    ax6.plot(time_steps, F_total_log[:, 1], label="F_total_y (N)", color="green")
    ax6.plot(time_steps, F_total_log[:, 2], label="F_total_z (N)", color="orange")
    ax6.set_title("F_total Changes Over Time")
    ax6.set_xlabel("Time Steps")
    ax6.set_ylabel("Force (N)")
    ax6.legend()
    ax6.grid()
    
    ax7 = fig.add_subplot(7, 1, 6)
    time_steps = range(len(rpm_log))
    ax7.plot(time_steps, rpm_log, label="RPM", color="orange")
    ax7.set_title("RPM Changes Over Time")
    ax7.set_xlabel("Time Steps")
    ax7.set_ylabel("RPM")
    ax7.legend()
    ax7.grid()
    
    ax7 = fig.add_subplot(7, 1, 7)
    time_steps = range(len(total_energy_log))
    ax7.plot(time_steps, total_energy_log, label="RPM", color="orange")
    ax7.set_title("Total Energy  Changes Over Time")
    ax7.set_xlabel("Time Steps")
    ax7.set_ylabel("total_energy_log")
    ax7.legend()
    ax7.grid()

    # 调整布局并保存图像
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path)
    if is_show:
        plt.show()
        
def plot_metrics(dest_error_log,drift_error_log,hijack_dist_log, save_path=None, is_show=False):
    '''
    绘制指标
    :param dest_error_log: 目的地误差
    :param drift_error_log: 漂移误差
    :param hijack_dist_log: 劫持距离
    '''

    fig = plt.figure(figsize=(16, 20))
    ax1 = fig.add_subplot(3, 1, 1)
    time_steps = range(len(dest_error_log))
    ax1.plot(time_steps, dest_error_log, label="Destination Error (m)", color="blue")
    ax1.set_title("Destination Error Over Time")
    ax1.set_xlabel("Time Steps")
    ax1.set_ylabel("Error (m)")
    ax1.legend()
    ax1.grid()
    # 绘制漂移误差
    ax2 = fig.add_subplot(3, 1, 2)
    time_steps = range(len(drift_error_log))
    ax2.plot(time_steps, drift_error_log, label="Drift Error (m)", color="green")
    ax2.set_title("Drift Error Over Time")
    ax2.set_xlabel("Time Steps")
    ax2.set_ylabel("Error (m)")
    ax2.legend()
    ax2.grid()
    # 绘制劫持距离  
    
    ax3= fig.add_subplot(3, 1, 3)
    time_steps = range(len(hijack_dist_log))
    ax3.plot(time_steps, hijack_dist_log, label="Hijack Distance (m)", color="orange")
    ax3.set_title("Hijack Distance Over Time")
    ax3.set_xlabel("Time Steps")
    ax3.set_ylabel("Distance (m)")
    ax3.legend()
    ax3.grid()
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path)
    if is_show:
        plt.show()
    
    
    
    