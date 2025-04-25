import numpy as np
from scipy.spatial.distance import euclidean
import math
from cal_tools import calculate_distance_3D, point_to_line_distance, add_gaussian_noise,calculate_distance_2D


class DroneModel:
    def __init__(self,  uav_config, comms_config, target_pos, takeoff_pos, attacker, way_points=None):
   
        '''
        无人机模型初始化
        :param m: 无人机质量 (kg)
        :param g: 重力加速度 (m/s^2)
        :param rho: 空气密度 (kg/m^3)
        :param n: 旋翼数量
        :param r: 旋翼半径 (m)
        :param C_T: 推力系数
        :param C_P: 功率系数
        :param C_d: 阻力系数
        :param A_front: 前投影面积 (m^2)
        :param comms_config: 通信配置对象 (可选)
        :param ground_station_pos: 地面站位置 (可选)
        '''
        # 飞行参数
        self.leader = False
        self.is_spoofing = False

        self.m = uav_config.m
        self.g = uav_config.g
        self.rho = uav_config.rho
        self.n = uav_config.n
        self.r = uav_config.r
        self.C_T = uav_config.C_T
        self.C_P = uav_config.C_P
        self.C_d = uav_config.C_d
        self.A_front = uav_config.A_front
        self.way_points_radius = uav_config.waypoint_radius
        self.max_height = uav_config.max_height
        self.min_height = uav_config.min_height
        self.A = np.pi * uav_config.r**2  # 单旋翼面积
        self.deceleration_radius = uav_config.deceleration_radius # 开始减速的阈值距离（米）
        self.velocity_smooth_factor = uav_config.velocity_smooth_factor # 速度跟踪平滑系数

        self.target_pos = np.array(target_pos, dtype=np.float64)  # 目标位置，转换为 NumPy 数组
        self.takeoff_pos = np.array(takeoff_pos, dtype=np.float64)  # 起始位置，转换为 NumPy 数组
        
        # 状态变量初始化
        self.actual_pos = np.array(takeoff_pos, dtype=np.float64)  # 实际位置
        self.last_dt_pos = np.array(takeoff_pos, dtype=np.float64)  # 上一时刻位置, 惯导位置
        self.gps_pos = np.array(takeoff_pos, dtype=np.float64)  # GPS位置
        # self.fake_gps_pos = np.array(takeoff_pos, dtype=np.float64)  # 虚假GPS位置


        self.dist_destination = calculate_distance_3D(self.actual_pos, self.target_pos) # 到达目标位置的距离
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.way_points = way_points if way_points is None else [np.array(wp, dtype=np.float64) for wp in way_points]  # 路径点列表
        self.waypoint_index = 0  # 当前路径点索引

        self.move_energy= 0.0
        self.energy_cost = 0.0

        self.max_rpm = uav_config.max_rpm         # 旋翼最大转速 (RPM)
        self.max_velocity = uav_config.max_velocity  # 最大速度 (m/s)
        self.max_thrust = self.n * self.C_T * self.rho * self.A * (self.max_rpm / 60.0)**2  # 最大推力 (N)
        self.max_rps = self.max_rpm / 60.0  # 最大转速 (rps)、
       
        self.comms = comms_config
        self.comms_active = False
        self.current_tx_power = 0.0
        self.info_record()
        self.metric_record()
        self.noise_std = uav_config.noise_std
        
        # 定义攻击者
        self.attacker = attacker

    
    def info_record(self):
        self.actual_pos_log = []  # 实际位置记录
        self.rpm_log = [] # 转速记录
        self.velocity_log = [] # 速度记录
        # self.F_desired_log = [] # 期望推力记录
        self.F_total_log = [] # 力记录
        self.mov_energy_log = [] # 运动能耗
        self.comms_energy_log = [] # 通信能耗
        self.total_energy_log = [] # 总能耗
        
    
        
    def metric_record(self):
        self.dest_error_log = []  # 目标位置误差记录
        self.drift_error_log = []  # 漂移误差记录
        self.hijack_dist_log = []  # 攻击位置误差记录
        self.attacker_dist_log = []  # 攻击者距离变化记录

    
    def update_metric(self):
        '''更新误差记录'''
        # 目标位置误差
        dist_to_target = calculate_distance_3D(self.actual_pos, self.target_pos)
        self.dest_error_log.append(dist_to_target)
        
        # 漂移误差
        drift_error = point_to_line_distance(self.actual_pos, self.takeoff_pos,self.target_pos)
        self.drift_error_log.append(drift_error)
        
        # 攻击位置误差
        dist_to_attacker = calculate_distance_3D(self.actual_pos, self.attacker. hijack_pos)
        self.hijack_dist_log.append(dist_to_attacker)
        
        # 攻击者距离变化
        dist_to_attacker = calculate_distance_3D(self.actual_pos, self.attacker.pos)
        self.attacker_dist_log.append(dist_to_attacker)
        # print(dist_to_attacker)
        if dist_to_attacker < self.attacker.attacker_radius:
            print(dist_to_attacker)
            self.is_spoofing = True
        
    def smart_update(self, dt):
        '''智能状态更新：根据目标位置自动计算飞行参数'''
        # 获取定位信息及通信能耗
        self.gps_pos, pos_req_energy = self.rec_pos()
        error = self.target_pos - self.gps_pos
        error_norm = np.linalg.norm(error)
        
        # 记录调试信息
        # print(f"Position error: {error}, Norm: {error_norm:.2f}m")
        
        # ========== 速度规划阶段 ==========
        # 计算最大允许速度
        max_thrust = self.n * self.C_T * self.rho * self.A * (self.max_rps**2)
        aero_term = self.rho * self.C_d * self.A_front
        max_speed = min(math.sqrt(2*max_thrust/aero_term), self.max_velocity) if aero_term > 0 else self.max_velocity
        
        # 速度期望值计算（带平滑过渡）
        if error_norm > 1e-3:
            # 基础速度方向
            velocity_direction = error / error_norm
            
            # 根据误差大小调整速度幅值（带减速缓冲）
            speed_ratio = min(error_norm / self.deceleration_radius, 1.0)
            desired_speed = max_speed * speed_ratio
            
            # 速度矢量合成
            velocity_desired = velocity_direction * desired_speed
        else:
            velocity_desired = np.zeros(3)

        # ========== 推力计算阶段 ==========
        # 动态模型计算（考虑速度跟踪）
        accel_desired = (velocity_desired - self.velocity) / dt
        F_required = self.m * (accel_desired + np.array([0, 0, self.g]))  # 包含重力补偿
        
        # 推力限幅处理
        F_norm = np.linalg.norm(F_required)
        if F_norm > max_thrust:
            F_required = F_required * (max_thrust / F_norm)
            # print(f"Thrust limited: {max_thrust:.2f}N")

        # ========== 动力系统计算 ==========
        # 计算螺旋桨转速（RPS）
        if F_norm > 1e-6:
            thrust_direction = F_required / F_norm
            required_rps = np.sqrt(F_norm / (self.n * self.C_T * self.rho * self.A))
        else:
            thrust_direction = np.array([0, 0, 1])  # 默认垂直方向
            required_rps = 0.0

        # 转速限幅
        actual_rps = np.clip(required_rps, 0, self.max_rps)
        rpm = actual_rps * 60  # 转换为RPM
        self.rpm_log.append(rpm)

        # ========== 运动状态更新 ==========
        # 计算实际推力（考虑转速限制）
        T = self.n * self.C_T * self.rho * self.A * actual_rps**2
        F_thrust = T * thrust_direction

        # 空气阻力计算（带数值保护）
        v_norm = np.linalg.norm(self.velocity)
        if v_norm > 1e-3:
            F_drag = -0.5 * self.rho * self.C_d * self.A_front * v_norm * self.velocity
        else:
            F_drag = np.zeros(3)

        # 合力计算
        F_gravity = np.array([0.0, 0.0, -self.m * self.g])
        F_total = F_thrust + F_gravity + F_drag
        self.F_total_log.append(F_total)

        # 运动学更新（带噪声和限幅）
        acceleration = F_total / self.m
        self.velocity += acceleration * dt
        self.velocity = np.clip(add_gaussian_noise(self.velocity, self.noise_std), 
                            -self.max_velocity, 
                            self.max_velocity)
        self.actual_pos += self.velocity * dt

        # ========== 能量计算 ==========
        power = self.n * self.C_P * self.rho * self.A * actual_rps**3
        self.move_energy += power * dt
        self.energy_cost += power * dt + pos_req_energy
        
        # ========== 状态记录 ==========
        self.velocity_log.append(self.velocity.copy())
        self.actual_pos_log.append(self.actual_pos.copy())
        self.mov_energy_log.append(self.move_energy)
        self.comms_energy_log.append(pos_req_energy)
        self.total_energy_log.append(self.energy_cost)
        
        # ========== 状态检查 ==========
        self.update_metric()
        self.judge_position()

    def rec_pos(self):

        if self.is_spoofing:
            # 触发攻击
            self.attacker.catch_victim_info(self.actual_pos, self.target_pos)
            self.gps_pos = self.attacker.cal_compose_pos()
        else:
            # 未触发攻击
            self.gps_pos = self.actual_pos
        distance_ua = self.attacker_dist_log[-1] if self.attacker_dist_log else 0  # 获取最后一个元素，如果列表为空则使用默认值0
        energy_cost = self.pos_req_energy_cost(distance_ua=distance_ua)
        return self.gps_pos, energy_cost

    
    def pos_req_energy_cost(self, distance_ua: float = None):
        if not self.is_spoofing:
            # --------------- 正常卫星定位模式 ---------------
            return np.random.normal(self.comms.gps_request,0.01)  # 直接返回预定义的卫星定位能耗

        else:
        # --------------- 被欺骗模式 ---------------
            # 1. Calculate Path Loss (Ground Station -> UAV)
            distance_km_attacker = distance_ua / 1000.0  # Distance in kilometers
            frequency_ghz = self.comms.frequency / 1e9  # Frequency in GHz

            # Free Space Path Loss (FSPL) in dB
            fspl_attacker = 20 * np.log10(distance_km_attacker) + 20 * np.log10(frequency_ghz) + 32.44  # Corrected FSPL formula

            # Atmospheric Attenuation (Rain and Oxygen) in dB
            rain_loss_attacker = self.comms.alpha_rain * self.comms.rain_rate * distance_km_attacker
            oxygen_loss_attacker = self.comms.alpha_oxygen * distance_km_attacker
            total_loss_attacker = fspl_attacker + rain_loss_attacker + oxygen_loss_attacker

            # Add Noise to Path Loss (in dB)
            noise_std = 0.5  # Noise standard deviation
            noise_attacker = np.random.normal(0, noise_std)
            total_loss_with_noise_attacker = total_loss_attacker + noise_attacker

            # 2. Link Budget Verification (Ground Station -> UAV)
            rx_power_attacker_dbm = (self.comms.tx_power + self.comms.tx_gain +
                                    self.comms.rx_gain - total_loss_with_noise_attacker)

            # 3. Calculate Communication Energy Consumption
            if rx_power_attacker_dbm < self.comms.rx_sensitivity:
                comms_energy_attacker = 0.0  # If signal strength is insufficient, energy consumption is 0
            else:
                # Convert Transmit Power from dBm to Watts
                tx_power_dbm_attacker = rx_power_attacker_dbm - self.comms.rx_sensitivity + self.comms.tx_power
                tx_power_w_attacker = 10 ** ((tx_power_dbm_attacker - 30) / 10)  # dBm to Watts

                # Calculate Transmit and Receive Energy
                tx_time = 0.001  # Transmission time in seconds
                tx_energy_attacker = (tx_power_w_attacker / self.comms.amp_efficiency +
                                    self.comms.circuit_power) * tx_time /1e6  # Convert to Joules
                rx_energy_attacker = self.comms.circuit_power * tx_time
                comms_energy_attacker = tx_energy_attacker + rx_energy_attacker

            # 4. Calculate Energy Consumption for Requesting Signal from Satellite (assumed to be constant)
            comms_energy_satellite = self.comms.gps_request

            # 5. Calculate Total Energy Consumption
            pos_req_energy = comms_energy_attacker + comms_energy_satellite
            return pos_req_energy
        
    def judge_position(self):
        '''判断无人机是否到达目标位置或路径点'''
        dist_to_target = calculate_distance_3D(self.actual_pos, self.target_pos)
        dist_to_hijack = calculate_distance_2D(self.actual_pos, self.attacker.hijack_pos)

        # 检查是否到达路径点
        if self.way_points and self.waypoint_index < len(self.way_points):
            dist_to_waypoint = calculate_distance_2D(self.actual_pos, self.way_points[self.waypoint_index])
            if dist_to_waypoint < self.deceleration_radius:
                if dist_to_waypoint < self.way_points_radius:
                    self.waypoint_index += 1  # 前往下一个路径点
                    return -1  # 尚未到达最终目标
        # 检查是否到达最终目标
        if dist_to_target < self.comms.area_radius:
            return 0
        elif dist_to_hijack < self.comms.area_radius:
            return 2
        elif self.actual_pos[2] < self.min_height :
            return 3
        elif  self.actual_pos[2] > self.max_height:
            return 4
        else:
            return -1
    @property
    def total_power(self):
        flight_power = self.n * self.C_P * self.rho * self.A * (self.current_rps**3)
        comms_power = self.current_tx_power / self.comms.amp_efficiency + self.comms.circuit_power
        return flight_power + comms_power



class FollowerDroneModel(DroneModel):
    def __init__(self, uav_config, comms_config=None, target_pos=None, takeoff_pos=None):
        super().__init__(uav_config, comms_config, target_pos, takeoff_pos)
        self.leader = False
        self.uplink_bandwidth = 1e6    # 上行1MHz
        self.downlink_bandwidth = 5e6  # 下行5MHz
        self.noise_spectral_density = -174  # dBm/Hz
    def calculate_noise_power(self, bandwidth_hz):
        return self.noise_spectral_density + 10 * np.log10(bandwidth_hz)

    def calculate_snr(self, received_power_dbm, bandwidth_hz):
        noise_power = self.calculate_noise_power(bandwidth_hz)
        return received_power_dbm - noise_power

    def _calculate_link_parameters(self, leader_pos, data_size_bits, bandwidth, is_uplink):
        distance_km = calculate_distance_3D(self.actual_pos, leader_pos) / 1000
        rx_sensitivity = (self.comms.uplink_rx_sensitivity if is_uplink 
                         else self.comms.downlink_rx_sensitivity)
        
        # 计算发射功率
        fspl = 20 * np.log10(distance_km * 1000) + 20 * np.log10(self.comms.frequency) - 147.55
        rain_loss = self.comms.alpha_rain * self.comms.rain_rate * distance_km
        oxygen_loss = self.comms.alpha_oxygen * distance_km
        required_tx_power = rx_sensitivity + fspl + rain_loss + oxygen_loss - \
                           self.comms.tx_gain - self.comms.rx_gain
        
        # 计算接收功率与SNR
        rx_power = self.calculate_received_power(distance_km, required_tx_power)
        snr_db = self.calculate_snr(rx_power, bandwidth)
        
        # 数据速率与能耗
        data_rate = self.calculate_data_rate(snr_db, bandwidth)
        tx_time = self.calculate_transmission_time(data_size_bits, data_rate)
        energy = self.calculate_energy_consumption(required_tx_power, tx_time)
        
        return required_tx_power, tx_time, energy
    
class LeaderDroneModel(DroneModel):
    def __init__(self, uav_config, comms_config=None, target_pos=None, takeoff_pos=None):
        super().__init__(uav_config, comms_config, target_pos, takeoff_pos)
        self.leader = True
