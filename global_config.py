# 定义光速
c = 3e8  # 光速 (m/s)
class CommsConfig:
    def __init__(self):
        # 基础通信参数
        self.frequency = 2.4e9          # 载波频率 (Hz)
        self.tx_gain = 3.0              # 发射天线增益 (dBi)
        self.rx_gain = 2.0              # 接收天线增益 (dBi)
        self.rx_sensitivity = -90       # 接收灵敏度 (dBm)
        self.amp_efficiency = 0.4       # 功放效率 (0.4表示40%)
        self.circuit_power = 2.0        # 电路基础功耗 (W)
        self.alpha_rain = 0.01          # 降雨衰减系数 (dB/km/mm/h)
        self.alpha_oxygen = 0.03        # 氧气吸收系数 (dB/km)
        self.rice_factor = 10.0         # 莱斯因子（线性值）
        self.rain_rate = 0.0            # 当前降雨率 (mm/h)
        self.gps_request = 0.165        # 卫星定位单次能耗（焦耳）
        self.tx_power = 30.0            # 发射功率 (dBm)
        
        # 新增卫星参数
        self.satellite_tx_power = 47.0   # 卫星发射功率 (dBm)
        self.satellite_frequency = 1.57542e9  # 卫星信号频率 (GPS L1频段)
        self.satellite_distance = 20200e3  # 卫星距离 (米)
        self.emax = 1000
        self.area_radius = 10
        
        
class DroneConfig:
    def __init__(self):
        self.m = 0.5 
        self.g = 9.81
        self.rho = 1.225
        self.n = 4                  # 保持四旋翼配置
        self.r = 0.08               # 减小桨叶半径以降低阻力 (m)
        self.C_T = 0.12             # 优化后的推力系数（原0.08）
        self.C_P = 0.05             # 调整功率系数（原0.04）
        self.max_rps = 140          # 最大转速（转/秒）对应7200 RPM
        # 空气动力学参数
        self.C_d = 0.4             # 优化流线型设计（原0.6）
        self.A_front = 0.06         # 减小迎风面积（原0.1）(m²)
        
        self.max_rpm = 7200
        self.deceleration_radius = 2.0
        self.velocity_smooth_factor = 0.1
        self.max_velocity = 5.0  # 最大速度 (m/s)
        
        self.noise_std = 0.5
        self.waypoint_radius = 5
        
        self.max_height = 120
        self.min_height = 50
        # self.max

class EnvConfig:
    def __init__(self):
        self.attacker_pos = [1800.0, 2400.0, 0.0]
        self.attacker_hijack_pos = [1800, 3400, 0]
        self.attacker_radius = 1000

        self.drone_takeoff_pos = [0, 0, 80]
        # self.drone_target_pos = [100, 100, 20]
        