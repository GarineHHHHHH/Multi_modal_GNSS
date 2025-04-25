from sympy import symbols, Eq, solve, I
import math
import matplotlib.pyplot as plt
from cal_tools import calculate_distance_2D,calculate_circle_intersection, calculate_line_circle_intersection
import numpy as np

class AttackerModel:
    def __init__(self, attack_type, pos, hijack_pos, attacker_radius,comms_config):
        self.attack_type = attack_type
        self.pos = pos
        self.attacker_radius = attacker_radius # 攻击半径(m)
        
        self.hijack_pos = hijack_pos
        self.emax = comms_config.emax

    def catch_victim_info(self, uav_pos, uav_target_pos):
        self.x_compose = None
        self.y_compose = None
        self.z_compose = None
        self.x_i, self.y_i, self.z_i = uav_pos
        self.x_d, self.y_d, self.z_d = uav_target_pos
        self.x_a_d, self.y_a_d, self.z_a_d = self.hijack_pos
    


    def cal_compose_pos(self):
        # self.z_compose = self.z_i ** 2
        if self.attack_type == "yaw_attack":
            self.z_compose = self.z_i 
            if self.attack_edge():
                # 处理两个圆的交点
                d_i = math.hypot(self.x_i - self.x_d, self.y_i - self.y_d)
                r1 = self.emax
                r2 = math.sqrt(d_i**2 + self.emax**2)
                points = calculate_circle_intersection(self.x_i, self.y_i, r1, self.x_d, self.y_d, r2)
                if not points:
                    return [self.x_i, self.y_i, self.z_i]  # 默认返回当前位置
                # 选择离目标更近的点
                d1 = math.hypot(points[0][0]-self.x_d, points[0][1]-self.y_d)
                d2 = math.hypot(points[1][0]-self.x_d, points[1][1]-self.y_d)
                self.x_compose, self.y_compose = points[0] if d1 <= d2 else points[1]
            else:
                # 处理直线和圆的交点
                dx_dir = self.x_a_d - self.x_i
                dy_dir = self.y_a_d - self.y_i
                points = calculate_line_circle_intersection(self.x_d, self.y_d, dx_dir, dy_dir, 
                                                        self.x_i, self.y_i, self.emax)
                if not points:
                    return [self.x_i, self.y_i, self.z_i]
                # 选择离目标更远的点（与原逻辑一致）
                d1 = math.hypot(points[0][0]-self.x_d, points[0][1]-self.y_d)
                d2 = math.hypot(points[1][0]-self.x_d, points[1][1]-self.y_d)
                self.x_compose, self.y_compose = points[0] if d1 >= d2 else points[1]
            return [self.x_compose, self.y_compose, self.z_compose]
        elif self.attack_type == "height_up":
            self.z_compose = self.z_i **2
            return [self.x_i, self.y_i, self.z_compose]
        elif self.attack_type == "height_down":
            self.z_compose = 0
            return [self.x_i, self.y_i, self.z_compose] 
        elif self.attack_type == "height_flick":
            """
            高度闪烁攻击：在极大和极小高度之间交替。
            """
            max_height = self.z_i*3  # 极大高度
            min_height = 10  # 极小高度（地面）

            # 使用一个简单的逻辑来交替高度
            if hasattr(self, 'flick_state') and self.flick_state == "high":
                self.z_compose = min_height
                self.flick_state = "low"
            else:
                self.z_compose = max_height
                self.flick_state = "high"

            return [self.x_i, self.y_i, self.z_compose]
            
    def attack_edge(self):
        # 保持原逻辑，建议后续验证几何条件正确性
        d_i = math.hypot(self.x_i-self.x_d, self.y_i-self.y_d)
        a_i = math.hypot(self.x_i-self.x_a_d, self.y_i-self.y_a_d)
        l_i = math.hypot(self.x_a_d-self.x_d, self.y_a_d-self.y_d)
        s_i = (d_i + a_i + l_i)*(d_i + a_i - l_i)*(d_i -a_i + l_i)*(a_i + l_i - d_i)
        if s_i < 0:
            return True  # 避免sqrt负数
        edge = math.sqrt(s_i) / (2 * a_i)
        return edge > self.emax

    
    
if __name__ == "__main__":
    from global_config import CommsConfig
    test_object = "up"
    if test_object == "yaw":
        # 测试代码
        attacker = AttackerModel("yaw_attack", pos=[100,50,0], hijack_pos=[200,50,0], emax=100)
        uav_pos = [0, 0, 100]
        uav_target_pos = [300, 300,  100]
        attacker.catch_victim_info(uav_pos, uav_target_pos)
        compose_pos = attacker.cal_compose_pos()
        print("强加位置:", compose_pos)

        # 绘制图形
        plt.figure(figsize=(8, 6))
        plt.plot([uav_pos[0], uav_target_pos[0]], [uav_pos[1], uav_target_pos[1]],
                    linestyle='--', color='green', label="Reference Line")
        plt.scatter(uav_pos[0], uav_pos[1], marker='o', color='blue', s=50, label="UAV Position")
        plt.scatter(uav_target_pos[0], uav_target_pos[1], marker='o', color='green', s=50, label="UAV Target")
        plt.scatter(attacker.pos[0], attacker.pos[1], marker='x', color='red', s=50, label="Attacker Position")
        plt.scatter(attacker.hijack_pos[0], attacker.hijack_pos[1], marker='x', color='purple', s=50, label="Hijack Position")
        plt.scatter(compose_pos[0], compose_pos[1], marker='*', color='orange', s=50, label="Compose Position")

        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("2D Positions and Reference Line")
        plt.legend()
        plt.grid(True)
        plt.savefig("attacker_test_plot.png")  # 保存图片
        # plt.show()
        plt.savefig("attacker_test_plot.png")  # 保存图片
    elif test_object == "up":
        attacker = AttackerModel("height_up", pos=[100,50,0], hijack_pos=[200,50,0],attack_radius=5, comms_config=CommsConfig())
        uav_pos = [0, 0, 100]
        uav_target_pos = [300, 300,  100]
        attacker.catch_victim_info(uav_pos, uav_target_pos)
        compose_pos = attacker.attack_choose()
        print("强加位置:", compose_pos)
        # 绘制3D图形
        plt.figure(figsize=(8, 6))
        ax = plt.axes(projection='3d')
        ax.plot([uav_pos[0], uav_target_pos[0]], [uav_pos[1], uav_target_pos[1]], [uav_pos[2], uav_target_pos[2]],
                    linestyle='--', color='green', label="Reference Line")
        ax.scatter(uav_pos[0], uav_pos[1], uav_pos[2], marker='o', color='blue', s=50, label="UAV Position")
        ax.scatter(uav_target_pos[0], uav_target_pos[1], uav_target_pos[2], marker='o', color='green', s=50, label="UAV Target")
        ax.scatter(attacker.pos[0], attacker.pos[1], attacker.pos[2], marker='x', color='red', s=50, label="Attacker Position")
        ax.scatter(attacker.hijack_pos[0], attacker.hijack_pos[1], attacker.hijack_pos[2], marker='x', color='purple', s=50, label="Hijack Position")
        ax.scatter(compose_pos[0], compose_pos[1], compose_pos[2], marker='*', color='orange', s=50, label="Compose Position")
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_zlabel("Z Position (m)")
        ax.set_title("3D Positions and Reference Line")
        ax.legend()
        ax.grid(True)
        plt.savefig("attacker_test_plot_3d.png")