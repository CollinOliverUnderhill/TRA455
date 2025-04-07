#!/usr/bin/env python3
import threading
import time
import math
import rclpy
from rclpy import Parameter
from interfaces.msg import QuadControlTarget, QuadState
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

import matplotlib.pyplot as plt

class WalkingController(Node):
    def __init__(self):
        super().__init__('shape_controller')

        self.quad_state_sub = self.create_subscription(QuadState, 'quad_state', self.quad_state_callback, 1)
        self.quad_control_target_pub = self.create_publisher(QuadControlTarget, 'quad_control_target', 10)
        self.quad_state = None

        self.print_timer = self.create_timer(2.0, self.print_callback)
        self.goal_check_timer = self.create_timer(0.1, self.goal_check_callback)
        self.log_timer = self.create_timer(0.1, self.log_callback)

        self.log_file = "log.txt"
        self.log_header = False
        self.path_trace = []  # 🟢 for path visualization
        with open(self.log_file, "w") as log_file:
            print("Log file cleared")

        self.param_client = self.create_client(SetParameters, '/mit_controller_node/set_parameters')
        self._goal_reached = False

    def quad_state_callback(self, msg):
        self.quad_state = msg

    def print_callback(self):
        pass

    def goal_check_callback(self):
        pass

    def move_square(self):
        self.reset_posture()
        while self.quad_state is None:
            time.sleep(0.1)

        print("Starting square motion (coordinate-based)")
        self.set_gait("WALKING_TROT")

        radius = 0.5  # 1m side
        speed = 0.2
        side_duration = 5
        turn_duration = 3.2  # slightly longer for better alignment

        x0 = self.quad_state.pose.pose.position.x
        y0 = self.quad_state.pose.pose.position.y
        start_time = time.time()
        corners_hit = 0
        tolerance = 0.1

        def distance(x1, y1, x2, y2):
            return math.hypot(x2 - x1, y2 - y1)

        points = [(0, 0), (1, 0), (1, 1), (0, 1)]  # unit square in local frame
        x_start = self.quad_state.pose.pose.position.x
        y_start = self.quad_state.pose.pose.position.y

        for i in range(4):
            print(f"Walking side {i + 1}...")
            self.send_motion(speed, 0.0, 0.0, side_duration)
            print("Turning 90 degrees...")
            self.send_motion(0.0, 0.0, 0.5, turn_duration, step_time=0.05)

        self.stop_motion()

    def move_triangle(self):
        self.reset_posture()
        while self.quad_state is None:
            time.sleep(0.1)

        print("Starting triangle motion (coordinate-based)")
        self.set_gait("WALKING_TROT")

        radius = 0.5  # 1m side
        speed = 0.2
        side_duration = 5
        turn_duration = 4.6  # slightly more than 120 degrees for compensation

        x0 = self.quad_state.pose.pose.position.x
        y0 = self.quad_state.pose.pose.position.y
        tolerance = 0.1

        for i in range(3):
            print(f"Walking side {i + 1}...")
            self.send_motion(speed, 0.0, 0.0, side_duration)
            print("Turning 120 degrees...")
            self.send_motion(0.0, 0.0, 0.5, turn_duration, step_time=0.05)

        self.stop_motion()

    def move_circle(self):
        self.reset_posture()
        while self.quad_state is None:
            time.sleep(0.1)

        print("Starting circular motion (coordinate-based)")
        self.set_gait("WALKING_TROT")

        radius = 0.5
        angular_speed = 0.2
        linear_speed = angular_speed * radius

        cmdMsg = QuadControlTarget()
        cmdMsg.body_x_dot = linear_speed
        cmdMsg.body_y_dot = 0.0
        cmdMsg.world_z = 0.30
        cmdMsg.hybrid_theta_dot = angular_speed
        cmdMsg.pitch = 0.0
        cmdMsg.roll = 0.0

        x0 = self.quad_state.pose.pose.position.x
        y0 = self.quad_state.pose.pose.position.y
        start_time = time.time()
        min_duration = 10  # seconds before checking proximity

        while True:
            x = self.quad_state.pose.pose.position.x
            y = self.quad_state.pose.pose.position.y
            dist = math.hypot(x - x0, y - y0)

            if time.time() - start_time > min_duration and dist < 0.15:
                print("✅ Circle complete based on position.")
                break

            self.quad_control_target_pub.publish(cmdMsg)
            time.sleep(0.01)

        self.stop_motion()

    def set_gait(self, gait_name):
        param_req = SetParameters.Request()
        param_req.parameters = [
            Parameter(name='simple_gait_sequencer.gait', value=gait_name).to_parameter_msg()
        ]
        self.param_client.call_async(param_req)
        time.sleep(1)

    def send_motion(self, x_dot, y_dot, theta_dot, duration, step_time=0.1):
        cmdMsg = QuadControlTarget()
        cmdMsg.body_x_dot = x_dot
        cmdMsg.body_y_dot = y_dot
        cmdMsg.world_z = 0.30
        cmdMsg.hybrid_theta_dot = theta_dot
        cmdMsg.pitch = 0.0
        cmdMsg.roll = 0.0

        steps = int(duration / step_time)
        for _ in range(steps):
            self.quad_control_target_pub.publish(cmdMsg)
            time.sleep(step_time)

    def stop_motion(self):
        print("Stopping motion")
        self.set_gait("STAND")

        cmdMsg = QuadControlTarget()
        cmdMsg.body_x_dot = 0.0
        cmdMsg.body_y_dot = 0.0
        cmdMsg.world_z = 0.30
        cmdMsg.hybrid_theta_dot = 0.0
        cmdMsg.pitch = 0.0
        cmdMsg.roll = 0.0

        # 发布更长时间的站立指令，确保惯性消除
        for _ in range(30):  # 3 秒稳定姿态保持
            self.quad_control_target_pub.publish(cmdMsg)
            time.sleep(0.1)

        print("🧘 Robot fully stabilized.")

    def reset_posture(self):
        print("🔄 Resetting robot posture...")

        # 切换控制器到 WALKING_TROT，然后再 STAND，模拟初始切换过程
        self.set_gait("WALKING_TROT")
        time.sleep(0.5)
        self.set_gait("STAND")
        time.sleep(1.0)

        # 发布一个清零指令但不主动调整 pitch/roll（保持控制器默认）
        cmdMsg = QuadControlTarget()
        cmdMsg.body_x_dot = 0.0
        cmdMsg.body_y_dot = 0.0
        cmdMsg.world_z = 0.30
        cmdMsg.hybrid_theta_dot = 0.0
        cmdMsg.pitch = 0.0
        cmdMsg.roll = 0.0

        for _ in range(10):
            self.quad_control_target_pub.publish(cmdMsg)
            time.sleep(0.1)

        print("✅ Posture reset complete")

        def show_path(self):
           if not self.path_trace:
            print("No path data to visualize.")
            return
        x_vals = [p[0] for p in self.path_trace]
        y_vals = [p[1] for p in self.path_trace]
        plt.figure(figsize=(6,6))
        plt.plot(x_vals, y_vals, marker='o', linestyle='-')
        plt.title("Robot Path")
        plt.xlabel("X position")
        plt.ylabel("Y position")
        plt.axis('equal')
        plt.grid(True)
        plt.show()

    def log_callback(self):
        if self.quad_state is not None:
            with open(self.log_file, "a") as log_file:
                self.path_trace.append((self.quad_state.pose.pose.position.x, self.quad_state.pose.pose.position.y))
                if not self.log_header:
                    log_file.write("x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_ang_vel, y_ang_vel, z_ang_vel\n")
                    self.log_header = True
                log_file.write(
                    f"{self.quad_state.pose.pose.position.x}, {self.quad_state.pose.pose.position.y}, {self.quad_state.pose.pose.position.z}, "
                    f"{self.quad_state.twist.twist.linear.x}, {self.quad_state.twist.twist.linear.y}, {self.quad_state.twist.twist.linear.z}, "
                    f"{self.quad_state.twist.twist.angular.x}, {self.quad_state.twist.twist.angular.y}, {self.quad_state.twist.twist.angular.z}\n"
                )

def main(args=None):
    rclpy.init(args=args)
    walking_controller = WalkingController()  # Initialized only once

    def run_func():
        executor = rclpy.executors.MultiThreadedExecutor(1)
        executor.add_node(walking_controller)
        executor.spin()

    runner = threading.Thread(target=run_func)
    runner.start()

    while rclpy.ok():
        shape = input("Enter shape to walk (square, triangle, circle, path, or quit): ").strip().lower()
        if shape == "square":
            walking_controller.move_square()
        elif shape == "triangle":
            walking_controller.move_triangle()
        elif shape == "circle":
            walking_controller.move_circle()
        elif shape == "path":
            walking_controller.show_path()
        elif shape == "quit":
            break
        else:
            print("Invalid shape entered. Try again.")

    # cleanup only after exiting input loop
    walking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
