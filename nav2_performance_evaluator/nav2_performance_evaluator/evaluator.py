#!/usr/bin/env python3

import math
import rclpy
import psutil
from visualization_msgs.msg import Marker
from rclpy.node import Node
from rclpy.action import ActionClient
from transforms3d.euler import quat2euler
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped

class PerformanceEvaluator(Node):

    def __init__(self):
        super().__init__("performance_evaluator")

        self.get_logger().info("Performance Evaluator Started")


        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10
        )

        self.plan_sub = self.create_subscription(
            Path,
            "/plan",
            self.plan_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/goal_marker",
            10
        )

        self.previous_yaw = None
        self.total_rotation = 0.0

        # Executed Path Length (from AMCL)
        self.executed_path_length = 0.0
        self.previous_x = None
        self.previous_y = None

        self.initial_planned_path_length = 0.0

        self.first_plan_received = False
        self.replan_count = 0

        # navigation Time Calculation Variables
        self.start_time = None

        # Planning Time Calculation Variables 
        self.planning_start_time = None
        self.planning_end_time = None

        self.planning_finished = False 

        #CPU & Memory Usage Calculation Variables
        self.processes = []

        keywords = [
            "planner_server",
            "controller_server",
            # "bt_navigator",
            # "behavior_server",
            # "amcl",
        ]

        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmd = " ".join(p.info['cmdline'])
                
                if any(k in cmd for k in keywords):
                    proc = psutil.Process(p.info['pid'])
                    proc.cpu_percent(interval=None)   # initialize
                    self.processes.append(proc)

                    self.get_logger().info(
                        f"Monitoring {cmd} (PID {proc.pid})")

            except Exception:
                pass

        if len(self.processes) == 0:
            self.get_logger().error("No Nav2 processes found!")


        self.cpu_samples = []
        self.memory_samples = []

# Initialize CPU measurement for every monitored process
        for proc in self.processes:
             proc.cpu_percent(interval=None)

        self.resource_timer = self.create_timer(
            0.5,
            self.resource_callback
        )

        self.goal = (
            -4.72791,
            18.3567,
            0.0
        )

        self.get_logger().info("Waiting for Nav2...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 Connected!")

        self.create_timer(1.0, self.publish_goal_marker)

        self.start_navigation()
    
    def quaternion_from_yaw(self, yaw):

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        return qz, qw


    def create_goal(self, x, y, yaw):

        goal = NavigateToPose.Goal()

        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        qz, qw = self.quaternion_from_yaw(yaw)

        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal.pose = pose

        return goal

    def publish_goal_marker(self):
        
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "goal"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = -4.72791
        marker.pose.position.y = 18.3567
        marker.pose.position.z = 0.2

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


    def start_navigation(self):

        self.start_time = self.get_clock().now()

        self.planning_start_time = self.get_clock().now()
        self.planning_finished = False

        self.first_plan_received = False
        self.replan_count = 0
        self.initial_planned_path_length = 0.0

        self.get_logger().info("")

        self.get_logger().info("Mission Started")

        goal = self.create_goal(*self.goal)

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:

            self.get_logger().error("Goal Rejected")
            return

        self.get_logger().info("Goal Accepted")

        result_future = self.goal_handle.get_result_async()

        result_future.add_done_callback(self.result_callback)


    def feedback_callback(self, feedback_msg):

        pass

    def result_callback(self, future):
        
        result = future.result()

        status = result.status

        if status != GoalStatus.STATUS_SUCCEEDED:
            
            self.get_logger().error("Navigation Failed!")
            return

        self.end_time = self.get_clock().now()

        navigation_time = (
            self.end_time - self.start_time
        ).nanoseconds / 1e9

        self.get_logger().info("")
        self.get_logger().info("MISSION COMPLETED SUCCESSFULLY")

        average_cpu = (
            sum(self.cpu_samples) / len(self.cpu_samples)
            if self.cpu_samples else 0.0
        )

        average_ram = (
            sum(self.memory_samples) / len(self.memory_samples)
            if self.memory_samples else 0.0
        )

        peak_ram = (
            max(self.memory_samples)
            if self.memory_samples else 0.0
        )

        self.get_logger().info(
            f"Total Navigation Time : {navigation_time:.3f} seconds"
        )

        self.get_logger().info(
            f"Planning Time : {self.planning_time:.3f} ms"
        )

        self.get_logger().info(
            f"Initial Planned Path Length : {self.initial_planned_path_length:.3f} m"
        )

        self.get_logger().info(
            f"Replan Count : {self.replan_count}"
        )

        self.get_logger().info(
            f"Executed Path Length : {self.executed_path_length:.3f} m")

        self.get_logger().info(
            f"Average CPU Usage : {average_cpu:.2f}%"
        )

        self.get_logger().info(
            f"Average Memory Usage : {average_ram:.2f} MiB"
        )
        
        average_linear_velocity = (
            self.executed_path_length / navigation_time
            if navigation_time > 0.0 else 0.0
        )

        self.get_logger().info(
            f"Average Linear Velocity : {average_linear_velocity:.3f} m/s"
        )

        average_angular_velocity = (
            self.total_rotation / navigation_time
            if navigation_time > 0.0 else 0.0
        )

        self.get_logger().info(
            f"Average Angular Velocity : {average_angular_velocity:.3f} rad/s"
        )

        rclpy.shutdown()

        self.resource_timer.cancel()

    def amcl_callback(self, msg):

        # Don't calculate anything before the mission starts
        if self.start_time is None:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        if self.previous_yaw is not None:
            
            delta = yaw - self.previous_yaw

            while delta > math.pi:
                delta -= 2.0 * math.pi

            while delta < -math.pi:
                delta += 2.0 * math.pi
            
            self.total_rotation += abs(delta)

        self.previous_yaw = yaw

        if self.previous_x is not None:
            
            distance = math.sqrt(
                (x - self.previous_x) ** 2 +
                (y - self.previous_y) ** 2
            )

            self.executed_path_length += distance

        self.previous_x = x
        self.previous_y = y
    
    def plan_callback(self, msg):
        
        if not self.planning_finished and self.planning_start_time is not None:
            self.planning_end_time = self.get_clock().now()

            self.planning_time = (
                self.planning_end_time -
                self.planning_start_time
            ).nanoseconds / 1e6

            self.planning_finished = True

            self.get_logger().info(
                f"Planning Time : {self.planning_time:.3f} ms"
            )

        current_length = 0.0

        if len(msg.poses) >= 2:
            
            for i in range(1, len(msg.poses)):
                p1 = msg.poses[i - 1].pose.position
                p2 = msg.poses[i].pose.position
                
                current_length += math.hypot(
                    p2.x - p1.x,
                    p2.y - p1.y
                )

        if not self.first_plan_received:
            self.first_plan_received = True
            self.initial_planned_path_length = current_length
            
            self.get_logger().info(
                f"Initial Planned Path Length : {self.initial_planned_path_length:.3f} m")

        else:
            self.replan_count += 1

            self.get_logger().info(
                f"Replan #{self.replan_count}")

    def resource_callback(self):
        
        if self.start_time is None:
            return
        
        total_cpu = 0.0
        total_memory = 0.0

        for proc in self.processes:
            try:
                total_cpu += proc.cpu_percent(interval=None)
                total_memory += proc.memory_info().rss / (1024 * 1024)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        self.cpu_samples.append(total_cpu)
        self.memory_samples.append(total_memory)

        # self.cpu_samples.append(cpu)
        # self.memory_samples.append(memory)


def main(args=None):

    rclpy.init(args=args)

    node = PerformanceEvaluator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()


