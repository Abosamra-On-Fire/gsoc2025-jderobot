#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.get_logger().info("Starting Waypoint Navigator")
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Waypoints with simple orientations
        self.waypoints = [
            self.create_waypoint(0.4, 0.0, 0.0),
            self.create_waypoint(0.8, 0.5, 0.0),
            self.create_waypoint(0.0, 0.6, 0.0)
        ]
        self.current_waypoint = 0
        self.send_waypoint()

    def create_waypoint(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = float(yaw)
        pose.pose.orientation.w = 1.0
        return pose

    def send_waypoint(self):
        wp = self.waypoints[self.current_waypoint]
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint+1} at ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = wp
        
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Navigation server not available!")
            rclpy.shutdown()
            return
            
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Progress: {feedback.distance_remaining:.2f}m remaining",
            throttle_duration_sec=1.0)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected! Check:")
            self.get_logger().error("- Robot localization")
            self.get_logger().error("- Obstacle-free path")
            rclpy.shutdown()
            return
            
        self.get_logger().info("Navigation started...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        # Correct way to access error code in Humble
        error_code = result.result.error_code if hasattr(result.result, 'error_code') else -1
        
        if error_code == 0:
            self.get_logger().info("Waypoint reached successfully!")


        self.current_waypoint += 1
        if self.current_waypoint < len(self.waypoints):
            time.sleep(2.0)  # Pause between waypoints
            self.send_waypoint()
        else:
            self.get_logger().info("All waypoints completed!")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()