import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus


class NavigateToGoal(Node):
    """Send one fixed Nav2 goal after localization and navigation are ready."""

    def __init__(self):
        super().__init__("navigate_to_goal")
        self.goal_x = -2.4844879132843367
        self.goal_y = 1.7525297248907472
        self.goal_yaw = -0.0002397727340128735
        self.sent = False
        self.finished = False
        self.last_status_log = self.get_clock().now()
        self.last_feedback_log = self.get_clock().now()
        self.have_amcl_pose = False
        self.amcl_pose = None
        self.have_plan = False
        self.have_cmd_vel = False
        self.lifecycle_requests = {}
        amcl_pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.amcl_state_client = self.create_client(
            GetState, "/amcl/get_state"
        )
        self.map_state_client = self.create_client(
            GetState, "/map_server/get_state"
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            amcl_pose_qos,
        )
        self.create_subscription(Path, "/plan", self.plan_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1.0, self.check_ready)
        self.get_logger().info(
            "Waiting for Nav2 action server, active lifecycle nodes, and AMCL pose"
        )

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg.pose.pose
        self.have_amcl_pose = True

    def plan_callback(self, msg):
        if not self.have_plan and msg.poses:
            self.have_plan = True
            self.get_logger().info(
                f"Global path generated: {len(msg.poses)} poses"
            )

    def cmd_vel_callback(self, msg):
        moving = (
            abs(msg.linear.x) > 1.0e-3
            or abs(msg.linear.y) > 1.0e-3
            or abs(msg.angular.z) > 1.0e-3
        )
        if moving and not self.have_cmd_vel:
            self.have_cmd_vel = True
            self.get_logger().info(
                f"Controller velocity command observed: "
                f"vx={msg.linear.x:.3f} vy={msg.linear.y:.3f} "
                f"wz={msg.angular.z:.3f}"
            )

    def check_ready(self):
        if self.sent or self.finished:
            return
        if not self.action_client.server_is_ready():
            self.log_waiting("Nav2 action server is not ready")
            return
        if not self.have_amcl_pose:
            self.log_waiting("AMCL pose is not available")
            return
        if not self.lifecycle_is_active(
            "amcl", self.amcl_state_client
        ) or not self.lifecycle_is_active(
            "map_server", self.map_state_client
        ):
            return
        self.send_goal()

    def lifecycle_is_active(self, name, client):
        if not client.service_is_ready():
            self.log_waiting("%s lifecycle service is not ready", name)
            return False
        if name in self.lifecycle_requests:
            future = self.lifecycle_requests[name]
            if not future.done():
                return False
            del self.lifecycle_requests[name]
            if future.exception() is not None:
                self.get_logger().error(
                    f"Failed to query {name} lifecycle state: "
                    f"{future.exception()}"
                )
                return False
            response = future.result()
            if response.current_state.id != 3:
                self.log_waiting(
                    f"{name} lifecycle state is "
                    f"{response.current_state.label}, waiting for active"
                )
                return False
            return True
        self.lifecycle_requests[name] = client.call_async(GetState.Request())
        return False

    def log_waiting(self, message, *args):
        now = self.get_clock().now()
        if (now - self.last_status_log).nanoseconds >= 5.0e9:
            self.get_logger().info(message % args if args else message)
            self.last_status_log = now

    def send_goal(self):
        self.sent = True
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
        self.get_logger().info(
            f"Sending goal once: frame=map x={self.goal_x:.6f} "
            f"y={self.goal_y:.6f} yaw={self.goal_yaw:.6f}"
        )
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.finished = True
            return
        self.get_logger().info("Goal accepted; navigation is executing")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):
        now = self.get_clock().now()
        if (now - self.last_feedback_log).nanoseconds < 2.0e9:
            return
        self.last_feedback_log = now
        remaining = feedback.feedback.distance_remaining
        self.get_logger().info(
            f"Navigation executing: distance_remaining={remaining:.3f} m"
        )

    def result_callback(self, future):
        status = future.result().status
        labels = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "FAILED (ABORTED)",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        label = labels.get(status, "FAILED (status=%d)" % status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Navigation result: {label}")
        else:
            self.get_logger().error(f"Navigation result: {label}")
        if self.amcl_pose is not None:
            error = math.hypot(
                self.amcl_pose.position.x - self.goal_x,
                self.amcl_pose.position.y - self.goal_y,
            )
            self.get_logger().info(
                f"Final AMCL position=({self.amcl_pose.position.x:.6f}, "
                f"{self.amcl_pose.position.y:.6f}), "
                f"target error={error:.3f} m"
            )
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
