#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState

class WaitForNode(Node):
    def __init__(self):
        super().__init__('wait_for_node')
        self.declare_parameter('robot_namespace', '')
        self.ns = self.get_parameter('robot_namespace').get_parameter_value().string_value.strip()

        if not self.ns:
            self.get_logger().error('[wait_for_node] robot_namespace parameter must be set')
            sys.exit(1)

        self.client = self.create_client(GetState, f'/{self.ns}/bt_navigator/get_state')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f'[wait_for_node] Waiting for /{self.ns}/bt_navigator/get_state...')

    def timer_callback(self):
        if not self.client.service_is_ready():
            self.get_logger().info(f'[wait_for_node] /{self.ns}/bt_navigator/get_state service is not ready yet...')
            return

        req = GetState.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result()
            state = response.current_state
            self.get_logger().info(f'[wait_for_node] {self.ns}/bt_navigator state: {state.label} ({state.id})')
            if state.id == 3:  # PRIMARY_STATE_ACTIVE
                self.get_logger().info(f'[wait_for_node] {self.ns}/bt_navigator is ACTIVE. Exiting successfully.')
                rclpy.shutdown()
                sys.exit(0)
        except Exception as e:
            self.get_logger().error(f'[wait_for_node] Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WaitForNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
