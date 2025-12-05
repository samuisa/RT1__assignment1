#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

# ----------------- NODE CLASS DEFINITION -----------------
class SimpleSpawner(Node):
    def __init__(self):
        # 1. NODE INITIALIZATION AND SERVICE CLIENT SETUP
        super().__init__('simple_spawner')
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # 2. WAIT FOR SERVICE AVAILABILITY (BLOCKING)
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for /spawn...")

        # 3. PREPARE AND SEND ASYNCHRONOUS SERVICE REQUEST
        req = Spawn.Request()
        req.x = 7.0
        req.y = 7.0
        req.theta = 0.0
        req.name = "turtle2"

        future = self.spawn_client.call_async(req)
        
        # 4. ATTACH CALLBACK TO HANDLE THE RESPONSE
        future.add_done_callback(self.spawn_callback)

    # ----------------- ASYNCHRONOUS CALLBACK -----------------
    def spawn_callback(self, future):
        # 5. PROCESS SERVICE RESPONSE (SUCCESS OR FAILURE)
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


# ----------------- MAIN EXECUTION BLOCK -----------------
def main(args=None):
    # 6. ROS 2 LIFECYCLE MANAGEMENT (INIT, SPIN, SHUTDOWN)
    rclpy.init(args=args)
    node = SimpleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()