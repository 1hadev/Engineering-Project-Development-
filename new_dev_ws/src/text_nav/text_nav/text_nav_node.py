import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import pyttsx3
import math
import os

class TextNav(Node):
    def __init__(self):
        super().__init__('text_nav')

        # TTS setup (optional)
        self.speaker = pyttsx3.init()
        self.speaker.setProperty('rate', 150)

        # Status publisher
        self.status_publisher = self.create_publisher(String, 'status', 10)

        # Subscribe to command topic
        self.command_subscriber = self.create_subscription(
            String,
            'text_nav_command',
            self.command_callback,
            10
        )
# Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Predefined locations (example)
        self.destinations = {
            "door": (0.420722, 0.618713, 0),
            "eating": (2.27117, 0.886419, 0)
        }

        self.get_logger().info("TextNav node started and waiting for commands on /text_nav_command.")
        self.speak("Text navigation node started successfully.")
        self.speak("Hello Professor, I am Ready to Navigate.")


    def command_callback(self, msg):
        # Get the command (destination name)
        destination_name = msg.data.strip().lower()
        #speak command works here??? check
        self.speak("Alright, Going to:" + destination_name)

        if destination_name in self.destinations:
            self.get_logger().info(f"Received command: {destination_name}")
            self.send_goal(destination_name)
        else:
            self.get_logger().warn(f"Unknown destination: {destination_name}")

    def send_goal(self, destination_name):
    # Extract coordinates for the destination
      x, y, yaw = self.destinations[destination_name]
      qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, yaw)

    # Create the goal message
      goal_msg = NavigateToPose.Goal()
      goal_msg.pose.header.frame_id = 'map'
      goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
      goal_msg.pose.pose.position.x = x
      goal_msg.pose.pose.position.y = y
      goal_msg.pose.pose.orientation.x = qx
      goal_msg.pose.pose.orientation.y = qy
      goal_msg.pose.pose.orientation.z = qz
      goal_msg.pose.pose.orientation.w = qw

      self._action_client.wait_for_server()

    # Send the goal
      future = self._action_client.send_goal_async(goal_msg)
      future.add_done_callback(lambda fut: self.goal_response_callback(fut, destination_name))
       
    def euler_to_quaternion(self, roll, pitch, yaw):
      qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
      qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
      qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
      qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
      return (qx, qy, qz, qw)


    def goal_response_callback(self, future, destination_name):
         goal_handle = future.result()

         if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

         self.get_logger().info('Goal accepted')
         result_future = goal_handle.get_result_async()
         result_future.add_done_callback(lambda fut: self.get_result_callback(fut, destination_name))

    def get_result_callback(self, future, destination_name):
        result = future.result().result
        status = future.result().status

        msg = String()

        if status == 3:  # Goal status: succeeded
            self.get_logger().info(f"Reached {destination_name}")
            msg.data = f"Reached {destination_name}"
            
        else:
            self.get.logger().warn(f"Failed to reach {destination_name}")
            msg.data = f"Failed to reach {destination_name}"
            

        self.status_publisher.publish(msg)
        self.speak(msg.data)
  
    def speak(self, text):
      try:
        safe_text = text.replace('"', '')  
        os.system(f'espeak "{safe_text}"')
      except Exception as e:
        self.get_logger().warn(f"TTS failed: {e}")

   #  def speak(self, text):
     #     try:
      #      print(f"Speaking: {text}")
       #     self.speaker.say(text)
        #    self.speaker.runAndWait()
         # except Exception as e:
          #  self.get_logger().warn(f"TTS failed: {e}")
     



def main(args=None):
    rclpy.init(args=args)
    node = TextNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

