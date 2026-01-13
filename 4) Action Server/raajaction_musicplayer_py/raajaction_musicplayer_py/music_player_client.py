import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from raaj_action_interfaces.action import PlaySong
import time


class MusicPlayerClient(Node):
    def __init__(self):
        super().__init__('music_player_client')
        self._action_client = ActionClient(self, PlaySong, 'play_song')

    def send_goal(self, song_name: str):
        # Creating goal
        goal_msg = PlaySong.Goal()
        goal_msg.song_name = song_name

        self.get_logger().info("🎧 Waiting for Music Player Server...")
        self._action_client.wait_for_server()
        self.get_logger().info(f"🎶 Sending goal: Play '{song_name}'")

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rejected by server.")
            rclpy.shutdown()
            return

        self.get_logger().info("▶️ Song accepted, playback starting...")
        self._goal_handle = goal_handle

        # Request result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        progress = feedback_msg.feedback.progress_percent
        print(f"\r🎵 Progress: {progress:5.1f}%", end='', flush=True)

    def result_callback(self, future):
        print()
        result = future.result().result
        self.get_logger().info(f"✅ Result: {result.result_message}")
        self.get_logger().info("✨ Music Player Client finished.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MusicPlayerClient()
    # Song request
    node.send_goal("Bohemian Rhapsody")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Client stopped manually.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

