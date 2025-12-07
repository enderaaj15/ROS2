import random
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from raaj_action_interfaces.action import PlaySong


class MusicPlayerServer(Node):
    def __init__(self):
        super().__init__('music_player_server')
        self._action_server = ActionServer(
            self,
            PlaySong,
            'play_song',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.current_goal_handle = None                                               # ADDED
        self.get_logger().info("🎶 Music Player Action Server ready!")

    def goal_callback(self, goal_request):
        song_name = goal_request.song_name

        # If another goal is active
        if self.current_goal_handle and self.current_goal_handle.is_active:            # ADDED
            if goal_request.skip_current:
                # Skip current -> abort old one, accept new
                self.get_logger().warn(
                    f"⏭ Skipping current song and starting new one: {song_name}")
                self.current_goal_handle.abort()                                       # ADDED
                return GoalResponse.ACCEPT
            else:
                # Don't skip -> reject
                self.get_logger().warn(
                    f"🚫 Rejecting '{song_name}' — another song is already playing.")
                return GoalResponse.REJECT

        # Accept if no other goal active
        self.get_logger().info(f"🎧 Accepting new song: {song_name}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("🛑 Received cancel request. Stopping playback...")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.current_goal_handle = goal_handle                                         # ADDED
        song_name = goal_handle.request.song_name
        self.get_logger().info(f"🎵 Now playing: {song_name}")

        vibe_lines = [
            "Vibe check ✅",
            "That beat tho 🔥",
            "Smooth vocals incoming 🎤",
            "Guitar solo moment 🎸",
            "Crowd goes wild 🙌",
            "Dropping the bass 🎶",
            "Chorus hits different 💥",
            "Pure chill vibes 😌",
            "Energy rising ⚡",
            "Outro in progress..."
        ]

        feedback_msg = PlaySong.Feedback()
        total_steps = 50    # 2% per update
        bar_length = 25     # length of visual bar

        for i in range(total_steps + 1):

            # Handle cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn(f"❌ Playback canceled at {i * 2}% progress.")
                return PlaySong.Result(result_message="Song playback canceled.")

            # If goal was aborted (e.g. skipped)
            if not goal_handle.is_active:
                self.get_logger().warn(
                    f"⚠️ {song_name} playback aborted early — another song took over.")
                return PlaySong.Result(result_message="Song skipped or aborted.")

            # Publish feedback
            progress_percent = i * (100 / total_steps)
            feedback_msg.progress_percent = progress_percent
            goal_handle.publish_feedback(feedback_msg)

            # Occasionally drop random lines
            if i % 2 == 0:
                self.get_logger().info(random.choice(vibe_lines))

            # Progress bar
            filled = int(bar_length * (progress_percent / 100))
            bar = "█" * filled + "-" * (bar_length - filled)
            print(f"\r[{bar}] {progress_percent:5.1f}%", end='', flush=True)
            rclpy.spin_once(self, timeout_sec=0.5)

        print()
        if goal_handle.is_active:
            goal_handle.succeed()
            result = PlaySong.Result()
            result.result_message = f"✅ Finished playing: {song_name}"
            self.get_logger().info(result.result_message)
            return result

        # Safety fallback
        self.get_logger().warn(f"⚠️ {song_name} could not finish (inactive goal).")
        return PlaySong.Result(result_message="Song did not complete normally.")


def main(args=None):
    rclpy.init(args=args)
    node = MusicPlayerServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

