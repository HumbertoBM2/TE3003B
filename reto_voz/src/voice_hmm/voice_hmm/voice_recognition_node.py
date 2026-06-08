import os
import threading
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from ament_index_python.packages import get_package_share_directory

from .read_audio import read_wav
from .record_audio import record_audio_arecord
from .recognizer_core import HMMVoiceRecognizer


class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__("voice_recognition_node")

        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("models_dir", "")
        self.declare_parameter("input_mode", "arecord")
        self.declare_parameter("alsa_device", "plughw:0,6")
        self.declare_parameter("record_duration", 2.0)
        self.declare_parameter("live_wav_path", "/tmp/voice_hmm_live.wav")
        self.declare_parameter("reject_word", "ninguna")
        self.declare_parameter("min_score_margin", 12.0)
        self.declare_parameter("min_audio_peak", 0.01)
        self.declare_parameter("words", [
            "avanza",
            "retrocede",
            "derecha",
            "izquierda",
            "alto",
            "empieza",
            "sube",
            "baja",
            "gira",
            "busca"
        ])

        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.words = list(self.get_parameter("words").value)
        self.input_mode = str(self.get_parameter("input_mode").value)
        self.alsa_device = str(self.get_parameter("alsa_device").value)
        self.record_duration = float(self.get_parameter("record_duration").value)
        self.live_wav_path = str(self.get_parameter("live_wav_path").value)
        self.reject_word = str(self.get_parameter("reject_word").value)
        self.min_score_margin = float(self.get_parameter("min_score_margin").value)
        self.min_audio_peak = float(self.get_parameter("min_audio_peak").value)

        models_dir = self.get_parameter("models_dir").value

        if models_dir == "":
            package_share = get_package_share_directory("voice_hmm")
            models_dir = os.path.join(package_share, "models")

        self.get_logger().info(f"Loading models from: {models_dir}")
        self.get_logger().info(f"Words: {self.words}")
        self.get_logger().info(f"Input mode: {self.input_mode}")
        self.get_logger().info(f"Reject word: {self.reject_word}")

        self.recognizer = HMMVoiceRecognizer(models_dir, self.words)

        self.listening = False
        self.audio_buffer = []
        self.recording_thread = None

        self.listen_sub = self.create_subscription(
            Bool,
            "/voice/listen_flag",
            self.listen_callback,
            10
        )

        self.audio_sub = None

        if self.input_mode == "topic":
            self.audio_sub = self.create_subscription(
                Int16MultiArray,
                "/voice/audio_chunk",
                self.audio_callback,
                10
            )

        self.command_pub = self.create_publisher(
            String,
            "/voice/recognized_command",
            10
        )

        self.scores_pub = self.create_publisher(
            String,
            "/voice/log_likelihoods",
            10
        )

        self.get_logger().info("Voice HMM recognizer node ready.")

    def listen_callback(self, msg):
        if self.input_mode == "arecord":
            self.handle_arecord_listen_flag(msg.data)
            return

        if self.input_mode != "topic":
            self.get_logger().error(
                f"Unknown input_mode '{self.input_mode}'. Use 'arecord' or 'topic'."
            )
            return

        if msg.data and not self.listening:
            self.get_logger().info("Listening started.")
            self.listening = True
            self.audio_buffer = []

        elif not msg.data and self.listening:
            self.get_logger().info("Listening stopped. Classifying...")
            self.listening = False
            self.classify_buffer()

    def audio_callback(self, msg):
        if self.input_mode != "topic" or not self.listening:
            return

        self.audio_buffer.extend(msg.data)

    def handle_arecord_listen_flag(self, enabled):
        if not enabled:
            return

        if self.recording_thread is not None and self.recording_thread.is_alive():
            self.get_logger().warn("Already recording. Ignoring listen request.")
            return

        self.recording_thread = threading.Thread(
            target=self.record_and_classify,
            daemon=True,
        )
        self.recording_thread.start()

    def record_and_classify(self):
        self.get_logger().info(
            f"Recording {self.record_duration:.1f}s from {self.alsa_device}"
        )

        try:
            record_audio_arecord(
                path=self.live_wav_path,
                duration_seconds=self.record_duration,
                sample_rate=self.sample_rate,
                alsa_device=self.alsa_device,
            )

            audio_float, sample_rate = read_wav(self.live_wav_path)
            self.classify_audio(audio_float, sample_rate)

        except Exception as exc:
            self.get_logger().error(f"Voice recognition failed: {exc}")

    def classify_buffer(self):
        if len(self.audio_buffer) == 0:
            self.get_logger().warn("Audio buffer is empty.")
            return

        audio_int16 = np.array(self.audio_buffer, dtype=np.int16)
        audio_float = audio_int16.astype(np.float64) / 32768.0

        self.classify_audio(audio_float, self.sample_rate)

    def classify_audio(self, audio_float, sample_rate):
        peak = float(np.max(np.abs(audio_float))) if len(audio_float) else 0.0

        if peak < self.min_audio_peak:
            self.get_logger().warn(
                f"Audio too quiet. peak={peak:.4f}, min_audio_peak={self.min_audio_peak:.4f}"
            )
            self.publish_result(self.reject_word, {})
            return

        best_word, scores = self.recognizer.predict(
            audio_float,
            sample_rate
        )

        if best_word is None:
            self.get_logger().warn("Could not recognize audio.")
            self.publish_result(self.reject_word, {})
            return

        final_word = self.apply_rejection(best_word, scores)
        self.publish_result(final_word, scores)

    def apply_rejection(self, best_word, scores):
        sorted_scores = sorted(
            scores.items(),
            key=lambda item: item[1],
            reverse=True
        )

        if len(sorted_scores) < 2:
            return best_word

        margin = sorted_scores[0][1] - sorted_scores[1][1]

        if margin < self.min_score_margin:
            self.get_logger().warn(
                f"Rejected as '{self.reject_word}'. Best={best_word}, margin={margin:.3f}"
            )
            return self.reject_word

        return best_word

    def publish_result(self, command, scores):
        command_msg = String()
        command_msg.data = command
        self.command_pub.publish(command_msg)

        sorted_scores = sorted(
            scores.items(),
            key=lambda item: item[1],
            reverse=True
        )

        scores_text = ", ".join(
            [f"{word}:{score:.3f}" for word, score in sorted_scores]
        ) if sorted_scores else "no_scores"

        scores_msg = String()
        scores_msg.data = scores_text
        self.scores_pub.publish(scores_msg)

        self.get_logger().info(f"Published command: {command}")
        self.get_logger().info(f"Scores: {scores_text}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecognitionNode()

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
