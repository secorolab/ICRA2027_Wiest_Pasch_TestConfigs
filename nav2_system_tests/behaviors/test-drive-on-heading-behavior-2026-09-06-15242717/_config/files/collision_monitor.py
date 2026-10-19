#!/usr/bin/env python3
"""ROS2 node that watches for /contact/* topics and latches /collision True."""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool


class CollisionMonitor(Node):
    def __init__(self) -> None:
        super().__init__("collision_monitor")

        self._contact_subs: dict[str, rclpy.subscription.Subscription] = {}

        self._pub = self.create_publisher(Bool, "/collision", 10)
        self._poll_timer = self.create_timer(1.0, self._poll_topics)

        self.get_logger().info("CollisionMonitor started — polling for /contact/* topics")

    # ── topic poller ──────────────────────────────────────────────────────────

    def _poll_topics(self) -> None:
        for topic, types in self.get_topic_names_and_types():
            if not topic.startswith("/contact/"):
                continue
            if topic in self._contact_subs:
                continue
            if "ros_gz_interfaces/msg/Contacts" not in types:
                continue

            self._contact_subs[topic] = self.create_subscription(
                Contacts,
                topic,
                self._on_contact,
                10,
            )
            self.get_logger().info(f"Subscribed to contact topic: {topic}")

    # ── contact callback ──────────────────────────────────────────────────────

    def _on_contact(self, msg: Contacts) -> None:
        if not msg.contacts:
            return

        self.get_logger().info("Collision detected — latching /collision=True")
        self._on_collision()

    # ── helpers ───────────────────────────────────────────────────────────────

    def _on_collision(self) -> None:
        self._poll_timer.cancel()
        self._poll_timer.destroy()

        for sub in self._contact_subs.values():
            sub.destroy()
        self._contact_subs.clear()

        self._publish_collision()
        self._collision_timer = self.create_timer(1.0, self._publish_collision)

    def _publish_collision(self) -> None:
        m = Bool()
        m.data = True
        self._pub.publish(m)


def main() -> None:
    rclpy.init()
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
