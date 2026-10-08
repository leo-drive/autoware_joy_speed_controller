#!/usr/bin/env python3
"""Bridge a remote DS4 gamepad from an MQTT broker to sensor_msgs/Joy.

The operator page (web/operator.html) publishes the browser Gamepad API state
to MQTT. This node runs on the vehicle, converts that state to the layout
joy_node produces for a DS4 on Linux (what DS4JoyConverter expects) and
publishes it on /joy, stamped with the vehicle clock.

Link loss is handled in stages:
  * age <= link_timeout:        publish the latest operator state
  * link_timeout < age <= stop_publish_after:
                                publish a neutral state (dead man's switch
                                released), so the controller commands zero
                                velocity and the vehicle stops under control
  * age > stop_publish_after:   publish nothing; the controller times out and
                                the vehicle interface falls back to its
                                communication fault handling (PARK + emergency)
"""

import json
import math
import os
import ssl
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Browser Gamepad API "standard" mapping indices
STD_CROSS, STD_CIRCLE, STD_SQUARE, STD_TRIANGLE = 0, 1, 2, 3
STD_L1, STD_R1, STD_L2, STD_R2 = 4, 5, 6, 7
STD_SHARE, STD_OPTIONS, STD_L3, STD_R3 = 8, 9, 10, 11
STD_UP, STD_DOWN, STD_LEFT, STD_RIGHT, STD_PS = 12, 13, 14, 15, 16
STD_NUM_BUTTONS = 17
STD_NUM_AXES = 4

# joy_node DS4 layout on Linux: 8 axes, 13 buttons.
# Sticks and the D-pad report left/up as +1; triggers rest at +1, -1 pressed.
DS4_NUM_AXES = 8
DS4_NUM_BUTTONS = 13

MAX_PAYLOAD_BYTES = 4096


def apply_deadzone(value, deadzone):
    """Match joy_node: zero inside the deadzone, rescale the rest to [-1, 1]."""
    if abs(value) < deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def neutral_ds4_joy():
    """Nothing pressed, sticks centered, triggers at rest."""
    axes = [0.0] * DS4_NUM_AXES
    axes[2] = 1.0
    axes[5] = 1.0
    return axes, [0] * DS4_NUM_BUTTONS


def standard_to_ds4_joy(std_axes, std_buttons, deadzone=0.05, button_threshold=0.5):
    """Convert browser "standard" gamepad state to the Linux joy_node DS4 layout."""

    def pressed(i):
        return 1 if std_buttons[i] >= button_threshold else 0

    axes = [0.0] * DS4_NUM_AXES
    # The browser reports right/down as +1, joy_node reports left/up as +1
    axes[0] = -apply_deadzone(std_axes[0], deadzone)  # left stick left/right
    axes[1] = -apply_deadzone(std_axes[1], deadzone)  # left stick up/down
    axes[3] = -apply_deadzone(std_axes[2], deadzone)  # right stick left/right
    axes[4] = -apply_deadzone(std_axes[3], deadzone)  # right stick up/down
    axes[2] = 1.0 - 2.0 * std_buttons[STD_L2]  # L2 trigger
    axes[5] = 1.0 - 2.0 * std_buttons[STD_R2]  # R2 trigger
    axes[6] = float(pressed(STD_LEFT) - pressed(STD_RIGHT))  # D-pad left/right
    axes[7] = float(pressed(STD_UP) - pressed(STD_DOWN))  # D-pad up/down

    buttons = [0] * DS4_NUM_BUTTONS
    buttons[0] = pressed(STD_CROSS)
    buttons[1] = pressed(STD_CIRCLE)
    buttons[2] = pressed(STD_TRIANGLE)
    buttons[3] = pressed(STD_SQUARE)
    buttons[4] = pressed(STD_L1)
    buttons[5] = pressed(STD_R1)
    buttons[6] = pressed(STD_L2)
    buttons[7] = pressed(STD_R2)
    buttons[8] = pressed(STD_SHARE)
    buttons[9] = pressed(STD_OPTIONS)
    buttons[10] = pressed(STD_PS)
    buttons[11] = pressed(STD_L3)
    buttons[12] = pressed(STD_R3)
    return axes, buttons


def parse_payload(payload):
    """Validate an operator message. Returns a dict or raises ValueError."""
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError(f"payload too large ({len(payload)} bytes)")
    msg = json.loads(payload)
    if not isinstance(msg, dict):
        raise ValueError("payload is not an object")
    if msg.get("mapping") != "standard":
        raise ValueError(f"unsupported gamepad mapping: {msg.get('mapping')!r}")

    session = msg.get("session")
    seq = msg.get("seq")
    if not isinstance(session, str) or not session or len(session) > 64:
        raise ValueError("invalid session")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        raise ValueError("invalid seq")

    def numbers(key, count, lo, hi):
        values = msg.get(key)
        if not isinstance(values, list) or len(values) < count:
            raise ValueError(f"{key} must be a list of at least {count} numbers")
        out = []
        for v in values[:count]:
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"{key} contains a non-number")
            out.append(min(max(float(v), lo), hi))
        return out

    return {
        "session": session,
        "seq": seq,
        "axes": numbers("axes", STD_NUM_AXES, -1.0, 1.0),
        "buttons": numbers("buttons", STD_NUM_BUTTONS, 0.0, 1.0),
    }


class CloudJoyBridge(Node):
    def __init__(self):
        super().__init__("cloud_joy_bridge")
        p = self.declare_parameter
        self.host = p("mqtt.host", "").value
        self.port = p("mqtt.port", 8883).value
        self.transport = p("mqtt.transport", "tcp").value
        self.use_tls = p("mqtt.tls", True).value
        self.ca_file = p("mqtt.ca_file", "").value
        self.username = p("mqtt.username", "").value
        self.password_env = p("mqtt.password_env", "JOY_MQTT_PASSWORD").value
        self.topic = p("mqtt.topic", "").value
        self.client_id = p("mqtt.client_id", "").value
        self.publish_rate = p("publish_rate", 20.0).value
        self.link_timeout = p("link_timeout", 0.5).value
        self.stop_publish_after = p("stop_publish_after", 3.0).value
        self.deadzone = p("deadzone", 0.05).value

        self.pub_joy = self.create_publisher(Joy, "output/joy", 1)

        self._lock = threading.Lock()
        self._session = None
        self._last_seq = -1
        self._latest = None  # (axes, buttons) in DS4 layout
        self._last_rx = None  # time.monotonic() of the last accepted message
        self._stage = "idle"
        self._stats = {"accepted": 0, "dropped": 0, "max_gap": 0.0}

        self.create_timer(1.0 / self.publish_rate, self.on_timer)
        self.create_timer(10.0, self.log_stats)
        self._mqtt = None

    # --- MQTT ---------------------------------------------------------------
    def start_mqtt(self):
        if not self.host or not self.topic:
            self.get_logger().error("mqtt.host and mqtt.topic must be set; not connecting")
            return
        import paho.mqtt.client as mqtt

        client_id = self.client_id or f"cloud_joy_bridge_{os.getpid()}"
        try:  # paho-mqtt >= 2.0
            client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION1, client_id=client_id, transport=self.transport
            )
        except AttributeError:  # paho-mqtt 1.x (Ubuntu 22.04)
            client = mqtt.Client(client_id=client_id, transport=self.transport)

        if self.username:
            password = os.environ.get(self.password_env)
            if password is None:
                self.get_logger().warn(f"${self.password_env} is not set; connecting without password")
            client.username_pw_set(self.username, password)
        if self.use_tls:
            client.tls_set(ca_certs=self.ca_file or None, cert_reqs=ssl.CERT_REQUIRED)
        else:
            self.get_logger().warn("MQTT TLS is disabled: commands travel unencrypted")

        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        client.reconnect_delay_set(min_delay=1, max_delay=5)
        client.connect_async(self.host, self.port, keepalive=10)
        client.loop_start()
        self._mqtt = client
        self.get_logger().info(f"connecting to {self.host}:{self.port} topic '{self.topic}'")

    def stop_mqtt(self):
        if self._mqtt is not None:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT connected")
            # QoS 0: a retransmitted (late) joystick state is worse than a lost one
            client.subscribe(self.topic, qos=0)
        else:
            self.get_logger().error(f"MQTT connection refused (rc={rc})")

    def _on_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected (rc={rc})")

    def _on_message(self, client, userdata, message):
        # A retained message is an old state replayed by the broker, never live input
        if message.retain:
            self._count_drop("retained message ignored")
            return
        self.handle_payload(message.payload)

    # --- operator state -----------------------------------------------------
    def handle_payload(self, payload, now=None):
        """Validate and store one operator message. Returns True if accepted."""
        now = time.monotonic() if now is None else now
        try:
            msg = parse_payload(payload)
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as e:
            self._count_drop(f"invalid message: {e}")
            return False

        with self._lock:
            link_alive = self._last_rx is not None and now - self._last_rx <= self.link_timeout
            if msg["session"] != self._session:
                # Only one operator at a time: a new page may take over only once
                # the current one has gone quiet.
                if link_alive:
                    self._count_drop("message from another operator session ignored", locked=True)
                    return False
                self.get_logger().info(f"operator session {msg['session'][:8]} active")
                self._session = msg["session"]
                self._last_seq = -1
            if msg["seq"] <= self._last_seq:
                self._count_drop("out-of-order message dropped", locked=True)
                return False

            if self._last_rx is not None and link_alive:
                self._stats["max_gap"] = max(self._stats["max_gap"], now - self._last_rx)
            self._last_seq = msg["seq"]
            self._latest = standard_to_ds4_joy(msg["axes"], msg["buttons"], self.deadzone)
            self._last_rx = now
            self._stats["accepted"] += 1
        return True

    def _count_drop(self, reason, locked=False):
        if locked:
            self._stats["dropped"] += 1
        else:
            with self._lock:
                self._stats["dropped"] += 1
        self.get_logger().debug(reason)

    # --- output -------------------------------------------------------------
    def on_timer(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            age = None if self._last_rx is None else now - self._last_rx
            latest = self._latest

        if age is None or age > self.stop_publish_after:
            self._set_stage("idle", "no operator input: not publishing /joy")
            return
        if age > self.link_timeout:
            self._set_stage("stopping", f"operator link lost ({age:.2f}s): publishing neutral state")
            axes, buttons = neutral_ds4_joy()
        else:
            self._set_stage("live", "operator link live")
            axes, buttons = latest

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = "cloud_joy"
        joy.axes = axes
        joy.buttons = buttons
        self.pub_joy.publish(joy)

    def _set_stage(self, stage, text):
        if stage != self._stage:
            # rclpy forbids changing the severity of a single logging call site
            if stage == "live":
                self.get_logger().info(text)
            else:
                self.get_logger().warn(text)
            self._stage = stage

    def log_stats(self):
        with self._lock:
            s = dict(self._stats)
            self._stats = {"accepted": 0, "dropped": 0, "max_gap": 0.0}
        if s["accepted"] or s["dropped"]:
            self.get_logger().info(
                f"last 10s: {s['accepted'] / 10.0:.1f} msg/s accepted, {s['dropped']} dropped, "
                f"max gap {s['max_gap'] * 1000:.0f} ms"
            )


def main():
    rclpy.init()
    node = CloudJoyBridge()
    node.start_mqtt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_mqtt()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
