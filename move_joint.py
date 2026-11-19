#!/usr/bin/env python3
"""
move_joint.py - send one joint to an angle and confirm it with the encoder.

  python3 move_joint.py --joint 0 --target 260

Publishes arm/joint_cmd (Float32MultiArray, 6 values; NaN = leave that joint
alone) and watches arm/joint_state (12 floats: 6 positions, then 6 velocities).
Exits 0 once the joint stays within --tol for --settle seconds, 1 on timeout.

Run with FASTDDS_BUILTIN_TRANSPORTS=UDPv4 (Docker agent <-> host ROS 2).
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

NUM_JOINTS = 6


class MoveJoint(Node):
    def __init__(self, joint, target, tol, settle, timeout):
        super().__init__("move_joint")
        self.joint, self.target, self.tol = joint, target, tol
        self.settle, self.timeout = settle, timeout

        self.cmd_pub = self.create_publisher(Float32MultiArray, "arm/joint_cmd", 10)
        state_qos = QoSProfile(depth=10,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32MultiArray, "arm/joint_state",
                                 self.on_state, state_qos)

        self.pos = None
        self.in_tol_since = None
        self.start = time.monotonic()
        self.last_print = 0.0
        self.result = None            # True = arrived, False = timed out
        self.create_timer(0.1, self.tick)   # re-send the target at 10 Hz

    def on_state(self, msg):
        if len(msg.data) >= NUM_JOINTS:
            self.pos = float(msg.data[self.joint])

    def tick(self):
        cmd = Float32MultiArray()
        cmd.data = [math.nan] * NUM_JOINTS
        cmd.data[self.joint] = float(self.target)
        self.cmd_pub.publish(cmd)

        now = time.monotonic()
        t = now - self.start
        if self.pos is None:
            if t > 3.0 and now - self.last_print > 1.0:
                print("waiting for arm/joint_state ... (agent up? best_effort QoS?)")
                self.last_print = now
        else:
            err = self.target - self.pos
            if now - self.last_print >= 0.2:
                print(f"t={t:5.1f}s  pos={self.pos:8.2f}  target={self.target:7.2f}  err={err:+8.2f}")
                self.last_print = now
            if abs(err) <= self.tol:
                if self.in_tol_since is None:
                    self.in_tol_since = now
                elif now - self.in_tol_since >= self.settle:
                    print(f"ARRIVED: joint {self.joint} at {self.pos:.2f} deg "
                          f"(err {err:+.2f}) after {t:.1f} s")
                    self.result = True
            else:
                self.in_tol_since = None

        if self.result is None and t > self.timeout:
            print(f"TIMEOUT after {self.timeout:.0f} s: last pos "
                  f"{self.pos if self.pos is not None else 'none'}")
            self.result = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", type=int, default=0)
    ap.add_argument("--target", type=float, required=True, help="degrees")
    ap.add_argument("--tol", type=float, default=0.5, help="degrees")
    ap.add_argument("--settle", type=float, default=0.5, help="seconds inside tol")
    ap.add_argument("--timeout", type=float, default=20.0, help="seconds")
    a = ap.parse_args()
    if not 0 <= a.joint < NUM_JOINTS:
        sys.exit(f"--joint must be 0..{NUM_JOINTS - 1}")

    rclpy.init()
    node = MoveJoint(a.joint, a.target, a.tol, a.settle, a.timeout)
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    ok = bool(node.result)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
