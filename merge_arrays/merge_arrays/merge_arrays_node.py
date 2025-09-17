#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout
from typing import List, Optional


def merge_sorted(a: List[int], b: List[int]) -> List[int]:
    """Linear-time merge for two sorted lists."""
    i = j = 0
    out = []
    na, nb = len(a), len(b)
    while i < na and j < nb:
        if a[i] <= b[j]:
            out.append(a[i]); i += 1
        else:
            out.append(b[j]); j += 1
    if i < na:
        out.extend(a[i:])
    if j < nb:
        out.extend(b[j:])
    return out


class MergeArraysNode(Node):
    def __init__(self):
        super().__init__('merge_arrays_node')

        # Subscribers
        self.sub1 = self.create_subscription(
            Int32MultiArray, '/input/array1', self.cb_array1, 10)
        self.sub2 = self.create_subscription(
            Int32MultiArray, '/input/array2', self.cb_array2, 10)

        # Publisher
        self.pub = self.create_publisher(
            Int32MultiArray, '/output/array', 10)

        # Latest messages (store data only)
        self.latest1: Optional[List[int]] = None
        self.latest2: Optional[List[int]] = None

        self.get_logger().info('merge_arrays_node started.')

    def cb_array1(self, msg: Int32MultiArray):
        self.latest1 = list(msg.data)
        self.try_publish()

    def cb_array2(self, msg: Int32MultiArray):
        self.latest2 = list(msg.data)
        self.try_publish()

    def try_publish(self):
        # Only publish when we have received at least one message from both sides
        if self.latest1 is None or self.latest2 is None:
            return

        merged = merge_sorted(self.latest1, self.latest2)

        # Build layout (1-D, length = len(merged)); layout can also be left empty,
        # but populating dim is a tiny bit nicer.
        dim = MultiArrayDimension(label='length', size=len(merged), stride=len(merged))
        layout = MultiArrayLayout(dim=[dim], data_offset=0)

        out = Int32MultiArray(layout=layout, data=merged)
        self.pub.publish(out)
        # Optional: log once in a while; keep quiet for performance
        self.get_logger().debug(f'Published merged array of len={len(merged)}')


def main():
    rclpy.init()
    node = MergeArraysNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
