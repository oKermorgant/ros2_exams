#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

dt = 0.05


class BallNode(Node):
    def __init__(self):

        super().__init__('ball')
        self.tb = TransformBroadcaster(self)
        self.tf = TransformStamped()
        self.tf.header.frame_id = 'torso'

        self.angle = 0.

        self.markers = MarkerArray()

        self.M = []

        for side in ('left', 'right'):
            marker = Marker()
            marker.ns = side
            marker.lifetime.sec = 1
            marker.header.frame_id = side + '_target'
            marker.action = 0
            marker.scale.x = marker.scale.y = marker.scale.z = .2
            marker.type = marker.SPHERE
            if side == 'left':
                marker.color.r = 1.
                marker.color.a = 1.
            else:
                marker.color.g = 1.
                marker.color.a = 1.

            self.markers.markers.append(marker)

            M = np.zeros((4,4))
            M[2,2] = M[3,3] = 1  # homogeneous
            M[2,3] = .8 if side == 'left' else .2    # height
            sign = 1 if side == 'left' else -1
            M[0,0] = M[1,1] = c = np.sqrt(2)/2
            s = sign*c
            M[0,1] = s
            M[1,0] = -s
            M[0,3] = 1.2 if side == 'left' else 1.2
            M[1,3] = s
            self.M.append(M)

        self.mk_pub = self.create_publisher(MarkerArray, '/spheres', 1)

        self.timer = self.create_timer(dt, self.update)

    def update(self):

        now = self.get_clock().now().to_msg()
        self.tf.header.stamp = now

        self.angle += 1.*dt

        for i,side in enumerate(('left', 'right')):

            self.tf.child_frame_id = side + '_target'

            angle = self.angle*(1+.5*i)

            X = np.array([.2*np.cos(angle), 0, .2*np.sin(angle), 1])
            Xs = (self.M[i] @ X.T)

            self.tf.transform.translation.x = Xs[0]
            self.tf.transform.translation.y = Xs[1]
            self.tf.transform.translation.z = Xs[2]
            self.tb.sendTransform(self.tf)

            mk = self.markers.markers[i]
            mk.header.stamp = now

        self.mk_pub.publish(self.markers)


rclpy.init()
rclpy.spin(BallNode())
rclpy.shutdown()

