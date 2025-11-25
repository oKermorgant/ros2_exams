import rclpy
from tf2_ros import Buffer, TransformListener
from math import pi, sqrt, cos, sin, atan2


class Tracker:

  def __init__(self, node):

    self.tf_buffer = Buffer(node = node)
    self.listener = TransformListener(self.tf_buffer, node)
    self.frame = None
    self.contour = None
    self.cur = 0

  def setFrame(self, frame):
    self.frame = frame

  def initFrom(self, contour):
    if not contour:
      return
    self.contour = contour
    self.cur = 0

  def track(self):
    if not self.contour or not self.frame:
      return (0.,0.)

    now = rclpy.time.Time()
    if not self.tf_buffer.can_transform(self.frame, 'map', now):
      return (0.,0.)

    me = self.tf_buffer.lookup_transform('map', self.frame, now).transform

    dx,dy = self.updateWP(me.translation.x, me.translation.y)

    theta = 2*atan2(me.rotation.z, me.rotation.w)
    dtheta = atan2(dy,dx) - theta
    while dtheta > pi:
      dtheta -= 2*pi
    while dtheta < -pi:
      dtheta += 2*pi

    vmax = 1.
    Kv = 2.
    Kw = 3.

    v = Kv*(dx*cos(theta)+dy*sin(theta))
    w = Kw*dtheta

    scale = max(1.,abs(v) / vmax)
    return v*(pi-abs(dtheta))/scale, w/scale

  def updateWP(self, x, y):

    dx = self.contour[self.cur].x - x
    dy = self.contour[self.cur].y - y

    print(sqrt(dx**2+dy**2))
    if sqrt(dx**2+dy**2) > 0.1:
      return dx,dy

    self.cur += 1
    if self.cur == len(self.contour):
      self.cur = 0
    return self.contour[self.cur].x - x, self.contour[self.cur].y - y

