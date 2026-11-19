"""Bag detector: overhead depth camera -> bag centre, yaw and size.

The camera looks straight down from (cam_x, cam_y, cam_z). For a pixel (u, v) with
depth d (distance along the optical axis = straight down):

    height  z = cam_z - d
    x = cam_x - d * (v - cy) / fy      (image rows go towards -x)
    y = cam_y - d * (u - cx) / fx      (image columns go towards -y)

Steps:
  1. keep pixels that stand more than 3 cm above the belt, inside the pick zone
  2. several bags can be on the belt: the blob closest to the photo-eye (largest y)
     = the bag that stopped the belt
  3. top height = median of the highest pixels -> bag height = top - belt
  4. top-face pixels -> (x, y) points -> smallest rotated rectangle around them
     (cv2.minAreaRect) -> centre, long side, short side, yaw
"""
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from bag_cell_interfaces.srv import DetectBag


def wrap_half_pi(a):
    """Box yaw is the same every 180 deg: bring it to (-90, 90] deg."""
    while a <= -math.pi / 2:
        a += math.pi
    while a > math.pi / 2:
        a -= math.pi
    return a


class BagDetector(Node):
    def __init__(self):
        super().__init__("bag_detector")
        self.declare_parameter("cam_xyz", [0.9, 0.0, 1.92])
        self.declare_parameter("belt_top", 0.5)
        self.declare_parameter("roi_x", [0.52, 1.28])   # belt width (photo-eye posts excluded)
        self.declare_parameter("roi_y", [-0.6, 0.6])
        self.declare_parameter("min_height", 0.03)      # m above the belt to count as bag
        self.cv = CvBridge()
        self.lock = threading.Lock()
        self.depth, self.depth_time, self.rgb, self.K = None, 0.0, None, None

        sensors = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, "/overhead_cam/depth_image", self.on_depth, 2,
                                 callback_group=sensors)
        self.create_subscription(Image, "/overhead_cam/image", self.on_rgb, 2,
                                 callback_group=sensors)
        self.create_subscription(CameraInfo, "/overhead_cam/camera_info", self.on_info, 2,
                                 callback_group=sensors)
        self.debug_pub = self.create_publisher(Image, "/bag_detector/debug", 2)
        self.create_service(DetectBag, "detect_bag", self.detect_cb,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("Ready: /detect_bag")

    # ---------------------------------------------------------------- inputs
    def on_depth(self, msg):
        d = self.cv.imgmsg_to_cv2(msg, "32FC1")
        with self.lock:
            self.depth, self.depth_time = np.array(d, dtype=np.float32), time.time()

    def on_rgb(self, msg):
        with self.lock:
            self.rgb = self.cv.imgmsg_to_cv2(msg, "bgr8")

    def on_info(self, msg):
        self.K = msg.k  # fx 0 cx / 0 fy cy / 0 0 1

    # ---------------------------------------------------------------- service
    def detect_cb(self, req, res):
        asked = time.time()
        frame = None
        while time.time() - asked < 5.0:   # wait for a frame taken after the request
            with self.lock:
                if self.depth is not None and self.depth_time > asked + 0.05 and self.K is not None:
                    frame, rgb = self.depth.copy(), None if self.rgb is None else self.rgb.copy()
                    break
            time.sleep(0.02)
        if frame is None:
            res.found, res.message = False, "no camera frame"
            return res
        try:
            self.measure(frame, rgb, res)
        except Exception as e:  # noqa: BLE001
            res.found, res.message = False, f"error: {e}"
        self.get_logger().info(res.message)
        return res

    def measure(self, D, rgb, res):
        cam_x, cam_y, cam_z = self.get_parameter("cam_xyz").value
        belt = self.get_parameter("belt_top").value
        rx = self.get_parameter("roi_x").value
        ry = self.get_parameter("roi_y").value
        fx, cx, fy, cy = self.K[0], self.K[2], self.K[4], self.K[5]

        # 1. every pixel -> world point
        v, u = np.mgrid[0:D.shape[0], 0:D.shape[1]]
        valid = np.isfinite(D) & (D > 0.1)
        X = cam_x - D * (v - cy) / fy
        Y = cam_y - D * (u - cx) / fx
        Z = cam_z - D
        mask = (valid & (Z > belt + self.get_parameter("min_height").value) & (Z < belt + 1.0)
                & (X > rx[0]) & (X < rx[1]) & (Y > ry[0]) & (Y < ry[1]))

        # 2. blobs big enough to be a bag; take the one closest to the photo-eye
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
        bags = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 150]
        if not bags:
            res.found, res.message = False, "no bag in the pick zone"
            return
        best = max(bags, key=lambda i: Y[labels == i].max())
        blob = labels == best

        # 3. top height (median of the top 10 %, ignores handles and noise)
        zs = Z[blob]
        top = float(np.median(zs[zs >= np.percentile(zs, 90) - 0.02]))

        # 4. top face -> rotated rectangle
        face = blob & (Z > top - 0.015)
        pts = np.stack([X[face], Y[face]], axis=1).astype(np.float32)
        (mx, my), (w, h), ang = cv2.minAreaRect(pts)
        pixel = (cam_z - top) / fx                 # size of one pixel on the top face
        w, h = w + pixel, h + pixel
        yaw = math.radians(ang)                    # direction of the 'w' side
        if h > w:
            w, h, yaw = h, w, yaw + math.pi / 2
        yaw = wrap_half_pi(yaw)

        res.found = True
        res.pose.position.x, res.pose.position.y = float(mx), float(my)
        res.pose.position.z = belt + (top - belt) / 2
        res.pose.orientation.z, res.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        res.size.x, res.size.y, res.size.z = float(w), float(h), float(top - belt)
        res.message = (f"bag {w:.3f} x {h:.3f} x {top - belt:.3f} m at "
                       f"({mx:.3f}, {my:.3f}) yaw {math.degrees(yaw):.1f} deg")
        self.publish_debug(rgb, blob, face, res.message)

    def publish_debug(self, rgb, blob, face, text):
        if rgb is None or rgb.shape[:2] != blob.shape:
            return
        img = rgb.copy()
        img[blob] = (0.5 * img[blob] + [0, 90, 0]).astype(np.uint8)
        img[face] = (0.5 * img[face] + [0, 120, 120]).astype(np.uint8)
        cv2.putText(img, text, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 255, 255), 1)
        self.debug_pub.publish(self.cv.cv2_to_imgmsg(img, "bgr8"))


def main():
    rclpy.init()
    node = BagDetector()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    exe.spin()


if __name__ == "__main__":
    main()
