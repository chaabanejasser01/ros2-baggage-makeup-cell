"""Belt controller: feeds bags and stops the belt at the pick zone.

Several bags ride the belt at once (FIFO). Service /next_bag (bag_cell_interfaces/NextBag):
  1. while the belt runs, new manifest bags come out of the wall hatch with a random gap
     (0.9 - 1.4 m), anywhere across the 0.8 m belt and a bit rotated, like real bags.
     The belt never stops for this: up to 5 bags ride the belt at once
  2. the belt runs at 0.7 m/s until the front bag cuts the photo-eye beam -> belt stops
  3. the tag reader reports that bag's tag (5 % no-read, like a real ATR)
  4. answer: bag name + tag (+ ground truth, only used to score the camera)
If the previous bag is still in the beam (robot failed), an operator takes it off first.
"""
import math
import random
import re
import subprocess
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from bag_cell_interfaces.srv import NextBag

BAG_SIZES = {"cabin": (0.40, 0.25, 0.20), "medium": (0.55, 0.35, 0.25),
             "large": (0.70, 0.45, 0.28)}
COLOURS = [(0.08, 0.08, 0.09), (0.10, 0.15, 0.35), (0.35, 0.35, 0.37), (0.55, 0.10, 0.12),
           (0.15, 0.35, 0.60), (0.80, 0.45, 0.10), (0.20, 0.40, 0.25)]
BELT_X, BELT_HALF_W, BELT_TOP, START_Y = 0.9, 0.4, 0.5, -4.5
MAX_OFFSET = 0.12   # bag centre at most 12 cm off the belt centre line (arm reach)
MAX_ON_BELT = 5
BELT_CMD = "/model/belt/link/base_link/track_cmd_vel"

SDF = """<sdf version="1.9"><model name="{name}"><link name="link">
<inertial><mass>{m}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz></inertia></inertial>
<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
<material><diffuse>{r} {g} {b} 1</diffuse><ambient>{r} {g} {b} 1</ambient></material></visual>
<visual name="handle"><pose>{hx} 0 {hz} 0 0 0</pose><geometry><box><size>0.03 0.12 0.03</size></box></geometry>
<material><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
<visual name="tag"><pose>{tx} {ty} 0 0 0 0</pose><geometry><box><size>0.05 0.003 0.12</size></box></geometry>
<material><diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient></material></visual>
</link>
</model></sdf>"""
# (no grip plugin here: cell_node adds it to the bag at the moment of the grip)


def gz(*args):
    res = subprocess.run(["gz", *args], capture_output=True, text=True, timeout=10)
    return res.returncode == 0 and "false" not in res.stdout, res.stdout


def gz_pose(name):
    """(x, y, z, yaw) of a model in Gazebo, or None."""
    _, out = gz("model", "-m", name, "-p")
    t = re.findall(r"\[\s*(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s*\]", out)
    if len(t) < 2:
        return None
    (x, y, z), (_, _, yaw) = [tuple(map(float, v)) for v in t[-2:]]
    return x, y, z, yaw


class BeltController(Node):
    def __init__(self):
        super().__init__("belt_controller")
        default = get_package_share_directory("bag_cell_scene") + "/config/manifest.yaml"
        self.declare_parameter("manifest", default)
        self.declare_parameter("world", "bag_hall")
        self.declare_parameter("speed", 0.7)          # m/s
        self.declare_parameter("no_read_rate", 0.05)
        self.declare_parameter("seed", 1)
        with open(self.get_parameter("manifest").value) as f:
            self.bags = yaml.safe_load(f)["bags"]
        self.rng = random.Random(self.get_parameter("seed").value)
        self.index = 0            # next manifest bag to put on the belt
        self.on_belt = []         # bags on the belt, front (closest to the photo-eye) first
        self.at_eye = None        # bag that stopped the belt last time
        self.gap = 0.0            # distance the last bag must travel before the next one comes
        self.beam_cut = threading.Event()
        self.running = False      # belt moving?

        self.belt_pub = self.create_publisher(Float64, BELT_CMD, 10)
        self.create_subscription(LaserScan, "/photo_eye", self.on_beam, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(NextBag, "next_bag", self.next_cb,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"{len(self.bags)} bags in the manifest. Ready: /next_bag")

    def on_beam(self, msg):
        # empty belt = no echo (inf); a bag in the beam gives a distance
        if msg.ranges and math.isfinite(msg.ranges[0]):
            self.beam_cut.set()
            if self.running:   # like a real photo-eye wired to the motor: stop at once
                self.running = False
                self.belt_pub.publish(Float64(data=0.0))
        else:
            self.beam_cut.clear()

    def belt(self, speed):
        self.running = speed > 0
        for _ in range(3):  # a few times: a lost message would leave the belt running
            self.belt_pub.publish(Float64(data=speed))
            time.sleep(0.02)

    def spawn(self):
        """Put the next manifest bag on the belt start (the belt keeps running)."""
        entry = self.bags[self.index]
        self.index += 1
        name = f"bag_{self.index}"
        sx, sy, sz = BAG_SIZES[entry["type"]]
        m = float(entry["mass"])
        # long side roughly along the belt, anywhere across it (but inside the side frames)
        yaw = math.pi / 2 + self.rng.uniform(-0.5, 0.5)
        half = sx / 2 * abs(math.cos(yaw)) + sy / 2 * abs(math.sin(yaw))
        room = min(MAX_OFFSET, max(0.0, BELT_HALF_W - 0.04 - half))
        x = BELT_X + self.rng.uniform(-room, room)
        r, g, b = self.rng.choice(COLOURS)
        sdf = SDF.format(name=name, m=m, sx=sx, sy=sy, sz=sz, r=r, g=g, b=b,
                         ixx=m * (sy**2 + sz**2) / 12, iyy=m * (sx**2 + sz**2) / 12,
                         izz=m * (sx**2 + sy**2) / 12, hx=-sx / 2 - 0.015, hz=sz / 2 - 0.05,
                         tx=sx / 4, ty=sy / 2 + 0.002)
        sdf = " ".join(sdf.split())
        req_txt = (f"sdf: '{sdf}' name: \"{name}\" pose: {{position: {{x: {x}, y: {START_Y}, "
                   f"z: {BELT_TOP + sz / 2 + 0.01}}}, orientation: {{z: {math.sin(yaw / 2)}, "
                   f"w: {math.cos(yaw / 2)}}}}}")
        world = self.get_parameter("world").value
        ok, _ = gz("service", "-s", f"/world/{world}/create", "--reqtype", "gz.msgs.EntityFactory",
                   "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req_txt)
        if not ok:
            self.get_logger().error(f"could not spawn {name}")
            return
        self.on_belt.append(dict(name=name, entry=entry, size=(sx, sy, sz), mass=m))
        self.gap = self.rng.uniform(0.9, 1.4)
        self.get_logger().info(f"{name} ({entry['type']}) on the belt, {len(self.on_belt)} on belt")

    def room_for_next(self):
        """True if the last bag has moved far enough from the belt start."""
        if self.index >= len(self.bags) or len(self.on_belt) >= MAX_ON_BELT:
            return False
        if not self.on_belt:
            return True
        p = gz_pose(self.on_belt[-1]["name"])
        return p is not None and p[1] > START_Y + self.gap

    def next_cb(self, req, res):
        world = self.get_parameter("world").value
        # previous bag still in the beam (robot failed): the operator takes it off
        if self.at_eye and self.beam_cut.is_set():
            gz("service", "-s", f"/world/{world}/remove", "--reqtype", "gz.msgs.Entity",
               "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
               "--req", f'name: "{self.at_eye}" type: MODEL')
            self.get_logger().warn(f"{self.at_eye} was still in the beam: removed by operator")
            time.sleep(0.5)
        self.at_eye = None
        if not self.on_belt and self.index >= len(self.bags):
            res.success, res.message = False, "manifest finished, no more bags"
            return res

        # run the belt until the front bag cuts the beam; new bags come in on the way
        t0 = time.time()
        speed = self.get_parameter("speed").value
        last_check = 0.0
        arrived = False
        while time.time() - t0 < 120.0:
            if time.time() - last_check > 0.5:
                last_check = time.time()
                if self.room_for_next():
                    self.spawn()
            if not self.on_belt:
                break
            if self.beam_cut.is_set():   # cut while we were busy (belt already stopped)
                arrived = True
                break
            self.belt(speed)
            if self.beam_cut.wait(timeout=0.1):
                arrived = True
                break
        self.belt(0.0)
        if not arrived:
            res.success, res.message = False, "no bag reached the photo-eye"
            return res
        time.sleep(0.5)  # settle

        bag = self.on_belt.pop(0)  # FIFO: the front bag is the one in the beam
        self.at_eye = bag["name"]
        p = gz_pose(bag["name"])   # ground truth (to score the camera only)
        if p:
            res.true_pose.position.x, res.true_pose.position.y, res.true_pose.position.z = p[:3]
            res.true_pose.orientation.z = math.sin(p[3] / 2)
            res.true_pose.orientation.w = math.cos(p[3] / 2)

        # tag reader
        entry = bag["entry"]
        read = "" if self.rng.random() < self.get_parameter("no_read_rate").value else entry["tag"]
        res.success, res.name, res.tag, res.true_mass = True, bag["name"], read, bag["mass"]
        res.true_size.x, res.true_size.y, res.true_size.z = bag["size"]
        res.message = (f"{bag['name']} ({entry['type']}, {bag['mass']} kg) stopped at the pick "
                       f"zone after {time.time() - t0:.1f} s, tag read: '{read or 'NO-READ'}', "
                       f"{len(self.on_belt)} more on the belt")
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = BeltController()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    exe.spin()


if __name__ == "__main__":
    main()
