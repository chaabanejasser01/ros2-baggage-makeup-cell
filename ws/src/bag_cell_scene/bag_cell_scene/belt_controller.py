"""Belt controller: runs the make-up belt on its own, like a real one fed by the sorter.

The belt runs from startup. Logic (photo-eye at the pick zone):
  - beam cut by a bag   -> belt stops at once (the bag waits for the robot)
  - beam clear again     -> the bag was lifted away: belt restarts 2 s later (sim time,
                            so the arm has carried the bag out of the pick zone)
  - while the belt runs, new manifest bags come out of the wall hatch with a random gap
    (0.9 - 1.4 m), anywhere across the 0.8 m belt and a bit rotated. Up to 5 on the belt.
  - early-bag store: a bag waits there until its flight's make-up opens.

Airport clock: starts when the robot asks for its first bag; published on
/cell/airport_clock (minutes after 'start', 1 s of sim = time_scale minutes).

Service /next_bag (bag_cell_interfaces/NextBag): waits until a bag is stopped at the
photo-eye and answers with its name + tag (5 % no-read, like a real ATR) + ground truth
(only used to score the camera). If the bag handed out last time is still in the beam
(robot failed), an operator takes it off first.
"""
import math
import random
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from tf2_msgs.msg import TFMessage
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
RESTART_DELAY = 2.0  # s (sim time) after the picked bag leaves the beam


def minutes(hhmm, start):
    h, m = map(int, hhmm.split(":"))
    sh, sm = map(int, start.split(":"))
    return (h * 60 + m) - (sh * 60 + sm)

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
            data = yaml.safe_load(f)
        self.bags = data["bags"]
        self.scale = float(data.get("time_scale", 1.0))
        self.opens = {s["flight"]: minutes(s["open"], data["start"])   # make-up opening time
                      for s in data.get("schedule", [])}
        self.t0 = None            # sim time of the first /next_bag (airport clock start)
        self.done = set()         # manifest indexes put on the belt or missed
        self.rng = random.Random(self.get_parameter("seed").value)
        self.on_belt = []         # bags on the belt, front (closest to the photo-eye) first
        self.at_eye = None        # bag stopped in the beam (waiting or handed to the robot)
        self.ready = None         # answer for /next_bag, not handed out yet
        self.gap = 0.0            # distance the last bag must travel before the next one comes
        self.beam_cut = threading.Event()
        self.eye_alive = threading.Event()   # first photo-eye message = Gazebo is running
        self.running = False      # belt moving?
        self.poses = {}           # bag -> (x, y, z, yaw), true poses from Gazebo (gz_pose_relay)

        self.belt_pub = self.create_publisher(Float64, BELT_CMD, 10)
        self.clock_pub = self.create_publisher(Float64, "/cell/airport_clock", 10)
        self.create_timer(0.5, lambda: self.clock_pub.publish(Float64(data=self.airport())))
        self.create_subscription(LaserScan, "/photo_eye", self.on_beam, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(NextBag, "next_bag", self.next_cb,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(TFMessage, "/cell/bag_poses", self.on_poses, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        world = self.get_parameter("world").value
        gz = MutuallyExclusiveCallbackGroup()   # Gazebo services (bridged by ros_gz_bridge)
        self.spawn_cli = self.create_client(SpawnEntity, f"/world/{world}/create",
                                            callback_group=gz)
        self.remove_cli = self.create_client(DeleteEntity, f"/world/{world}/remove",
                                             callback_group=gz)
        threading.Thread(target=self.flow, daemon=True).start()
        self.get_logger().info(f"{len(self.bags)} bags in the manifest. Ready: /next_bag")

    def on_poses(self, msg):
        for t in msg.transforms:
            p, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.poses[t.child_frame_id] = (p.x, p.y, p.z, yaw)

    def gz_call(self, client, req, timeout=5.0):
        """Call a bridged Gazebo service from a plain thread (not a callback)."""
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        done = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _: done.set())
        return done.wait(timeout) and future.result() is not None and future.result().success

    def remove(self, name):
        req = DeleteEntity.Request()
        req.entity.name, req.entity.type = name, Entity.MODEL
        return self.gz_call(self.remove_cli, req)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9   # sim time

    def airport(self):
        """Airport clock, minutes after start (0 until the robot asks for its first bag)."""
        return 0.0 if self.t0 is None else (self.now() - self.t0) * self.scale

    def next_to_release(self):
        """Next bag in check-in order. The early-bag store keeps a bag until its flight's
        make-up opens (mis-sorted bags have no flight here: they come at once)."""
        for i, b in enumerate(self.bags):
            if i in self.done:
                continue
            opens = self.opens.get(b["flight"])
            if opens is None or self.airport() >= opens:
                return i
        return None

    def on_beam(self, msg):
        self.eye_alive.set()
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

    def spawn(self, index):
        """Put manifest bag 'index' on the belt start (the belt keeps running)."""
        entry = self.bags[index]
        name = f"bag_{index + 1}"
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
        req = SpawnEntity.Request()
        f = req.entity_factory
        f.name, f.sdf = name, sdf
        f.pose = Pose()
        f.pose.position.x, f.pose.position.y, f.pose.position.z = x, START_Y, BELT_TOP + sz / 2 + 0.01
        f.pose.orientation.z, f.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.gz_call(self.spawn_cli, req):
            self.get_logger().warn(f"could not spawn {name}, will retry")
            return
        self.done.add(index)
        self.on_belt.append(dict(name=name, entry=entry, size=(sx, sy, sz), mass=m))
        self.gap = self.rng.uniform(0.9, 1.4)
        self.get_logger().info(f"{name} ({entry['type']}) on the belt, {len(self.on_belt)} on belt")

    def room_for_next(self):
        """True if the last bag has moved far enough from the belt start."""
        if len(self.on_belt) >= MAX_ON_BELT:
            return False
        if not self.on_belt:
            return True
        p = self.poses.get(self.on_belt[-1]["name"])
        return p is not None and p[1] > START_Y + self.gap

    def arrived(self):
        """The front bag cut the beam: prepare the answer for /next_bag."""
        time.sleep(0.5)  # settle
        bag = self.on_belt.pop(0)  # FIFO: the front bag is the one in the beam
        res = NextBag.Response()
        p = self.poses.get(bag["name"])   # ground truth (to score the camera only)
        if p:
            res.true_pose.position.x, res.true_pose.position.y, res.true_pose.position.z = p[:3]
            res.true_pose.orientation.z = math.sin(p[3] / 2)
            res.true_pose.orientation.w = math.cos(p[3] / 2)
        entry = bag["entry"]
        read = "" if self.rng.random() < self.get_parameter("no_read_rate").value else entry["tag"]
        res.success, res.name, res.tag, res.true_mass = True, bag["name"], read, bag["mass"]
        res.true_size.x, res.true_size.y, res.true_size.z = bag["size"]
        res.message = (f"{bag['name']} ({entry['type']}, {bag['mass']} kg) stopped at the pick "
                       f"zone, tag read: '{read or 'NO-READ'}', {len(self.on_belt)} more on the belt")
        self.get_logger().info(res.message)
        self.at_eye, self.ready = bag["name"], res

    def flow(self):
        """The belt's own control loop (runs all the time)."""
        self.eye_alive.wait()          # Gazebo is up
        speed = self.get_parameter("speed").value
        clear_since = self.now()
        last_spawn_check = last_cmd = 0.0
        while rclpy.ok():
            if time.time() - last_spawn_check > 0.5:
                last_spawn_check = time.time()
                i = self.next_to_release()
                if i is not None and self.room_for_next():
                    self.spawn(i)
            if self.beam_cut.is_set():
                if self.at_eye is None and self.on_belt:
                    self.belt(0.0)
                    self.arrived()
            else:
                if self.at_eye is not None:        # the bag was lifted away
                    self.at_eye = None
                    clear_since = self.now()
                if self.on_belt and self.now() - clear_since > RESTART_DELAY:
                    if not self.running or time.time() - last_cmd > 1.0:
                        self.belt(speed)           # (re-sent every second, in case one is lost)
                        last_cmd = time.time()
            time.sleep(0.05)

    def next_cb(self, req, res):
        # bag handed out last time still in the beam (robot failed): the operator takes it off
        if self.at_eye and self.ready is None and self.beam_cut.is_set():
            self.remove(self.at_eye)
            self.get_logger().warn(f"{self.at_eye} was still in the beam: removed by operator")
        if self.t0 is None:
            self.t0 = self.now()   # the robot is ready: the airport clock starts
        t0 = time.time()
        while self.ready is None and time.time() - t0 < 900.0:
            if not self.on_belt and len(self.done) >= len(self.bags) and self.at_eye is None:
                res.success, res.message = False, "manifest finished, no more bags"
                return res
            time.sleep(0.05)
        if self.ready is None:
            res.success, res.message = False, "no bag reached the photo-eye"
            return res
        out, self.ready = self.ready, None
        return out


def main():
    rclpy.init()
    node = BeltController()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    exe.spin()


if __name__ == "__main__":
    main()
