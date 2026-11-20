"""Cart manager: where does the next bag go in a cart?

Height-map packing (like a palletizing robot): the cart floor is a grid of 5 cm cells, each
with the height of what is on it. For the next bag, every position and both turns (long side
along x or along y) are tried; it goes where it rests LOWEST (floor first, then on other
bags), at equal height the furthest from the robot (fill from the back). Rejected if:
  - the bag would rest on less than 60 % of its footprint, or not under its centre (tips)
  - the arm cannot get the cup high enough to lower it in there (6 cm above the walls /
    the stack, with the bag hanging under the cup): the reach is lower far from the robot
The heights are updated with where the bag REALLY landed (Gazebo), not where it was meant.
In practice (UR20: ~1.55 m high at 1 m away): about 14-20 bags per 1.6 x 1.2 m cart,
stacked up to ~1.45 m.

Services
  plan_placement   (cart, size) -> release pose + safe height above it
  report_placement (cart, bag)  -> reads where the bag REALLY landed (Gazebo),
                                   updates the slot height and MoveIt's scene
  dispatch_cart    (cart)       -> cart full or its flight closed: a tug takes it away,
                                   an empty one comes
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from tf2_msgs.msg import TFMessage
from shape_msgs.msg import SolidPrimitive

from bag_cell_interfaces.msg import CartDispatch
from bag_cell_interfaces.srv import DispatchCart, PlanPlacement, ReportPlacement

DECK = 0.35        # cart deck height
WALL_TOP = 0.65    # top of the cart walls
DROP = 0.02        # release 2 cm above the resting height
CLEARANCE = 0.06   # bag bottom 6 cm above walls / bags before going down
WALL = 0.04        # wall thickness + margin
GAP = 0.03         # space between two bags
CELL = 0.05        # height-map grid
SUPPORT = 0.6      # a bag on top of others needs 60 % of its footprint supported
# highest cup-tip height the arm reaches pointing down, vs distance from the robot base axis
# (UR20, offline IK on the URDF with the cell's joint limits, minus a 3 cm margin)
REACH = [(0.4, 1.78), (0.6, 1.71), (0.8, 1.65), (1.0, 1.55), (1.2, 1.42), (1.4, 1.22),
         (1.5, 1.06), (1.6, 0.89), (1.7, 0.63)]


def z_max(r):
    """linear interpolation in REACH"""
    if r <= REACH[0][0]:
        return REACH[0][1]
    for (r0, z0), (r1, z1) in zip(REACH, REACH[1:]):
        if r <= r1:
            return z0 + (z1 - z0) * (r - r0) / (r1 - r0)
    return 0.0

# cart outline (x range, y range): bags go along x, centred in y
CARTS = {
    "cart_a": dict(x=(-1.15, 0.45), y=(0.45, 1.65)),     # 1.6 x 1.2 m dolly
    "cart_b": dict(x=(-1.15, 0.45), y=(-1.65, -0.45)),
}


class Cart:
    def __init__(self, name):
        c = CARTS[name]
        self.name = name
        self.x0, self.y0 = c["x"][0] + WALL, c["y"][0] + WALL
        self.nx = int((c["x"][1] - c["x"][0] - 2 * WALL) / CELL)
        self.ny = int((c["y"][1] - c["y"][0] - 2 * WALL) / CELL)
        self.reset()

    def reset(self):
        self.h = np.full((self.nx, self.ny), DECK)   # height of what is on each cell
        self.bags, self.volume = [], 0.0

    def cells(self, length, width, yaw):
        """footprint in cells (along x, along y) of a bag turned by yaw (0 or 90 deg)"""
        lx, ly = (length, width) if yaw == 0.0 else (width, length)
        return math.ceil((lx + GAP) / CELL - 1e-6), math.ceil((ly + GAP) / CELL - 1e-6)

    def centre(self, i, j, cx, cy):
        return self.x0 + (i + cx / 2) * CELL, self.y0 + (j + cy / 2) * CELL

    def find(self, size):
        """best (base, i, j, yaw) for a bag of this size, or None (cart full)"""
        best = None
        for yaw in (0.0, math.pi / 2):
            cx, cy = self.cells(size.x, size.y, yaw)
            for i in range(self.nx - cx + 1):
                for j in range(self.ny - cy + 1):
                    foot = self.h[i:i + cx, j:j + cy]
                    base = float(foot.max())
                    if best is not None and base > best[0] + 0.01:
                        continue
                    if base > DECK + 0.01:   # on other bags: enough support, under the centre
                        flat = foot > base - 0.03
                        if flat.mean() < SUPPORT or not flat[cx // 2, cy // 2]:
                            continue
                    x, y = self.centre(i, j, cx, cy)
                    above = max(WALL_TOP, base) + size.z + CLEARANCE
                    if above > z_max(math.hypot(x, y)):
                        continue
                    # lowest first, then the far side first (fill from the back)
                    score = (round(base, 2), -math.hypot(x, y))
                    if best is None or score < best[4]:
                        best = (base, i, j, yaw, score)
        return best[:4] if best else None

    def add(self, i, j, yaw, size, top):
        cx, cy = self.cells(size.x, size.y, yaw)
        self.h[i:i + cx, j:j + cy] = np.maximum(self.h[i:i + cx, j:j + cy], top)

    def cell_of(self, x, y, yaw, size):
        """grid position of a bag found at (x, y) with this yaw (when it landed off plan)"""
        yaw = 0.0 if abs(math.cos(yaw)) > abs(math.sin(yaw)) else math.pi / 2
        cx, cy = self.cells(size.x, size.y, yaw)
        i = min(max(round((x - self.x0) / CELL - cx / 2), 0), self.nx - cx)
        j = min(max(round((y - self.y0) / CELL - cy / 2), 0), self.ny - cy)
        return i, j, yaw

    def fill_ratio(self):
        usable = self.nx * self.ny * CELL * CELL * 0.6  # 0.6 m load height
        return self.volume / usable


class CartManager(Node):
    def __init__(self):
        super().__init__("cart_manager")
        self.declare_parameter("world", "bag_hall")
        self.carts = {n: Cart(n) for n in CARTS}
        self.pending = {}  # bag -> slot index chosen at planning time
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.dispatch_pub = self.create_publisher(CartDispatch, "/cell/cart_dispatch", 10)
        self.poses = {}   # bag -> (x, y, z, roll, pitch, yaw), true poses (gz_pose_relay)
        self.create_subscription(TFMessage, "/cell/bag_poses", self.on_poses, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        world = self.get_parameter("world").value
        self.remove_cli = self.create_client(DeleteEntity, f"/world/{world}/remove")
        self.create_service(PlanPlacement, "plan_placement", self.plan_cb)
        self.create_service(ReportPlacement, "report_placement", self.report_cb)
        self.create_service(DispatchCart, "dispatch_cart", self.dispatch_cb)
        self.get_logger().info("Ready: /plan_placement /report_placement /dispatch_cart")

    def on_poses(self, msg):
        for t in msg.transforms:
            p, q = t.transform.translation, t.transform.rotation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.poses[t.child_frame_id] = (p.x, p.y, p.z, roll, pitch, yaw)

    def plan_cb(self, req, res):
        cart = self.carts.get(req.cart)
        spot = cart.find(req.size) if cart else None
        if spot is None:
            res.ok, res.message = False, f"{req.cart} full"
            return res
        base, i, j, yaw = spot
        x, y = cart.centre(i, j, *cart.cells(req.size.x, req.size.y, yaw))
        res.ok = True
        res.place_tip.position.x, res.place_tip.position.y = x, y
        res.place_tip.position.z = base + req.size.z + DROP
        res.place_tip.orientation.z = math.sin(yaw / 2)
        res.place_tip.orientation.w = math.cos(yaw / 2)
        res.above_tip_z = max(WALL_TOP, base) + req.size.z + CLEARANCE
        layer = "floor" if base < DECK + 0.01 else "on top"
        res.message = (f"{req.cart} bag {len(cart.bags) + 1} at ({x:+.2f}, {y:+.2f}), "
                       f"{'along x' if yaw == 0.0 else 'along y'}, {layer}")
        self.pending[req.cart] = (i, j, yaw)
        self.get_logger().info(res.message)
        return res

    def report_cb(self, req, res):
        # wait for a pose newer than the request (the bag was just released: let it settle)
        self.poses.pop(req.bag, None)
        t_end = time.time() + 2.0
        while req.bag not in self.poses and time.time() < t_end:
            time.sleep(0.05)
        pose = self.poses.get(req.bag)
        if pose is None:
            res.actual_cart = "unknown"
            return res
        x, y, z, roll, pitch, yaw = pose
        res.pose.position.x, res.pose.position.y, res.pose.position.z = x, y, z
        res.pose.orientation.z, res.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        res.actual_cart = "floor" if z < DECK else "belt" if x > 0.6 else "unknown"
        for name, c in CARTS.items():
            if c["x"][0] <= x <= c["x"][1] and c["y"][0] <= y <= c["y"][1] and z > DECK:
                res.actual_cart = name
                cart = self.carts[name]
                spot = self.pending.pop(name, None)
                if spot is None or name != req.cart:   # landed somewhere unplanned
                    spot = cart.cell_of(x, y, yaw, req.size)
                cart.add(*spot, req.size, z + req.size.z / 2)   # real top of the bag
                cart.bags.append(req.bag)
                cart.volume += req.size.x * req.size.y * req.size.z
                self.update_scene(req.bag, req.size, x, y, z, yaw)
                break
        self.get_logger().info(f"{req.bag} landed in {res.actual_cart} at ({x:.2f}, {y:.2f}, "
                               f"{z:.2f}), tilt {math.degrees(max(abs(roll), abs(pitch))):.0f} deg")
        return res

    def dispatch_cb(self, req, res):
        cart = self.carts.get(req.cart)
        if cart is None:
            res.ok = False
            return res
        msg = CartDispatch(cart=req.cart, flight=req.flight, reason=req.reason,
                           bags=len(cart.bags), fill_ratio=cart.fill_ratio(),
                           stack_height=float(cart.h.max()) - DECK)
        scene = PlanningScene(is_diff=True)
        for bag in cart.bags:   # the tug takes the cart away: bags leave the simulation
            req_rm = DeleteEntity.Request()
            req_rm.entity.name, req_rm.entity.type = bag, Entity.MODEL
            self.remove_cli.call_async(req_rm)   # (no need to wait for the answer)
            obj = CollisionObject(id=bag, operation=CollisionObject.REMOVE)
            obj.header.frame_id = "world"
            scene.world.collision_objects.append(obj)
        self.scene_pub.publish(scene)
        self.dispatch_pub.publish(msg)
        res.ok, res.bags, res.fill_ratio = True, msg.bags, msg.fill_ratio
        self.get_logger().info(f"{req.cart} ({req.flight}, {req.reason}) dispatched with "
                               f"{msg.bags} bags ({100 * msg.fill_ratio:.0f} % full), "
                               "empty cart in place")
        cart.reset()
        return res

    def update_scene(self, bag, size, x, y, z, yaw):
        """Put the bag in MoveIt's scene where it really is."""
        obj = CollisionObject(id=bag, operation=CollisionObject.ADD)
        obj.header.frame_id = "world"
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,
                                             dimensions=[size.x, size.y, size.z]))
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        obj.primitive_poses.append(p)
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(obj)
        self.scene_pub.publish(scene)


def main():
    rclpy.init()
    exe = MultiThreadedExecutor()   # poses keep coming while report_cb waits for one
    exe.add_node(CartManager())
    exe.spin()


if __name__ == "__main__":
    main()
