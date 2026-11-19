"""Cart manager: where does the next bag go in a cart?

Each cart has fixed slots. A slot holds up to 2 bags (one on top of the other).
  cart A / cart B (1.1 x 0.75 m): 2 slots side by side  -> up to 4 bags
  reject cart     (0.8 x 0.6 m) : 1 slot                -> up to 2 bags
Bags are laid with their long side across the cart (along y).
Rule: use the slot whose top is lowest (fills the floor first, then the 2nd layer).

Services
  plan_placement   (cart, size) -> release pose + safe height above it
  report_placement (cart, bag)  -> reads where the bag REALLY landed (Gazebo),
                                   updates the slot height and MoveIt's scene
  dispatch_cart    (cart)       -> cart full: a tug takes it away, an empty one comes
"""
import math
import re
import subprocess

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from bag_cell_interfaces.msg import CartDispatch
from bag_cell_interfaces.srv import DispatchCart, PlanPlacement, ReportPlacement

DECK = 0.35        # cart deck height
WALL_TOP = 0.65    # top of the cart walls
DROP = 0.02        # release 2 cm above the resting height
CLEARANCE = 0.08   # bag bottom 8 cm above walls / bags before going down
LAYERS = 2

# slot centres (x, y), and the cart outline (x range, y range) to check where bags land
CARTS = {
    "cart_a": dict(slots=[(-0.37, 0.85), (0.17, 0.85)], x=(-0.65, 0.45), y=(0.475, 1.225)),
    "cart_b": dict(slots=[(-0.37, -0.85), (0.17, -0.85)], x=(-0.65, 0.45), y=(-1.225, -0.475)),
    "cart_reject": dict(slots=[(-0.85, 0.0)], x=(-1.15, -0.55), y=(-0.4, 0.4)),
}
SLOT_YAW = math.pi / 2


def gz_model_pose(name):
    """(x, y, z, roll, pitch, yaw) of a Gazebo model, read with the 'gz model' tool."""
    out = subprocess.run(["gz", "model", "-m", name, "-p"], capture_output=True, text=True,
                         timeout=10).stdout
    t = re.findall(r"\[\s*(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s*\]", out)
    if len(t) < 2:
        return None
    return tuple(map(float, t[-2])) + tuple(map(float, t[-1]))


class Cart:
    def __init__(self, name):
        self.name = name
        self.slots = CARTS[name]["slots"]
        self.reset()

    def reset(self):
        self.top = [DECK] * len(self.slots)       # height of what is on each slot
        self.layers = [0] * len(self.slots)
        self.bags, self.volume = [], 0.0

    def next_slot(self):
        free = [i for i in range(len(self.slots)) if self.layers[i] < LAYERS]
        return min(free, key=lambda i: self.top[i]) if free else None

    def fill_ratio(self):
        c = CARTS[self.name]
        usable = (c["x"][1] - c["x"][0]) * (c["y"][1] - c["y"][0]) * 0.6  # 0.6 m load height
        return self.volume / usable


class CartManager(Node):
    def __init__(self):
        super().__init__("cart_manager")
        self.declare_parameter("world", "bag_hall")
        self.carts = {n: Cart(n) for n in CARTS}
        self.pending = {}  # bag -> slot index chosen at planning time
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.dispatch_pub = self.create_publisher(CartDispatch, "/cell/cart_dispatch", 10)
        self.create_service(PlanPlacement, "plan_placement", self.plan_cb)
        self.create_service(ReportPlacement, "report_placement", self.report_cb)
        self.create_service(DispatchCart, "dispatch_cart", self.dispatch_cb)
        self.get_logger().info("Ready: /plan_placement /report_placement /dispatch_cart")

    def plan_cb(self, req, res):
        cart = self.carts.get(req.cart)
        i = cart.next_slot() if cart else None
        if i is None:
            res.ok, res.message = False, f"{req.cart} full"
            return res
        x, y = cart.slots[i]
        h = req.size.z
        res.ok = True
        res.place_tip.position.x, res.place_tip.position.y = x, y
        res.place_tip.position.z = cart.top[i] + h + DROP
        res.place_tip.orientation.z = math.sin(SLOT_YAW / 2)
        res.place_tip.orientation.w = math.cos(SLOT_YAW / 2)
        res.above_tip_z = max(WALL_TOP, cart.top[i]) + h + CLEARANCE
        res.message = f"{req.cart} slot {i + 1}, layer {cart.layers[i] + 1}"
        self.pending[req.cart] = i
        self.get_logger().info(res.message)
        return res

    def report_cb(self, req, res):
        pose = gz_model_pose(req.bag)
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
                i = self.pending.pop(name, None)
                if i is None or name != req.cart:   # landed somewhere unplanned: nearest slot
                    i = min(range(len(cart.slots)), key=lambda k: math.hypot(
                        cart.slots[k][0] - x, cart.slots[k][1] - y))
                cart.top[i] = max(cart.top[i], z + req.size.z / 2)   # real top of the bag
                cart.layers[i] += 1
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
        msg = CartDispatch(cart=req.cart, bags=len(cart.bags), fill_ratio=cart.fill_ratio(),
                           stack_height=max(cart.top) - DECK)
        world = self.get_parameter("world").value
        scene = PlanningScene(is_diff=True)
        for bag in cart.bags:   # the tug takes the cart away: bags leave the simulation
            subprocess.run(["gz", "service", "-s", f"/world/{world}/remove", "--reqtype",
                            "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean", "--timeout",
                            "3000", "--req", f'name: "{bag}" type: MODEL'],
                           capture_output=True, timeout=10)
            obj = CollisionObject(id=bag, operation=CollisionObject.REMOVE)
            obj.header.frame_id = "world"
            scene.world.collision_objects.append(obj)
        self.scene_pub.publish(scene)
        self.dispatch_pub.publish(msg)
        res.ok, res.bags, res.fill_ratio = True, msg.bags, msg.fill_ratio
        self.get_logger().info(f"{req.cart} dispatched with {msg.bags} bags "
                               f"({100 * msg.fill_ratio:.0f} % full), empty cart in place")
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
    rclpy.spin(CartManager())


if __name__ == "__main__":
    main()
