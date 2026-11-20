"""Flight router: tag -> flight -> make-up position (cart), following the flight schedule.

This station has 2 make-up positions (cart A, cart B). Over the day the
positions are reused:
  - a flight's make-up OPENS at its 'open' time on a free position
  - it CLOSES at its 'close' time: its cart is towed to the aircraft (dispatch_cart) and the
    position is free for the next flight
The airport clock comes from the belt controller (/cell/airport_clock, minutes after start).

Routing of a bag (cart "" = problem bag: the robot leaves it, an operator takes it off
the belt and handles it by hand):
    tag ""                         -> ""  (no-read)
    tag not in manifest            -> ""  (unknown bag)
    flight not on this station     -> ""  (mis-sorted)
    flight not open yet            -> ""  (early bag)
    less than 20 min before close  -> ""  (late bag: rush)
    otherwise                      -> the cart of that flight
"""
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64, String

from bag_cell_interfaces.srv import DispatchCart, RouteBag

LATE_CUTOFF = 20.0   # airport minutes before close: no more bags into the cart


def minutes(hhmm, start):
    h, m = map(int, hhmm.split(":"))
    sh, sm = map(int, start.split(":"))
    return (h * 60 + m) - (sh * 60 + sm)


def hhmm(mins, start):
    sh, sm = map(int, start.split(":"))
    t = int(round(sh * 60 + sm + mins))
    return f"{t // 60 % 24:02d}:{t % 60:02d}"


class FlightRouter(Node):
    def __init__(self):
        super().__init__("flight_router")
        default = get_package_share_directory("bag_cell_scene") + "/config/manifest.yaml"
        self.declare_parameter("manifest", default)
        with open(self.get_parameter("manifest").value) as f:
            data = yaml.safe_load(f)
        self.start = data["start"]
        self.flights = {s["flight"]: dict(s, open_m=minutes(s["open"], self.start),
                                          close_m=minutes(s["close"], self.start), state="planned")
                        for s in data["schedule"]}
        self.free = list(data["positions"])   # free make-up positions
        self.cart_of = {}                      # open flight -> cart
        self.bags = {b["tag"]: b for b in data["bags"]}
        self.clock = 0.0

        group = MutuallyExclusiveCallbackGroup()
        self.dispatch = self.create_client(DispatchCart, "dispatch_cart",
                                           callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(Float64, "/cell/airport_clock", self.on_clock, 10,
                                 callback_group=group)
        self.create_service(RouteBag, "route_bag", self.route_cb, callback_group=group)
        self.board_pub = self.create_publisher(String, "/cell/flight_board", 10)
        self.create_timer(0.5, self.update, callback_group=group)
        self.get_logger().info(f"{len(self.bags)} tags known, {len(self.flights)} flights")

    def on_clock(self, msg):
        self.clock = msg.data

    def update(self):
        """Open and close flights as the airport clock goes on."""
        for f, s in self.flights.items():
            if s["state"] == "open" and self.clock >= s["close_m"]:
                cart = self.cart_of.pop(f)
                s["state"] = "closed"
                if self.dispatch.wait_for_service(timeout_sec=2.0):
                    req = DispatchCart.Request(cart=cart, flight=f, reason="flight closed")
                    self.dispatch.call(req)
                self.free.append(cart)
                self.get_logger().warn(f"{hhmm(self.clock, self.start)} {f} make-up CLOSED, "
                                       f"{cart} towed to the aircraft")
        for f, s in sorted(self.flights.items(), key=lambda kv: kv[1]["open_m"]):
            if s["state"] == "planned" and self.clock >= s["open_m"] and self.free:
                s["state"] = "open"
                self.cart_of[f] = self.free.pop(0)
                self.get_logger().warn(f"{hhmm(self.clock, self.start)} {f} ({s['dest']}, "
                                       f"STD {s['std']}) make-up OPEN on {self.cart_of[f]}")
        board = [f"{hhmm(self.clock, self.start)}"] + [
            f"{c}: {f} {self.flights[f]['dest']} closes {self.flights[f]['close']}"
            for f, c in sorted(self.cart_of.items(), key=lambda kv: kv[1])]
        self.board_pub.publish(String(data=" | ".join(board)))

    def route_cb(self, req, res):
        bag = self.bags.get(req.tag)
        res.cart, res.mass, res.known = "", 10.0, bag is not None
        if not req.tag:
            res.reason = "no-read"
        elif bag is None:
            res.reason = "unknown tag"
        else:
            res.flight, res.mass = bag["flight"], float(bag["mass"])
            s = self.flights.get(bag["flight"])
            if s is None:
                res.reason = "mis-sorted (other station)"
            elif s["state"] == "planned":
                res.reason = "early bag (make-up not open)"
            elif s["state"] == "closed" or self.clock > s["close_m"] - LATE_CUTOFF:
                res.reason = "late bag (rush)"
            else:
                res.cart, res.reason = self.cart_of[bag["flight"]], "ok"
        self.get_logger().info(f"{hhmm(self.clock, self.start)} tag '{req.tag}' -> {res.cart or 'operator'} "
                               f"({res.reason})")
        return res


def main():
    rclpy.init()
    node = FlightRouter()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    exe.spin()


if __name__ == "__main__":
    main()
