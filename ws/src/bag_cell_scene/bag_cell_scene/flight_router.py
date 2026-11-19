"""Flight router: tag number -> flight -> cart, using the check-in manifest.

    tag ""                 -> cart_reject  (no-read)
    tag not in manifest    -> cart_reject  (unknown bag)
    flight not built here  -> cart_reject  (other flight)
    otherwise              -> the cart of that flight
"""
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from bag_cell_interfaces.srv import RouteBag


class FlightRouter(Node):
    def __init__(self):
        super().__init__("flight_router")
        default = get_package_share_directory("bag_cell_scene") + "/config/manifest.yaml"
        self.declare_parameter("manifest", default)
        with open(self.get_parameter("manifest").value) as f:
            data = yaml.safe_load(f)
        self.flight_to_cart = data["flights"]
        self.bags = {b["tag"]: b for b in data["bags"]}
        self.create_service(RouteBag, "route_bag", self.route_cb)
        self.get_logger().info(f"{len(self.bags)} tags known, carts {self.flight_to_cart}")

    def route_cb(self, req, res):
        bag = self.bags.get(req.tag)
        res.cart, res.mass, res.known = "cart_reject", 10.0, bag is not None
        if not req.tag:
            res.reason = "no-read"
        elif bag is None:
            res.reason = "unknown tag"
        else:
            res.flight, res.mass = bag["flight"], float(bag["mass"])
            if bag["flight"] in self.flight_to_cart:
                res.cart, res.reason = self.flight_to_cart[bag["flight"]], "ok"
            else:
                res.reason = f"flight {bag['flight']} not built here"
        self.get_logger().info(f"tag '{req.tag}' -> {res.cart} ({res.reason})")
        return res


def main():
    rclpy.init()
    rclpy.spin(FlightRouter())


if __name__ == "__main__":
    main()
