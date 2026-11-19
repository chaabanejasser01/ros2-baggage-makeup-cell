"""Metrics logger: writes every bag and every dispatched cart to CSV, and keeps a
summary.md up to date (the numbers for the report).

Output folder: <output_dir>/run_YYYYmmdd_HHMMSS/
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from bag_cell_interfaces.msg import BagResult, CartDispatch

BAG_FIELDS = ["bag", "tag", "flight", "target_cart", "actual_cart", "success", "reason",
              "cycle_time", "planning_time", "mass", "meas_L", "meas_S", "meas_H",
              "true_L", "true_S", "true_H", "position_error", "yaw_error"]


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


class MetricsLogger(Node):
    def __init__(self):
        super().__init__("metrics_logger")
        self.declare_parameter("output_dir", os.path.expanduser("~/bag_cell_results"))
        out = self.get_parameter("output_dir").value
        self.dir = os.path.join(out, time.strftime("run_%Y%m%d_%H%M%S"))
        os.makedirs(self.dir, exist_ok=True)
        self.bags, self.carts = [], []
        with open(os.path.join(self.dir, "bags.csv"), "w", newline="") as f:
            csv.writer(f).writerow(BAG_FIELDS)
        with open(os.path.join(self.dir, "carts.csv"), "w", newline="") as f:
            csv.writer(f).writerow(["cart", "bags", "fill_ratio", "stack_height"])
        self.create_subscription(BagResult, "/cell/bag_result", self.on_bag, 50)
        self.create_subscription(CartDispatch, "/cell/cart_dispatch", self.on_cart, 50)
        self.get_logger().info(f"Logging to {self.dir}")

    def on_bag(self, m):
        row = [m.bag, m.tag, m.flight, m.target_cart, m.actual_cart, int(m.success), m.reason,
               round(m.cycle_time, 2), round(m.planning_time, 3), m.mass,
               round(m.measured_size.x, 3), round(m.measured_size.y, 3),
               round(m.measured_size.z, 3), m.true_size.x, m.true_size.y, m.true_size.z,
               round(m.position_error, 4), round(m.yaw_error, 4)]
        self.bags.append(dict(zip(BAG_FIELDS, row)))
        with open(os.path.join(self.dir, "bags.csv"), "a", newline="") as f:
            csv.writer(f).writerow(row)
        self.write_summary()

    def on_cart(self, m):
        self.carts.append(m)
        with open(os.path.join(self.dir, "carts.csv"), "a", newline="") as f:
            csv.writer(f).writerow([m.cart, m.bags, round(m.fill_ratio, 3),
                                    round(m.stack_height, 3)])
        self.write_summary()

    def write_summary(self):
        b = self.bags
        n = len(b)
        ok = [r for r in b if r["success"]]
        sorted_ok = [r for r in ok if r["actual_cart"] == r["target_cart"]]
        rejects = [r for r in b if r["target_cart"] == "cart_reject"]
        measured = [r for r in b if r["meas_L"] > 0]
        cyc = mean([r["cycle_time"] for r in ok])
        lines = [
            "# Baggage cell run summary", "",
            f"Bags handled: **{n}**", "",
            "| Metric | Value |", "|---|---|",
            f"| Pick-and-place success | {len(ok)}/{n} ({100 * len(ok) / max(n, 1):.0f} %) |",
            f"| Correct cart (of successful picks) | {len(sorted_ok)}/{len(ok)} "
            f"({100 * len(sorted_ok) / max(len(ok), 1):.0f} %) |",
            f"| Sent to reject (no-read / no BSM / other flight / full) | {len(rejects)} |",
            f"| Mean cycle time (sim s) | {cyc:.1f} |",
            f"| Throughput, 1 robot (bags/h) | {3600 / cyc if cyc == cyc and cyc > 0 else 0:.0f} |",
            f"| Mean MoveIt planning time per bag (s) | "
            f"{mean([r['planning_time'] for r in ok]):.2f} |",
            f"| Perception: long side error (mm) | "
            f"{1000 * mean([abs(r['meas_L'] - r['true_L']) for r in measured]):.1f} |",
            f"| Perception: short side error (mm) | "
            f"{1000 * mean([abs(r['meas_S'] - r['true_S']) for r in measured]):.1f} |",
            f"| Perception: height error (mm) | "
            f"{1000 * mean([abs(r['meas_H'] - r['true_H']) for r in measured]):.1f} |",
            f"| Perception: centre error (mm) | "
            f"{1000 * mean([r['position_error'] for r in measured]):.1f} |",
            f"| Perception: yaw error (deg) | "
            f"{math.degrees(mean([r['yaw_error'] for r in measured])):.2f} |",
            "",
        ]
        if self.carts:
            lines += ["## Carts dispatched", "", "| Cart | Bags | Fill | Stack height (m) |",
                      "|---|---|---|---|"]
            lines += [f"| {c.cart} | {c.bags} | {100 * c.fill_ratio:.0f} % | "
                      f"{c.stack_height:.2f} |" for c in self.carts]
        reasons = {}
        for r in b:
            if not r["success"] or r["target_cart"] == "cart_reject":
                reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        if reasons:
            lines += ["", "## Rejects and failures", "", "| Reason | Count |", "|---|---|"]
            lines += [f"| {k} | {v} |" for k, v in sorted(reasons.items())]
        with open(os.path.join(self.dir, "summary.md"), "w") as f:
            f.write("\n".join(lines) + "\n")
        self.get_logger().info(f"{n} bags, {len(ok)} ok, {len(sorted_ok)} in the right cart, "
                               f"mean cycle {cyc:.1f} s")


def main():
    rclpy.init()
    rclpy.spin(MetricsLogger())


if __name__ == "__main__":
    main()
