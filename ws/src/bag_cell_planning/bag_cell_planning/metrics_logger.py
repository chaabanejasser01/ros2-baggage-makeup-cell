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
            csv.writer(f).writerow(["cart", "flight", "reason", "bags", "fill_ratio",
                                    "stack_height"])
        self.t_start = None   # sim time when the first bag arrived
        self.create_subscription(BagResult, "/cell/bag_result", self.on_bag, 50)
        self.create_subscription(CartDispatch, "/cell/cart_dispatch", self.on_cart, 50)
        self.get_logger().info(f"Logging to {self.dir}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_bag(self, m):
        if self.t_start is None:
            self.t_start = self.now() - m.cycle_time
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
            csv.writer(f).writerow([m.cart, m.flight, m.reason, m.bags, round(m.fill_ratio, 3),
                                    round(m.stack_height, 3)])
        self.write_summary()

    def write_summary(self):
        b = self.bags
        n = len(b)
        ok = [r for r in b if r["success"]]
        sorted_ok = [r for r in ok if r["actual_cart"] == r["target_cart"]]
        measured = [r for r in b if r["meas_L"] > 0]
        cyc = mean([r["cycle_time"] for r in ok])
        elapsed = self.now() - self.t_start if self.t_start is not None else 0.0
        placed = [r for r in ok if r["actual_cart"]]
        lines = [
            "# Baggage cell run summary", "",
            f"Bags handled: **{n}**", "",
            "| Metric | Value |", "|---|---|",
            f"| Pick-and-place success | {len(ok)}/{n} ({100 * len(ok) / max(n, 1):.0f} %) |",
            f"| Correct cart (of successful picks) | {len(sorted_ok)}/{len(ok)} "
            f"({100 * len(sorted_ok) / max(len(ok), 1):.0f} %) |",
            f"| Mean cycle time (sim s) | {cyc:.1f} |",
            f"| Elapsed (sim s) | {elapsed:.0f} |",
            f"| Throughput, measured (bags/h) | "
            f"{3600 * len(placed) / elapsed if elapsed > 0 else 0:.0f} |",
            f"| Throughput, robot limit 3600/cycle (bags/h) | "
            f"{3600 / cyc if cyc == cyc and cyc > 0 else 0:.0f} |",
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
            lines += ["## Carts dispatched", "",
                      "| Cart | Flight | Why | Bags | Fill | Stack height (m) |",
                      "|---|---|---|---|---|---|"]
            lines += [f"| {c.cart} | {c.flight or '-'} | {c.reason or '-'} | {c.bags} | "
                      f"{100 * c.fill_ratio:.0f} % | {c.stack_height:.2f} |" for c in self.carts]
        flights = {}
        for r in b:
            if not r["flight"]:
                continue
            f = flights.setdefault(r["flight"], {"bags": 0, "loaded": 0, "why": {}})
            f["bags"] += 1
            if r["success"] and r["actual_cart"] == r["target_cart"]:
                f["loaded"] += 1
            else:
                f["why"][r["reason"]] = f["why"].get(r["reason"], 0) + 1
        if flights:
            lines += ["", "## Per flight", "", "| Flight | Bags | Loaded on its cart | "
                      "Not loaded (why) |", "|---|---|---|---|"]
            lines += [f"| {k} | {v['bags']} | {v['loaded']} | " +
                      (", ".join(f"{n} {why}" for why, n in v["why"].items()) or "-") + " |"
                      for k, v in sorted(flights.items())]
        reasons = {}
        for r in b:
            if not r["success"]:
                reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        if reasons:
            lines += ["", "## Failures", "", "| Reason | Count |", "|---|---|"]
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
