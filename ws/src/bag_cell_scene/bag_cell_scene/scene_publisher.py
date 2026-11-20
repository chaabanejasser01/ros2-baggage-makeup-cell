import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def make_box(name, size, x, y, z):
    """One box obstacle for MoveIt: name, size (sx, sy, sz), centre (x, y, z) in the world frame."""
    obj = CollisionObject()
    obj.header.frame_id = "world"
    obj.id = name

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0  # no rotation

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


def cart_boxes(name, cx, cy, back, L=1.6, W=1.2):
    """A cart L x W = base (deck at 0.35 m) + back wall + 2 end walls, 0.3 m high.
    back = +1 / -1: back wall on the +y / -y side."""
    return [make_box(f"{name}_base", (L, W, 0.35), cx, cy, 0.175),
            make_box(f"{name}_back", (L, 0.03, 0.3), cx, cy + back * (W / 2 - 0.015), 0.5),
            make_box(f"{name}_end1", (0.03, W, 0.3), cx + L / 2 - 0.015, cy, 0.5),
            make_box(f"{name}_end2", (0.03, W, 0.3), cx - L / 2 + 0.015, cy, 0.5)]


class ScenePublisher(Node):
    def __init__(self):
        super().__init__("scene_publisher")
        self.pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        # wait until move_group is listening, otherwise the message is lost
        if self.pub.get_subscription_count() == 0:
            self.get_logger().info("Waiting for move_group...")
            return

        scene = PlanningScene()
        scene.is_diff = True  # add to the existing scene, don't replace it
        scene.world.collision_objects = (
            [make_box("belt", (0.8, 6.0, 0.5), 0.9, -2.0, 0.25),   # x 0.5 .. 1.3, y -5 .. 1
             make_box("belt_end_stop", (0.9, 0.08, 0.3), 0.9, 1.04, 0.6),
             # space taken by the bags still coming on the belt (up to 30 cm tall), upstream
             # of the pick zone: the belt runs while the arm works, so the arm stays out of it
             make_box("belt_traffic", (0.8, 4.5, 0.3), 0.9, -2.75, 0.65),   # y -5 .. -0.5
             make_box("eye_emitter", (0.06, 0.06, 0.62), 0.45, 0.25, 0.31),
             make_box("eye_reflector", (0.06, 0.06, 0.62), 1.35, 0.25, 0.31),
             make_box("floor", (4.0, 4.0, 0.02), 0.0, 0.0, -0.011),
             make_box("pedestal", (0.24, 0.24, 0.295), 0.0, 0.0, 0.1475)]
            + cart_boxes("cart_a", -0.35, 1.05, +1)
            + cart_boxes("cart_b", -0.35, -1.05, -1)
            # safety fence (north, south, east, and west between the two cart openings)
            + [make_box("fence_north", (3.45, 0.06, 2.05), -0.075, 1.75, 1.025),
               make_box("fence_south", (3.45, 0.06, 2.05), -0.075, -1.75, 1.025),
               make_box("fence_east", (0.06, 3.5, 2.05), 1.65, 0.0, 1.025),
               make_box("fence_west", (0.06, 0.9, 2.05), -1.8, 0.0, 1.025)]
            # overhead camera: boom + housing above the pick zone
            + [make_box("camera_boom", (1.0, 0.08, 0.08), 1.34, 0.0, 2.08),
               make_box("camera_housing", (0.12, 0.12, 0.1), 0.9, 0.0, 1.98)]
        )
        self.pub.publish(scene)
        self.get_logger().info(f"Published {len(scene.world.collision_objects)} objects")
        self.timer.cancel()  # done, publish only once


def main():
    rclpy.init()
    node = ScenePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
