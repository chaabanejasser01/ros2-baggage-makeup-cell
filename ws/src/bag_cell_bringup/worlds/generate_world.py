#!/usr/bin/env python3
"""Generates the baggage make-up room: worlds/bag_hall.sdf + 4 sign textures.

Run it after changing anything here:
    python3 generate_world.py

Frame: robot base at (0, 0). +x = towards the belt, +y = cart A side.
Bags come out of a hatch in the south wall (y = -5.2), ride the 6 m x 0.8 m belt towards +y,
and the belt stops when a bag cuts the photo-eye beam at y = 0.25 (pick zone).
The objects the robot works near keep the SAME boxes as scene_publisher.py.
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
TEX_MODEL = "bag_hall_textures"
TEX_DIR = os.path.join(HERE, "..", "models", TEX_MODEL, "materials", "textures")
PI = math.pi

CONCRETE = (0.62, 0.62, 0.60)
WALL = (0.85, 0.85, 0.82)
WALL_LOW = (0.30, 0.36, 0.42)
STEEL = (0.55, 0.57, 0.60)
DARK = (0.25, 0.26, 0.28)
RUBBER = (0.07, 0.07, 0.07)
YELLOW = (0.95, 0.75, 0.05)
WHITE = (0.95, 0.95, 0.95)
RED = (0.80, 0.10, 0.10)
BLUE_A, GREEN_B = (0.15, 0.30, 0.70), (0.15, 0.55, 0.25)

BELT_Y0, BELT_Y1 = -5.0, 1.0      # belt runs from y0 to y1, top at z = 0.5
BELT_X, BELT_W = 0.9, 0.8        # belt centre line and width: x 0.5 .. 1.3
X0, X1 = BELT_X - BELT_W / 2 - 0.05, BELT_X + BELT_W / 2 + 0.05   # hatch / fence gap
EYE_Y = 0.25                      # photo-eye beam position


def fmt(v):
    return " ".join(f"{x:.4g}" for x in v)


def material(rgb, emissive=0.0, tex=None):
    r, g, b = rgb
    s = f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
    if emissive:
        s += f"<emissive>{r * emissive:.3g} {g * emissive:.3g} {b * emissive:.3g} 1</emissive>"
    if tex:
        s += (f"<pbr><metal><albedo_map>model://{TEX_MODEL}/materials/textures/{tex}"
              f"</albedo_map><metalness>0</metalness><roughness>0.7</roughness></metal></pbr>")
    return s + "</material>"


class Model:
    """One static model = one link holding many shapes."""

    def __init__(self, name, pose=(0, 0, 0, 0, 0, 0)):
        self.name, self.pose, self.items, self.n, self.plugins = name, pose, [], 0, ""

    def _add(self, geom, pose, color, collide, alpha, emissive, tex):
        self.n += 1
        pose = tuple(pose) + (0,) * (6 - len(pose))
        vis = (f'<visual name="v{self.n}"><pose>{fmt(pose)}</pose><geometry>{geom}</geometry>'
               + material(color, emissive, tex))
        if alpha < 1:
            vis += f"<transparency>{1 - alpha}</transparency>"
        self.items.append(vis + "</visual>")
        if collide:
            self.items.append(f'<collision name="c{self.n}"><pose>{fmt(pose)}</pose>'
                              f"<geometry>{geom}</geometry></collision>")

    def box(self, size, pose, color, collide=False, alpha=1.0, emissive=0.0, tex=None):
        self._add(f"<box><size>{fmt(size)}</size></box>", pose, color, collide, alpha,
                  emissive, tex)

    def cyl(self, r, length, pose, color, collide=False):
        self._add(f"<cylinder><radius>{r}</radius><length>{length}</length></cylinder>",
                  pose, color, collide, 1.0, 0.0, None)

    def sdf(self):
        body = "\n        ".join(self.items)
        return (f'    <model name="{self.name}">\n      <static>true</static>\n'
                f"      <pose>{fmt(self.pose)}</pose>\n      <link name=\"base_link\">\n"
                f"        {body}\n      </link>\n{self.plugins}    </model>\n")


def floor_line(m, x0, y0, x1, y1, color=YELLOW, w=0.08):
    length = math.hypot(x1 - x0, y1 - y0)
    m.box((length, w, 0.002), ((x0 + x1) / 2, (y0 + y1) / 2, 0.006, 0, 0,
                               math.atan2(y1 - y0, x1 - x0)), color)


def rect(m, x0, y0, x1, y1, color=WHITE, w=0.05):
    floor_line(m, x0, y0, x1, y0, color, w)
    floor_line(m, x1, y0, x1, y1, color, w)
    floor_line(m, x1, y1, x0, y1, color, w)
    floor_line(m, x0, y1, x0, y0, color, w)


# ---------------------------------------------------------------- textures
def make_textures():
    os.makedirs(TEX_DIR, exist_ok=True)

    def font(size):
        try:
            return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size)
        except OSError:
            return ImageFont.load_default()

    def sign(fname, bg, stripe, lines):
        w, h = 1024, 256
        img = Image.new("RGB", (w, h), bg)
        d = ImageDraw.Draw(img)
        if stripe:
            d.rectangle([0, 0, 40, h], fill=stripe)
            d.rectangle([w - 40, 0, w, h], fill=stripe)
        y = 20
        for text, size, color in lines:
            tw = d.textlength(text, font=font(size))
            d.text(((w - tw) / 2, y), text, font=font(size), fill=color)
            y += int(size * 1.15)
        img.save(os.path.join(TEX_DIR, fname))

    amber = (255, 190, 40)
    # flights change over the day (see the flight board), so the signs only name the position
    sign("sign_cart_a.png", (12, 20, 40), (40, 80, 200),
         [("MAKE-UP", 40, (200, 200, 200)), ("POSITION A", 84, amber),
          ("SEE FLIGHT BOARD", 44, (230, 230, 230))])
    sign("sign_cart_b.png", (12, 20, 40), (40, 150, 70),
         [("MAKE-UP", 40, (200, 200, 200)), ("POSITION B", 84, amber),
          ("SEE FLIGHT BOARD", 44, (230, 230, 230))])
    sign("warning_robot.png", (250, 200, 20), None,
         [("DANGER", 90, (0, 0, 0)), ("ROBOT CELL - KEEP OUT", 60, (0, 0, 0))])
    root = os.path.join(TEX_DIR, "..", "..")
    with open(os.path.join(root, "model.config"), "w") as f:
        f.write('<?xml version="1.0"?>\n<model>\n  <name>bag_hall_textures</name>\n'
                '  <version>1.0</version>\n  <sdf version="1.9">model.sdf</sdf>\n'
                "  <description>Sign textures</description>\n</model>\n")
    with open(os.path.join(root, "model.sdf"), "w") as f:
        f.write('<?xml version="1.0"?>\n<sdf version="1.9"><model name="bag_hall_textures">'
                '<static>true</static><link name="link"/></model></sdf>\n')


# ---------------------------------------------------------------- room
def room():
    """Floor, 4 walls (no roof so you can look in from above), floor markings."""
    m = Model("room")
    m.items.append('<collision name="plane"><geometry><plane><normal>0 0 1</normal>'
                   "<size>40 40</size></plane></geometry></collision>")
    m.box((10.4, 9.6, 0.004), (0, -0.6, 0.002), CONCRETE)
    H, LOW = 4.0, 1.0
    for cx, cy, sx, sy in ((-5.1, -0.6, 0.2, 9.6), (5.1, -0.6, 0.2, 9.6),
                           (0, 4.1, 10.4, 0.2)):
        m.box((sx, sy, LOW), (cx, cy, LOW / 2), WALL_LOW, collide=True)
        m.box((sx, sy, H - LOW), (cx, cy, LOW + (H - LOW) / 2), WALL, collide=True)
    # south wall with the hatch the belt comes out of (x X0..X1, z up to 1.1)
    for x0, x1 in ((-5.2, X0), (X1, 5.2)):
        m.box((x1 - x0, 0.2, LOW), ((x0 + x1) / 2, -5.3, LOW / 2), WALL_LOW, collide=True)
        m.box((x1 - x0, 0.2, H - LOW), ((x0 + x1) / 2, -5.3, LOW + (H - LOW) / 2), WALL,
              collide=True)
    m.box((X1 - X0, 0.2, H - 1.1), (BELT_X, -5.3, 1.1 + (H - 1.1) / 2), WALL, collide=True)
    for k in range(10):  # rubber strip curtain in the hatch
        m.box((0.09, 0.01, 0.5), (X0 + 0.05 + k * 0.1, -5.19, 0.85), RUBBER, alpha=0.8)
    # floor markings: robot zone (yellow), cart bays (white)
    rect(m, -1.95, -1.9, 1.8, 1.9, YELLOW, 0.08)
    rect(m, -0.75, 0.4, 0.55, 1.3)
    rect(m, -0.75, -1.3, 0.55, -0.4)
    return m


# ---------------------------------------------------------------- belt
def belt():
    """Driven conveyor (Gazebo TrackController). Model frame: x along the belt."""
    length = BELT_Y1 - BELT_Y0
    cy = (BELT_Y0 + BELT_Y1) / 2
    m = Model("belt", (BELT_X, cy, 0.25, 0, 0, PI / 2))
    # the collision box IS the moving belt surface (friction direction fdir1)
    m.items.append(
        f'<collision name="belt_surface"><geometry><box><size>{length} {BELT_W} 0.5</size></box>'
        "</geometry><surface><friction><ode><mu>0.7</mu><mu2>150</mu2><fdir1>0 1 0</fdir1>"
        "</ode></friction></surface></collision>")
    m.box((length, BELT_W - 0.04, 0.03), (0, 0, 0.235), RUBBER)           # belt top
    for v in (-BELT_W / 2 + 0.01, BELT_W / 2 - 0.01):
        m.box((length, 0.03, 0.12), (0, v, 0.19), STEEL)                  # side frames
    n = int(length / 1.2) + 1
    for i in range(n):
        u = -length / 2 + 0.1 + i * (length - 0.2) / (n - 1)
        for v in (-BELT_W / 2 + 0.04, BELT_W / 2 - 0.04):
            m.box((0.05, 0.05, 0.38), (u, v, -0.06), DARK)                # legs
        m.box((0.05, BELT_W - 0.1, 0.04), (u, 0, -0.17), DARK)
    m.box((0.08, BELT_W + 0.1, 0.3), (length / 2 + 0.04, 0, 0.35), YELLOW, collide=True)  # end stop
    m.plugins = ('      <plugin filename="gz-sim-track-controller-system" '
                 'name="gz::sim::systems::TrackController">\n'
                 "        <link>base_link</link>\n"
                 "        <odometry_publish_frequency>1</odometry_publish_frequency>\n"
                 "      </plugin>\n")
    return m


def photo_eye():
    """Laser beam across the belt 5 cm above it: emitter post (x X0) + reflector (x X1).
    Range limit: the beam ends just before the reflector, so empty belt = no reading."""
    m = Model("photo_eye")
    for x in (X0, X1):
        m.box((0.04, 0.04, 0.62), (x, EYE_Y, 0.31), DARK, collide=True)
        m.box((0.05, 0.05, 0.06), (x, EYE_Y, 0.55), (0.1, 0.1, 0.1), collide=True)
    m.box((0.01, 0.02, 0.02), (X0 + 0.03, EYE_Y, 0.55), RED, emissive=1.0)
    m.items.insert(0, f"""<sensor name="beam" type="gpu_lidar">
          <pose>{X0 + 0.03} {EYE_Y} 0.55 0 0 0</pose>
          <topic>photo_eye</topic>
          <update_rate>50</update_rate>
          <always_on>true</always_on>
          <lidar>
            <scan><horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal></scan>
            <range><min>0.02</min><max>{X1 - X0 - 0.08:.2f}</max><resolution>0.001</resolution></range>
          </lidar>
        </sensor>""")
    return m


def tag_reader():
    """ATR tag-reader arch over the belt, upstream (outside the fence)."""
    m = Model("tag_reader")
    for x in (X0 - 0.1, X1 + 0.1):
        m.box((0.1, 0.25, 1.3), (x, -2.6, 0.65), (0.3, 0.3, 0.32), collide=True)
        m.box((0.02, 0.2, 0.5), (x + (0.06 if x < BELT_X else -0.06), -2.6, 0.8), (0.05, 0.25, 0.1),
              emissive=0.6)
    m.box((X1 - X0 + 0.3, 0.35, 0.25), (BELT_X, -2.6, 1.42), (0.3, 0.3, 0.32), collide=True)
    return m


# ---------------------------------------------------------------- the cell
def cart(name, pose, color, back, L=1.6, W=1.2):
    """Baggage dolly L x W, deck at 0.35 m, 0.3 m walls on 3 sides (front open).
    Collision = base box + 3 walls (same as scene_publisher.py).
    back = +1 / -1: back wall on local +y / -y. Tow bar on the -x end (west, the tug side)."""
    m = Model(name, pose)
    walls = (("back", (L, 0.03, 0.3), (0, back * (W / 2 - 0.015), 0.5)),
             ("end1", (0.03, W, 0.3), (L / 2 - 0.015, 0, 0.5)),
             ("end2", (0.03, W, 0.3), (-L / 2 + 0.015, 0, 0.5)))
    m.items.append('<collision name="base"><pose>0 0 0.175 0 0 0</pose><geometry><box>'
                   f"<size>{L} {W} 0.35</size></box></geometry></collision>")
    for nm, size, p in walls:
        m.items.append(f'<collision name="{nm}"><pose>{fmt(p)} 0 0 0</pose><geometry>'
                       f"<box><size>{fmt(size)}</size></box></geometry></collision>")
        m.box(size, p, color)
    m.box((L, W, 0.05), (0, 0, 0.325), (0.6, 0.6, 0.62))                  # deck
    m.box((L - 0.08, W - 0.1, 0.06), (0, 0, 0.26), DARK)                  # chassis
    for x in (-L / 2 + 0.12, L / 2 - 0.12):
        for y in (-W / 2 + 0.06, W / 2 - 0.06):
            m.cyl(0.11, 0.07, (x, y, 0.11, PI / 2, 0, 0), RUBBER)         # wheels
    m.box((0.55, 0.04, 0.04), (-L / 2 - 0.28, 0, 0.18), DARK)                # tow bar
    return m


def pedestal():
    m = Model("pedestal")
    m.items.append('<collision name="c"><pose>0 0 0.1475 0 0 0</pose><geometry><box>'
                   "<size>0.24 0.24 0.295</size></box></geometry></collision>")
    m.box((0.45, 0.45, 0.02), (0, 0, 0.01), DARK)
    m.box((0.24, 0.24, 0.275), (0, 0, 0.1575), (0.9, 0.45, 0.1))
    return m


def fence():
    """2 m safety fence. North/south/east walls = same boxes as in scene_publisher.py.
    South side has a gap where the belt comes in; west side is open for the carts
    (light-curtain posts)."""
    m = Model("fence")
    PANEL = (0.25, 0.27, 0.30)

    def run(x0, y0, x1, y1):
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, math.ceil(length / 1.2))
        for i in range(n + 1):
            t = i / n
            m.box((0.06, 0.06, 2.0), (x0 + t * (x1 - x0), y0 + t * (y1 - y0), 1.0), YELLOW,
                  collide=True)
        m.box((length, 0.02, 1.85), ((x0 + x1) / 2, (y0 + y1) / 2, 1.05, 0, 0, yaw), PANEL,
              collide=True, alpha=0.45)

    run(-1.8, 1.75, 1.65, 1.75)                       # north
    run(-1.8, -1.75, X0, -1.75)                       # south (belt gap x X0..X1)
    run(X1, -1.75, 1.65, -1.75)
    m.box((X1 - X0, 0.02, 0.9), (BELT_X, -1.75, 1.55), PANEL, collide=True, alpha=0.45)
    run(1.65, -1.75, 1.65, 1.75)                      # east
    m.box((0.03, 0.8, 0.2), (1.68, 0.0, 1.6), YELLOW, tex="warning_robot.png")
    # west: open for the carts (1.2 m wide openings), light-curtain posts on each side
    run(-1.8, -0.44, -1.8, 0.44)                      # closed between the two carts
    for y in (-1.7, -0.44, 0.44, 1.7):
        m.box((0.08, 0.08, 1.9), (-1.8, y, 0.95), YELLOW, collide=True)
        m.box((0.02, 0.03, 1.6), (-1.75, y, 0.9), RED, emissive=0.8)
    # one sign above each cart opening, facing the operators outside
    for y in (-1.75, 1.75):
        m.box((0.08, 0.08, 2.7), (-2.0, y, 1.35), DARK, collide=True)
    m.box((0.08, 3.6, 0.08), (-2.0, 0, 2.7), DARK)
    for y, tex in ((1.05, "sign_cart_a.png"), (-1.05, "sign_cart_b.png")):
        m.box((0.03, 0.8, 0.2), (-2.0, y, 2.5), WHITE, tex=tex)
    return m


def camera_mast():
    """Overhead RGB-D camera above the pick zone, on a mast outside the fence."""
    m = Model("camera_mast")
    m.box((0.12, 0.12, 2.12), (1.8, 0, 1.06), DARK, collide=True)
    m.box((1.0, 0.08, 0.08), (1.34, 0, 2.08), DARK, collide=True)
    m.box((0.12, 0.12, 0.1), (0.9, 0, 1.98), (0.05, 0.05, 0.05), collide=True)
    return m


CAMERA = """    <model name="overhead_camera">
      <static>true</static>
      <pose>0.9 0 1.92 0 1.5708 0</pose>
      <link name="link">
        <sensor name="overhead_cam" type="rgbd_camera">
          <topic>overhead_cam</topic>
          <update_rate>5</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>0.9</horizontal_fov>
            <image><width>320</width><height>240</height></image>
            <clip><near>0.2</near><far>4.0</far></clip>
            <depth_camera><clip><near>0.2</near><far>4.0</far></clip></depth_camera>
          </camera>
        </sensor>
      </link>
    </model>
"""

HEADER = """<?xml version="1.0"?>
<!-- GENERATED by generate_world.py - edit that file, not this one. -->
<sdf version="1.9">
  <world name="bag_hall">
    <physics name="1ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.75 0.8 0.85 1</background>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.8 1</diffuse>
      <direction>-0.4 0.3 -0.85</direction>
    </light>
"""


def build(extra=""):
    models = [room(), belt(), photo_eye(), tag_reader(), pedestal(), fence(), camera_mast(),
              cart("cart_a", (-0.35, 1.05, 0, 0, 0, 0), BLUE_A, 1),
              cart("cart_b", (-0.35, -1.05, 0, 0, 0, 0), GREEN_B, -1)]
    return HEADER + CAMERA + "".join(m.sdf() for m in models) + extra + "  </world>\n</sdf>\n"


if __name__ == "__main__":
    for f in os.listdir(TEX_DIR) if os.path.isdir(TEX_DIR) else []:
        os.remove(os.path.join(TEX_DIR, f))  # drop old textures
    make_textures()
    with open(os.path.join(HERE, "bag_hall.sdf"), "w") as f:
        f.write(build())
    print("wrote bag_hall.sdf")
