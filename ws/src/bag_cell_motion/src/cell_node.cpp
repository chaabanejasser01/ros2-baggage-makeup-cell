// Baggage make-up cell: the main loop, one bag at a time.
//
//   1. next bag               /next_bag         (belt runs until the photo-eye, tag is read)
//                                                (asked as soon as the bag is lifted off the
//                                                 belt: the next bag comes while the arm works)
//   2. measure it             /detect_bag       (overhead depth camera)
//   3. which cart?            /route_bag        (tag -> flight -> cart, check-in manifest)
//   4. where in the cart?     /plan_placement   (next free slot)
//      cart full?             /dispatch_cart    (tug takes it, empty cart comes) then re-plan
//   5. pick and place         MoveIt (OMPL for free moves, straight lines near the bag)
//   6. check where it landed  /report_placement (true pose from Gazebo)
//   7. log                    /cell/bag_result  (metrics_logger writes the CSV)
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <bag_cell_interfaces/msg/bag_result.hpp>
#include <bag_cell_interfaces/srv/detect_bag.hpp>
#include <bag_cell_interfaces/srv/dispatch_cart.hpp>
#include <bag_cell_interfaces/srv/plan_placement.hpp>
#include <bag_cell_interfaces/srv/report_placement.hpp>
#include <bag_cell_interfaces/srv/route_bag.hpp>
#include <bag_cell_interfaces/srv/next_bag.hpp>

using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Vector3;
namespace srv = bag_cell_interfaces::srv;
using bag_cell_interfaces::msg::BagResult;

const double TOOL_LENGTH = 0.145;  // flange (tool0) -> cup face
const double BELT_TOP = 0.50;
const double APPROACH = 0.10;      // cup starts 10 cm above the bag top
const double LIFT = 0.15;          // bag bottom 15 cm above the belt after the grip
const std::string WORLD = "/world/bag_hall";

// tool0 pose so that the cup TIP is at (x, y, z), cup pointing down, turned by yaw
Pose tip_pose(double x, double y, double z, double yaw)
{
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z + TOOL_LENGTH;
  p.orientation.x = std::cos(yaw / 2);  // = rotZ(yaw) * rotX(180 deg)
  p.orientation.y = std::sin(yaw / 2);
  p.orientation.z = 0.0;
  p.orientation.w = 0.0;
  return p;
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

// a box looks the same every 180 deg: smallest yaw difference modulo pi
double box_yaw_diff(double a, double b)
{
  return std::fabs(std::remainder(a - b, M_PI));
}

class Cell
{
public:
  explicit Cell(rclcpp::Node::SharedPtr node)
  : node_(node), arm_(node, "ur_manipulator"), log_(node->get_logger())
  {
    double v = node->get_parameter("velocity_scaling").as_double();
    arm_.setEndEffectorLink("tool0");
    arm_.setMaxVelocityScalingFactor(v);
    arm_.setMaxAccelerationScalingFactor(v);
    arm_.setPlanningTime(5.0);

    next_ = node->create_client<srv::NextBag>("next_bag");
    detect_ = node->create_client<srv::DetectBag>("detect_bag");
    route_ = node->create_client<srv::RouteBag>("route_bag");
    plan_ = node->create_client<srv::PlanPlacement>("plan_placement");
    report_ = node->create_client<srv::ReportPlacement>("report_placement");
    dispatch_ = node->create_client<srv::DispatchCart>("dispatch_cart");
    result_pub_ = node->create_publisher<BagResult>("/cell/bag_result", 10);
    // Gazebo entity ids of the bags (needed to add the grip plugin to a bag)
    gz_.Subscribe(WORLD + "/dynamic_pose/info", &Cell::on_gz_poses, this);
  }

  // home = the start pose: arm standing straight up, base turned to face the belt
  // (same joint values as initial_positions in bag_cell_robot.urdf.xacro)
  bool go_home()
  {
    arm_.setStartStateToCurrentState();
    arm_.setJointValueTarget(std::vector<double>{-1.5708, -1.57, 0.0, -1.57, 0.0, 0.0});
    MoveGroup::Plan plan;
    bool ok = arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      arm_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    RCLCPP_INFO(log_, "  %-32s %s", "home (standing, facing the belt)", ok ? "OK" : "FAILED");
    return ok;
  }

  // One bag, start to finish (the 7 steps at the top of this file).
  // Returns false when there are no more bags.
  // more = another bag comes after this one (ask the belt for it early)
  bool process_bag(bool more)
  {
    BagResult r;
    planning_time_ = 0.0;

    // 1. the bag at the photo-eye
    auto bag = wait_for_bag();
    if (!bag) {return false;}
    const auto t0 = node_->now();   // cycle time = from the bag at the pick zone to arm free
    r.bag = bag->name;
    r.tag = bag->tag;
    r.true_size = bag->true_size;
    RCLCPP_INFO(log_, "==== %s at the pick zone (tag '%s')", r.bag.c_str(), r.tag.c_str());

    // 2. measure it
    auto det = call<srv::DetectBag>(detect_, std::make_shared<srv::DetectBag::Request>());
    if (!det || !det->found) {
      return give_up(r, "camera: " + (det ? det->message : std::string("no answer")), t0,
          more, true);
    }
    score_camera(r, *det, *bag);

    // 3. which cart?
    std::string cart = route(r);
    if (cart.empty()) {return give_up(r, "no cart: " + r.reason, t0, more, true);}

    // 4. where in the cart? (cart full: it is dispatched, an empty one comes)
    auto plan = plan_in(cart, det->size);
    if (!plan) {
      dispatch(cart, r.flight);
      plan = plan_in(cart, det->size);
    }
    r.target_cart = cart;
    if (!plan) {return give_up(r, "no placement found", t0, more, true);}

    // 5. pick it up (if that fails the arm may be over the belt: home first)
    std::string why;
    if (!pick(r.bag, *det, why)) {return give_up(r, why, t0, more, false);}
    // the bag is off the belt (it restarts by itself): ask for the next one meanwhile
    if (more) {request_next_bag();}

    // 6. into the cart. Never let go over the belt (the bag would fall into the photo-eye
    //    beam and pass for the next one).
    bool retreat = false;
    if (!place(cart, *plan, r.bag, retreat)) {   // still holding it: home, operator takes it
      go_home();
      release(r.bag);
      return give_up(r, "no path to " + cart + ", bag taken by operator", t0, more, true);
    }

    // 7. where did it really land? then log the bag
    r.actual_cart = report(cart, r, det->size);
    finish(r, retreat, retreat ? r.reason : "retreat failed", t0);
    return end_cycle(more, retreat);
  }

private:
  // ---- the steps of process_bag()

  // wait for the bag at the photo-eye (asked for early, or ask now)
  srv::NextBag::Response::SharedPtr wait_for_bag()
  {
    if (!next_bag_.valid()) {request_next_bag();}
    srv::NextBag::Response::SharedPtr bag;
    // (long wait: between two flights the belt can be empty for a while)
    if (next_bag_.wait_for(900s) == std::future_status::ready) {bag = next_bag_.get();}
    next_bag_ = {};
    if (!bag || !bag->success) {
      RCLCPP_ERROR(log_, "no bag: %s", bag ? bag->message.c_str() : "belt not answering");
      return nullptr;
    }
    return bag;
  }

  // how far off was the camera? (true pose from Gazebo, for the metrics only)
  void score_camera(BagResult & r, const srv::DetectBag::Response & det,
    const srv::NextBag::Response & bag)
  {
    r.measured_size = det.size;
    r.position_error = std::hypot(det.pose.position.x - bag.true_pose.position.x,
        det.pose.position.y - bag.true_pose.position.y);
    r.yaw_error = box_yaw_diff(yaw_of(det.pose.orientation), yaw_of(bag.true_pose.orientation));
    RCLCPP_INFO(log_, "camera: %s", det.message.c_str());
  }

  // tag -> flight -> cart (router down: "" = operator)
  std::string route(BagResult & r)
  {
    auto req = std::make_shared<srv::RouteBag::Request>();
    req->tag = r.tag;
    auto res = call<srv::RouteBag>(route_, req);
    std::string cart = res ? res->cart : "";
    r.flight = res ? res->flight : "";
    r.mass = res ? res->mass : 10.0;
    r.reason = res ? res->reason : "router down";
    RCLCPP_INFO(log_, "route: %s -> %s (%s)", r.flight.c_str(), cart.c_str(), r.reason.c_str());
    return cart;
  }

  // above the bag, straight down onto it, suction on, straight up
  bool pick(const std::string & bag, const srv::DetectBag::Response & det, std::string & why)
  {
    const double px = det.pose.position.x;
    const double py = det.pose.position.y;
    const double top = BELT_TOP + det.size.z;
    add_bag_to_scene(bag, det.pose, det.size);
    double yaw;   // bag yaw or bag yaw + 180 deg, MoveIt picks the easier one
    if (!move_to(px, py, top + APPROACH, yaw_of(det.pose.orientation), "above the bag", yaw) ||
      !straight_to(tip_pose(px, py, top + 0.002, yaw), "down onto the bag"))
    {
      why = "pick approach failed";
      return false;
    }
    if (!grip(bag, det.pose, det.size)) {
      why = "grip failed";
      return false;
    }
    if (!straight_to(tip_pose(px, py, top + LIFT, yaw), "lift")) {
      release(bag);   // still just above the belt: set it down, an operator takes it off
      why = "lift failed";
      return false;
    }
    return true;
  }

  // where did the bag really land? (cart manager reads it from Gazebo)
  std::string report(const std::string & cart, const BagResult & r, const Vector3 & size)
  {
    auto req = std::make_shared<srv::ReportPlacement::Request>();
    req->cart = cart;
    req->bag = r.bag;
    req->size = size;
    req->mass = r.mass;
    auto res = call<srv::ReportPlacement>(report_, req);
    return res ? res->actual_cart : "unknown";
  }

  // the bag failed: an operator takes it away, log it, go on with the next one
  bool give_up(BagResult & r, const std::string & why, const rclcpp::Time & t0, bool more,
    bool arm_clear)
  {
    remove_bag(r.bag);
    finish(r, false, why, t0);
    return end_cycle(more, arm_clear);
  }

  // ---- helpers
  // ask the belt for the next bag, after a delay (runs in the background)
  void request_next_bag(std::chrono::milliseconds delay = 0ms)
  {
    next_bag_ = std::async(std::launch::async, [this, delay]() {
          std::this_thread::sleep_for(delay);
          // (a request sent before the service is discovered is lost: the cell would wait forever)
          if (!next_->wait_for_service(30s)) {return srv::NextBag::Response::SharedPtr();}
          auto f = next_->async_send_request(std::make_shared<srv::NextBag::Request>()).future;
          return f.wait_for(910s) == std::future_status::ready ? f.get() : nullptr;
        }).share();
  }

  // end of a bag: the belt brings the next one while the arm goes home.
  // arm_clear = the arm is away from the belt (over a cart), so the belt can run at once.
  bool end_cycle(bool more, bool arm_clear)
  {
    if (more && arm_clear && !next_bag_.valid()) {request_next_bag();}
    // next bag at the photo-eye (or arriving within 1.5 s): go straight to it, the arm is
    // over a cart, out of the camera view. Otherwise wait at home, out of the way.
    bool waiting = arm_clear && next_bag_.valid() &&
      next_bag_.wait_for(1500ms) == std::future_status::ready;
    if (!waiting) {go_home();}
    if (more && !next_bag_.valid()) {request_next_bag();}
    return true;
  }

  template<typename S>
  typename S::Response::SharedPtr call(
    typename rclcpp::Client<S>::SharedPtr client,
    typename S::Request::SharedPtr req, std::chrono::seconds timeout = 60s)
  {
    if (!client->wait_for_service(10s)) {
      RCLCPP_ERROR(log_, "service %s not available", client->get_service_name());
      return nullptr;
    }
    auto future = client->async_send_request(req);
    if (future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(log_, "service %s timed out", client->get_service_name());
      return nullptr;
    }
    return future.get();
  }

  srv::PlanPlacement::Response::SharedPtr plan_in(const std::string & cart, const Vector3 & size)
  {
    auto req = std::make_shared<srv::PlanPlacement::Request>();
    req->cart = cart;
    req->size = size;
    auto res = call<srv::PlanPlacement>(plan_, req);
    if (res) {RCLCPP_INFO(log_, "planner: %s", res->message.c_str());}
    return res && res->ok ? res : nullptr;
  }

  void dispatch(const std::string & cart, const std::string & flight)
  {
    auto req = std::make_shared<srv::DispatchCart::Request>();
    req->cart = cart;
    req->flight = flight;
    req->reason = "full";
    auto res = call<srv::DispatchCart>(dispatch_, req);
    if (res) {
      RCLCPP_WARN(log_, "%s full -> dispatched with %d bags (%.0f %% fill)", cart.c_str(),
        res->bags, 100 * res->fill_ratio);
    }
    std::this_thread::sleep_for(1s);  // let MoveIt get the updated scene
  }

  // Free-space move with OMPL (goes around obstacles) to the cup tip at (x, y, z).
  // A suitcase is symmetric, so the cup may be turned by yaw OR yaw + 180 deg: both are given
  // to MoveIt as goals and it uses whichever it reaches. reached_yaw = the one it used.
  // The joint limits of the cell (bag_cell_description/config/joint_limits.yaml) keep the
  // arm in one configuration family.
  // Put the held bag in the cart: above the slot, straight down, let go, back up.
  // Returns false only if the arm could not get above the cart (it still holds the bag).
  bool place(const std::string & cart, const srv::PlanPlacement::Response & p,
    const std::string & bag, bool & retreat_ok)
  {
    const auto & t = p.place_tip;
    double yaw = yaw_of(t.orientation);   // slot yaw or + 180 deg: both line the bag up
    if (!move_to(t.position.x, t.position.y, p.above_tip_z, yaw, "above the slot in " + cart,
      yaw))
    {
      return false;
    }
    // (if this stops short, the bag just drops a bit further into the cart)
    straight_to(tip_pose(t.position.x, t.position.y, t.position.z, yaw), "down into " + cart);
    release(bag);
    std::this_thread::sleep_for(300ms);  // let it drop the last 2 cm
    retreat_ok = straight_to(tip_pose(t.position.x, t.position.y, p.above_tip_z, yaw),
        "retreat");
    return true;
  }

  bool move_to(double x, double y, double z, double yaw, const std::string & what,
    double & reached_yaw)
  {
    arm_.setStartStateToCurrentState();
    arm_.setPoseTargets({tip_pose(x, y, z, yaw), tip_pose(x, y, z, yaw + M_PI)});
    MoveGroup::Plan plan;
    // OMPL is random: now and then its path fails MoveIt's final collision check (e.g. the
    // hanging bag grazes the belt end stop). A new try usually finds a clean one.
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
      ok = arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      planning_time_ += plan.planning_time;
      if (!ok) {RCLCPP_WARN(log_, "  %-32s plan %d failed, trying again", what.c_str(),
          attempt + 1);}
    }
    ok = ok && arm_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    reached_yaw = yaw_of(arm_.getCurrentPose().pose.orientation);
    RCLCPP_INFO(log_, "  %-32s %s", what.c_str(), ok ? "OK" : "FAILED");
    return ok;
  }

  bool straight_to(const Pose & target, const std::string & what)
  {
    arm_.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory traj;
    // 1 cm steps; collisions off: the cup has to touch the bag, the bag has to touch the cart
    double fraction = arm_.computeCartesianPath({target}, 0.01, traj, false);
    bool ok = fraction > 0.99 && arm_.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
    RCLCPP_INFO(log_, "  %-32s %s (%.0f %% of path)", what.c_str(), ok ? "OK" : "FAILED",
      fraction * 100);
    return ok;
  }

  moveit_msgs::msg::CollisionObject box(const std::string & id, const Pose & pose,
    const Vector3 & size)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.id = id;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {size.x, size.y, size.z};
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
    return obj;
  }

  // the bag on the belt is an obstacle until we grab it
  void add_bag_to_scene(const std::string & name, const Pose & pose, const Vector3 & size)
  {
    scene_.applyCollisionObject(box(name, pose, size));
  }

  // suction ON = weld the bag to the wrist in Gazebo: add the DetachableJoint plugin to the
  // bag now (it attaches as soon as it loads). Suction OFF = its detach topic (release()).
  // Talks to Gazebo with gz-transport directly (no 'gz' command line: ~1 s per call).
  bool grip(const std::string & name, const Pose & pose, const Vector3 & size)
  {
    uint64_t id = 0;
    for (int i = 0; i < 40 && !id; ++i) {   // the pose stream has every moving model
      {
        std::lock_guard<std::mutex> lock(ids_mutex_);
        auto it = gz_ids_.find(name);
        if (it != gz_ids_.end()) {id = it->second;}
      }
      if (!id) {std::this_thread::sleep_for(50ms);}
    }
    if (!id) {
      RCLCPP_ERROR(log_, "  %-32s FAILED (%s not in Gazebo)", "grip", name.c_str());
      return false;
    }
    gz::msgs::EntityPlugin_V req;
    req.mutable_entity()->set_id(id);
    auto * plugin = req.add_plugins();
    plugin->set_name("gz::sim::systems::DetachableJoint");
    plugin->set_filename("gz-sim-detachable-joint-system");
    plugin->set_innerxml("<parent_link>link</parent_link><child_model>ur</child_model>"
      "<child_link>wrist_3_link</child_link><detach_topic>/" + name + "/detach</detach_topic>");
    gz::msgs::Boolean rep;
    bool result = false;
    if (!gz_.Request(WORLD + "/entity/system/add", req, 3000, rep, result) || !result) {
      RCLCPP_ERROR(log_, "  %-32s FAILED (Gazebo did not add the plugin)", "grip");
      return false;
    }
    // the release publisher is made now so Gazebo has found it before we need it
    detach_pub_ = gz_.Advertise<gz::msgs::Empty>("/" + name + "/detach");
    moveit_msgs::msg::AttachedCollisionObject att;
    att.link_name = "suction_tool";
    att.touch_links = {"suction_tool", "wrist_3_link"};
    att.object = box(name, pose, size);
    scene_.applyAttachedCollisionObject(att);
    RCLCPP_INFO(log_, "  %-32s OK", "grip");
    return true;
  }

  void release(const std::string & name)
  {
    gz::msgs::Empty msg;
    detach_pub_.Publish(msg);
    std::this_thread::sleep_for(100ms);
    detach_pub_.Publish(msg);   // a second time, just in case (extra detaches are ignored)
    moveit_msgs::msg::AttachedCollisionObject att;
    att.link_name = "suction_tool";
    att.object.id = name;
    att.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;  // back to the world
    scene_.applyAttachedCollisionObject(att);
    RCLCPP_INFO(log_, "  %-32s OK", "release");
  }

  // an operator takes the bag away (failure case)
  void remove_bag(const std::string & name)
  {
    gz::msgs::Entity req;
    req.set_name(name);
    req.set_type(gz::msgs::Entity::MODEL);
    gz::msgs::Boolean rep;
    bool result = false;
    if (!gz_.Request(WORLD + "/remove", req, 3000, rep, result) || !result) {
      RCLCPP_WARN(log_, "could not remove %s", name.c_str());
    }
    scene_.removeCollisionObjects({name});
  }

  void finish(BagResult & r, bool success, const std::string & reason,
    const rclcpp::Time & t0)
  {
    r.success = success;
    r.reason = reason;
    r.cycle_time = (node_->now() - t0).seconds();
    r.planning_time = planning_time_;
    result_pub_->publish(r);
    RCLCPP_INFO(log_, "==== %s: %s, target %s, landed in %s, %.1f s", r.bag.c_str(),
      success ? "DONE" : "FAILED", r.target_cart.c_str(), r.actual_cart.c_str(), r.cycle_time);
  }

  void on_gz_poses(const gz::msgs::Pose_V & msg)
  {
    std::lock_guard<std::mutex> lock(ids_mutex_);
    for (const auto & p : msg.pose()) {
      if (p.name().rfind("bag_", 0) == 0) {gz_ids_[p.name()] = p.id();}
    }
  }

  rclcpp::Node::SharedPtr node_;
  MoveGroup arm_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp::Logger log_;
  rclcpp::Client<srv::NextBag>::SharedPtr next_;
  rclcpp::Client<srv::DetectBag>::SharedPtr detect_;
  rclcpp::Client<srv::RouteBag>::SharedPtr route_;
  rclcpp::Client<srv::PlanPlacement>::SharedPtr plan_;
  rclcpp::Client<srv::ReportPlacement>::SharedPtr report_;
  rclcpp::Client<srv::DispatchCart>::SharedPtr dispatch_;
  rclcpp::Publisher<BagResult>::SharedPtr result_pub_;
  double planning_time_ = 0.0;
  std::shared_future<srv::NextBag::Response::SharedPtr> next_bag_;
  // (declared before gz_: members are destroyed in reverse order, so gz_ stops its
  //  callbacks before the map they write to is gone)
  std::mutex ids_mutex_;
  std::map<std::string, uint64_t> gz_ids_;
  gz::transport::Node gz_;
  gz::transport::Node::Publisher detach_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "cell_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (!node->has_parameter("num_bags")) {node->declare_parameter("num_bags", 10);}
  if (!node->has_parameter("velocity_scaling")) {node->declare_parameter("velocity_scaling", 0.6);}
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});

  {
    Cell cell(node);
    // no move at startup: the arm waits in its start pose until the first bag stops
    const int n = node->get_parameter("num_bags").as_int();
    for (int i = 0; i < n && rclcpp::ok(); ++i) {
      RCLCPP_INFO(node->get_logger(), "---------------- bag %d / %d", i + 1, n);
      if (!cell.process_bag(i + 1 < n)) {break;}
    }
    cell.go_home();
  }
  RCLCPP_INFO(node->get_logger(), "Done. Results in the metrics_logger folder (summary.md).");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
