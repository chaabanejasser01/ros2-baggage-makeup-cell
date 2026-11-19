cmake_minimum_required(VERSION 3.8)
project(bag_cell_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# every .msg / .srv file becomes C++ and Python code
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BagResult.msg"
  "msg/CartDispatch.msg"
  "srv/NextBag.srv"
  "srv/DetectBag.srv"
  "srv/RouteBag.srv"
  "srv/PlanPlacement.srv"
  "srv/ReportPlacement.srv"
  "srv/DispatchCart.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
