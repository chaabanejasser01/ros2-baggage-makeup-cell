# One line of the cell's log: what happened to one bag.
string bag
string tag                         # 10-digit licence plate read by the ATR ("" = no-read)
string flight
string target_cart                 # cart chosen by the flight router ("" = problem bag)
string actual_cart                 # where the bag really ended (checked in Gazebo)
bool success                       # picked and placed without a motion failure
string reason                      # problem bag (no-read, late...) / why it failed
float64 cycle_time                 # s (sim time) from bag arrival to robot free again
float64 planning_time              # s spent in MoveIt planning for this bag
float64 mass                       # kg (from the BSM)
geometry_msgs/Vector3 measured_size  # x long side, y short side, z height (camera)
geometry_msgs/Vector3 true_size      # same order, ground truth from the simulator
float64 position_error             # m, camera centre vs truth (x, y)
float64 yaw_error                  # rad, camera yaw vs truth (box symmetry removed)
