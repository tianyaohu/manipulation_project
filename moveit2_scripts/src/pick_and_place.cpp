#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <functional>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <random>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// Function to generate a random number in the specified range
double getRandomNumber(double rangeMin, double rangeMax) {
  // Set up a random number generator
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the range
  std::uniform_real_distribution<double> distribution(rangeMin, rangeMax);

  // Generate a random number
  return distribution(gen);
}

void ui_loop() {
  char userInput;

  int i = 0;

  std::cout << "doing UI loop " << std::endl;

  while (rclcpp::ok()) {
    // keep printing ui instruction every 3 commands
    if (i % 3 == 0) {
      std::cout << "Enter a direction (k for left, ; for right, o for front, . "
                   "for back, q to quit): ";
    }
    i++;

    // Get User Input
    std::cin >> userInput;

    if (userInput == 'q') {
      std::cout << "Exiting the program." << std::endl;
      break; // exit the while loop if 'q' is entered
    }

    switch (userInput) {
    case 'q':
      std::cout << "Executing pickup" << std::endl;
      break;
    case 'k':
      std::cout << "Left" << std::endl;
    //   translate(move_group_arm, 0.1, [](geometry_msgs::msg::Pose &pose) {
    //     pose.position.x += 0.1; // Modify the X coordinate
    //   });
    case ';':
      std::cout << "Right" << std::endl;
    //   translate(move_group_arm, 0.1, [](geometry_msgs::msg::Pose &pose) {
    //     pose.position.x -= 0.1; // Modify the X coordinate
    //   });
    case 'o':
      std::cout << "Front" << std::endl;
      // Example usage for translating along Y axis
    //   translate(move_group_arm, 0.1, [](geometry_msgs::msg::Pose &pose) {
    //     pose.position.y += 0.1; // Modify the Y coordinate
    //   });
    case '.':
      std::cout << "Back" << std::endl;
    //   translate(move_group_arm, 0.1, [](geometry_msgs::msg::Pose &pose) {
    //     pose.position.y -= 0.1; // Modify the Y coordinate
    //   });
    default:
      std::cout << "Invalid input" << std::endl;
    }
  }
}

void translate_ee(
    moveit::planning_interface::MoveGroupInterface &move_group_arm,
    int num_step,
    const std::function<void(geometry_msgs::msg::Pose &)> &modify_pose,
    const double jump_threshold = 0.0, const double eef_step = 0.01) {

  RCLCPP_INFO(LOGGER, "Translating End Effector!");

  // get target pose
  geometry_msgs::msg::Pose target_pose = move_group_arm.getCurrentPose().pose;

  // init and define way points
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;

  for (int i = 0; i < num_step; i++) {
    modify_pose(target_pose); // Use the lambda to modify the pose
    approach_waypoints.push_back(target_pose);
  }

  moveit_msgs::msg::RobotTrajectory trajectory_approach;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);
}

float calc_gripper_value(float distance) {
  float value =
      ((-140840.9278) * powf(distance, 5.0) +
       (+22667.9431) * powf(distance, 4.0) +
       (-1551.6277) * powf(distance, 3.0) + (+38.2691) * powf(distance, 2.0) +
       (-8.0630) * powf(distance, 1.0) + (+0.8047) * powf(distance, 0.0));
  return value;
}

bool close_gripper(
    moveit::planning_interface::MoveGroupInterface &move_group_gripper,
    float length) {
  RCLCPP_INFO(LOGGER, "Close Gripper! with value %f", length);

  std::vector<double> joint_group_positions_gripper =
      move_group_gripper.getCurrentJointValues();

  // clear
  //   joint_group_positions_gripper.clear();

  // set gripper value
  joint_group_positions_gripper[2] = calc_gripper_value(
      length); // Adjust the index based on your gripper joints

  // set gripper
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

  // Plan and execute the motion
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool gripper_success = (move_group_gripper.plan(my_plan) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan);

  return gripper_success;
}

bool step_close_gripper(
    moveit::planning_interface::MoveGroupInterface &move_group_gripper,
    float value) {

  float RATIO_TOLERANCE = 1.2;

  bool result = true;

  for (int i = 3; i >= 0; i--) {
    result = result && close_gripper(move_group_gripper,
                                     std::pow(RATIO_TOLERANCE, i) * value);
    // Sleep for 100 milliseconds
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }
  return result;
}

bool set_gripper_state(
    moveit::planning_interface::MoveGroupInterface &move_group_gripper,
    moveit::planning_interface::MoveGroupInterface::Plan &my_plan_gripper,
    const char *gripper_state = "gripper_open") {
  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget(gripper_state);

  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  return success_gripper;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "arm_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

  // sim object length (square)
  float SIM_OBJ_LENGTH = 0.0205;

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -2.50; // Shoulder Lift
  joint_group_positions_arm[2] = 1.50;  // Elbow
  joint_group_positions_arm[3] = -1.50; // Wrist 1
  joint_group_positions_arm[4] = -1.55; // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.3279;
  target_pose1.position.y = -0.01078;
  target_pose1.position.z = 0.264;

  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // ########### END OF PREGRASP ############

  // Open Gripper
  set_gripper_state(move_group_gripper, my_plan_gripper, "gripper_open");

  // DEFININING APPROACHING AMOUNT
  float APPROACH_DIST = 0.045;

  // Approach
  translate_ee(move_group_arm, 2,
               [APPROACH_DIST](geometry_msgs::msg::Pose &pose) {
                 pose.position.z -= APPROACH_DIST; // reach down
               });

  //   Close Gripper
  //   close_gripper(move_group_gripper, SIM_OBJ_LENGTH);
  //   set_gripper_state(move_group_gripper, "gripper_close");
  step_close_gripper(move_group_gripper, SIM_OBJ_LENGTH);

  // Retreat
  translate_ee(move_group_arm, 2,
               [APPROACH_DIST](geometry_msgs::msg::Pose &pose) {
                 pose.position.z += APPROACH_DIST; // lift up
               });

  float CLEARANCE = 0.03;

  // CLEARANCE Retreat
  translate_ee(move_group_arm, 2, [CLEARANCE](geometry_msgs::msg::Pose &pose) {
    pose.position.z += CLEARANCE; // lift up
  });

  // Panning 180ish degrees
  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] += M_PI; // Shoulder Pan

  std::cout << "joint_group_positions_arm[0]: " << joint_group_positions_arm[0]
            << std::endl;

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Approach CLEARANCE
  translate_ee(move_group_arm, 2, [CLEARANCE](geometry_msgs::msg::Pose &pose) {
    pose.position.z -= CLEARANCE; // reach down
  });

  // Approach
  translate_ee(move_group_arm, 2,
               [APPROACH_DIST](geometry_msgs::msg::Pose &pose) {
                 pose.position.z -= APPROACH_DIST; // reach down
               });

  // Open Gripper
  set_gripper_state(move_group_gripper, my_plan_gripper, "gripper_open");

  // Retreat

  translate_ee(move_group_arm, 2,
               [APPROACH_DIST](geometry_msgs::msg::Pose &pose) {
                 pose.position.z += APPROACH_DIST; // lift
               });

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -2.50; // Shoulder Lift
  joint_group_positions_arm[2] = 1.50;  // Elbow
  joint_group_positions_arm[3] = -1.50; // Wrist 1
  joint_group_positions_arm[4] = -1.55; // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  rclcpp::shutdown();
  return 0;
}