#include "get_cube_pose/get_pose_client.hpp"

FindObjectsClient::FindObjectsClient(const rclcpp::NodeOptions &options)
    : Node("get_pose_client_node", options) {

  // create client
  this->client_ptr_ = rclcpp_action::create_client<Find>(this, "find_objects");

  // create timer
  this->timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&FindObjectsClient::send_goal, this));
}

bool FindObjectsClient::is_goal_done() { return this->goal_done_; }
bool FindObjectsClient::was_goal_successful() { return this->goal_successful; }

float *FindObjectsClient::get_last_pos() { return this->last_pos; }

void FindObjectsClient::send_goal() {
  using namespace std::placeholders;

  this->timer_->cancel();

  this->goal_done_ = false;

  if (!this->client_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    return;
  }

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    this->goal_done_ = true;
    return;
  }

  auto goal_msg = Find::Goal();
  goal_msg.plan_grasps = false;

  auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&FindObjectsClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&FindObjectsClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&FindObjectsClient::result_callback, this, _1);

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

// ... other member function definitions ...

void FindObjectsClient::goal_response_callback(
    const GoalHandleFind::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void FindObjectsClient::feedback_callback(
    GoalHandleFind::SharedPtr,
    const std::shared_ptr<const Find::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
  (void)feedback;

  // tried printing feedback objs; However it seems that they are void
  //  RCLCPP_INFO(this->get_logger(), "Object Name %s",
  //  feedback->object.name); RCLCPP_INFO(this->get_logger(), "Support
  //  surface: %s",
  //              feedback->object.support_surface);
  //  for (const auto &pose : feedback->object.primitive_poses) {
  //    RCLCPP_INFO(this->get_logger(), "Position: x=%f, y=%f, z=%f",
  //                pose.position.x, pose.position.y, pose.position.z);
  //  }
}

void FindObjectsClient::result_callback(
    const GoalHandleFind::WrappedResult &result) {
  this->goal_done_ = true;
  // check if last request was successful
  this->goal_successful = false;
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    this->goal_successful = true;
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }

  // record result
  this->last_pos[0] =
      result.result->objects[0].object.primitive_poses[0].position.x;
  this->last_pos[1] =
      result.result->objects[0].object.primitive_poses[0].position.y;

  RCLCPP_INFO(this->get_logger(), "Result received");
  RCLCPP_INFO(this->get_logger(), "X: %f", this->last_pos[0]);
  RCLCPP_INFO(this->get_logger(), "Y: %f", this->last_pos[1]);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FindObjectsClient>());
  rclcpp::shutdown();
  return 0;
}