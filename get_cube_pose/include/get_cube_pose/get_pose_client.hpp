#ifndef GET_POSE_CLIENT_HPP
#define GET_POSE_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <grasping_msgs/action/find_graspable_objects.hpp>
#include <memory>

class FindObjectsClient : public rclcpp::Node {
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit FindObjectsClient(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void send_goal();

  bool is_goal_done();
  bool was_goal_successful();

  float *get_last_pos();

private:
  rclcpp_action::Client<Find>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  bool goal_successful;
  float last_pos[2];

  void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle);
  void feedback_callback(GoalHandleFind::SharedPtr,
                         const std::shared_ptr<const Find::Feedback> feedback);
  void result_callback(const GoalHandleFind::WrappedResult &result);
};

#endif // GET_POSE_CLIENT_HPP