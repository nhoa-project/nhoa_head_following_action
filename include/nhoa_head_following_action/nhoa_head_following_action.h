#ifndef HEAD_FOLLOWING_ACTION_H_
#define HEAD_FOLLOWING_ACTION_H_

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <angles/angles.h>
#include <cmath>
#include <control_msgs/PointHeadAction.h>
#include <iostream>
#include <play_motion_msgs/PlayMotionAction.h>
//#include <control_msgs/PointHeadActionGoal.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nhoa_head_following_action/HeadFollowingAction.h>
//#include <people_msgs/People.h>
#include <hri_msgs/IdsList.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
//#include <indires_macro_actions/NavigateHomeAction.h>
//#include <indires_macro_actions/ExplorationAction.h>
//#include <indires_macro_actions/TeleoperationAction.h>

// Probably it is not required
//#include <adapted_move_base/move_base.h>

//#include <upo_navigation_macro_actions/Yield.h>

#include <mutex> //Mutex

// Dynamic reconfigure
/*#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <indires_macro_actions/NavigationMacroActionsConfig.h>
*/

// namespace macroactions {

class Nhoa_head_following_action {
public:
  // enum datatype{INT_TYPE=1, DOUBLE_TYPE=2, BOOL_TYPE=3, STRING_TYPE=4,
  // GROUP_TYPE=5};

  Nhoa_head_following_action(tf2_ros::Buffer *tf);
  ~Nhoa_head_following_action();

  void headFollowingCallback(
      const nhoa_head_following_action::HeadFollowingGoal::ConstPtr &goal);
  // void peopleCallback(const people_msgs::People::ConstPtr &msg);
  void idListCallback(const hri_msgs::IdsList::ConstPtr &msg);

  // void changeParametersNarrowPlaces();
  // void changeParametersNarrowPlaces2();
  // bool reconfigureParameters(std::string node, std::string param_name,
  // std::string value, const datatype type);

private:
  // Dynamic reconfigure
  // boost::recursive_mutex configuration_mutex_;
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>
  // *dsrv_;
  // void
  // reconfigureCB(upo_navigation_macro_actions::NavigationMacroActionsConfig
  // &config, uint32_t level);

  // bool getPersonFromHRI(const std::string id,
  //                      geometry_msgs::TransformStamped &p);
  bool getPersonFromHRI(const std::string id, std::string &found_id,
                        geometry_msgs::TransformStamped &p);
  bool getPerson(const std::string frame_id,
                 geometry_msgs::TransformStamped &p);

  control_msgs::PointHeadGoal
  computeLookAtPointGoal(const geometry_msgs::TransformStamped &p);

  void fixFrame(std::string &cad);
  float normalizeAngle(float val, float min, float max);
  geometry_msgs::PoseStamped
  transformPoseTo(const geometry_msgs::PoseStamped &pose_in,
                  std::string frame_out);
  geometry_msgs::PointStamped
  transformPointTo(const geometry_msgs::PointStamped &point_in,
                   std::string frame_out);

  // CLIENT FOR THE MOVE_BASE SERVER
  // typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  //    moveBaseClient;
  // std::shared_ptr<moveBaseClient> moveBaseClient_;

  // Our Action interface type for moving ARI's head, provided as a typedef for
  // convenience
  // typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction>
  //    PointHeadClient;
  // std::shared_ptr<PointHeadClient> pointHeadClient_;
  typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction>
      PointHeadClient;
  typedef std::shared_ptr<PointHeadClient> PointHeadClientPtr;
  PointHeadClientPtr pointHeadClient_;

  typedef actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction>
      PlayMotionClient;
  typedef std::shared_ptr<PlayMotionClient> PlayMotionClientPtr;
  PlayMotionClientPtr playMotionClient_;

  tf2_ros::Buffer *tf_;
  tf2_ros::TransformListener *tflistener_;

  ros::NodeHandle nh_;

  typedef actionlib::SimpleActionServer<
      nhoa_head_following_action::HeadFollowingAction>
      headFollowingActionServer;
  std::shared_ptr<headFollowingActionServer> HeadActionServer_;

  nhoa_head_following_action::HeadFollowingFeedback headFeedback_;
  nhoa_head_following_action::HeadFollowingResult headResult_;

  //
  double control_frequency_;
  //
  bool person_detected_;
  //
  bool use_look_around_;
  std::string robot_head_frame_; // head_front_camera_link

  // bool test_;
  ros::Subscriber hri_ids_sub_;
  std::vector<std::string> id_list_;
  std::mutex lmutex_;
};
//};
#endif
