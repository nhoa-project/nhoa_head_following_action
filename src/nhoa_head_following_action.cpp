#include <nhoa_head_following_action/nhoa_head_following_action.h>

/*
Status can take this values:
uint8 PENDING         = 0   # The goal has yet to be processed by the action
server
uint8 ACTIVE          = 1   # The goal is currently being processed by
the action server
uint8 PREEMPTED       = 2   # The goal received a cancel
request after it started executing #   and has since completed its execution
(Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved
successfully by the action server (Terminal State)
uint8 ABORTED         = 4   #
The goal was aborted during execution by the action server due #    to some
failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by
the action server without being processed, #    because the goal was
unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal
received a cancel request after it started executing #    and has not yet
completed execution
uint8 RECALLING       = 7   # The goal received a cancel
request before it started executing, #    but the action server has not yet
confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal
received a cancel request before it started executing #    and was successfully
cancelled (Terminal State)
uint8 LOST            = 9   # An action client can
determine that a goal is LOST. This should not be #    sent over the wire by an
action server
*/

// namespace macroactions {

Nhoa_head_following_action::Nhoa_head_following_action(tf2_ros::Buffer *tf) {
  tf_ = tf;
  tflistener_ = new tf2_ros::TransformListener(*tf_);

  ros::NodeHandle n("~");

  // Dynamic reconfigure
  // dsrv_ = new
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>(n);
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>::CallbackType
  // cb = boost::bind(&Upo_navigation_macro_actions::reconfigureCB, this, _1,
  // _2); dsrv_->setCallback(cb);

  n.param<double>("control_frequency", control_frequency_, 1.0);
  n.param<bool>("use_look_around", use_look_around_, true);
  n.param<std::string>("robot_head_frame", robot_head_frame_,
                       "head_front_camera_color_optical_frame");

  person_detected_ = false;

  ros::NodeHandle nh;
  std::string hri_id_topic = "";
  n.param<std::string>("hri_ids_topic", hri_id_topic, "humans/bodies/tracked");
  hri_ids_sub_ = nh.subscribe<hri_msgs::IdsList>(
      hri_id_topic.c_str(), 1, &Nhoa_head_following_action::idListCallback,
      this);

  // Dynamic reconfigure
  // dsrv_ = new
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>(n);
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>::CallbackType
  // cb = boost::bind(&Upo_navigation_macro_actions::reconfigureCB, this, _1,
  // _2); dsrv_->setCallback(cb);

  if (use_look_around_) {
    playMotionClient_.reset(new PlayMotionClient("play_motion"));
    while (!playMotionClient_->waitForServer(ros::Duration(0.5))) {
      ROS_WARN(
          "\n\n\nWaiting for connection to PlayMotion action server\n\n\n");
    }
    ROS_INFO("\n\n\nPlayMotionAction client connected!\n\n\n");
  }

  // pointHeadClient_ = std::make_shared<PointHeadClient>(
  //    "head_controller/point_head_action", false);
  pointHeadClient_.reset(
      new PointHeadClient("/head_controller/point_head_action"));
  while (!pointHeadClient_->waitForServer(ros::Duration(0.5))) {
    ROS_WARN("\n\n\nWaiting for connection to PointHead action server\n\n\n");
  }
  ROS_INFO("\n\n\nPointHeadAction client connected!\n\n\n");
  // ros::Duration(10.0).sleep();

  // Initialize action server
  ROS_INFO("Initializing HEAD_FOLLOWING action server...");
  HeadActionServer_ = std::make_shared<headFollowingActionServer>(
      nh_, "HeadFollowing",
      boost::bind(&Nhoa_head_following_action::headFollowingCallback, this, _1),
      false); // boost::bind

  // start action server
  HeadActionServer_->start();
  ROS_INFO("\n\n\n\nHEAD_FOLLOWING action server started!\n\n\n\n");
}

Nhoa_head_following_action::~Nhoa_head_following_action() {}

/*
void
Upo_navigation_macro_actions::reconfigureCB(upo_navigation_macro_actions::NavigationMacroActionsConfig
&config, uint32_t level){

    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    control_frequency_ = config.control_frequency;
    secs_to_check_block_ = config.secs_to_check_block;
  block_dist_ = config.block_dist;
  secs_to_wait_ = config.secs_to_wait;
  social_approaching_type_ = config.social_approaching_type;
  secs_to_yield_ = config.secs_to_yield;
  //use_leds_ = config.use_leds;
  //leds_number_ = config.leds_number;

}*/

bool Nhoa_head_following_action::getPersonFromHRI(
    const std::string id, std::string &found_id,
    geometry_msgs::TransformStamped &p) {

  // Get the id list
  lmutex_.lock();
  std::vector<std::string> list = id_list_;
  lmutex_.unlock();
  std::string id_frame = "body_";
  found_id = "-1";
  // first, check the target_id
  if (id == "-1") {
    // we use the first person if exists
    if (!list.empty()) {
      id_frame = id_frame + list[0];
      found_id = id_frame;
    } else
      return false;
  } else {
    // we use directly the id to get the TF
    id_frame = id_frame + id;
    found_id = id_frame;
  }
  // Now, we look for the frame in robot base frame
  return getPerson(id_frame, p);
}

bool Nhoa_head_following_action::getPerson(const std::string frame_id,
                                           geometry_msgs::TransformStamped &p) {

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped =
        tf_->lookupTransform(robot_head_frame_, frame_id, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("HeadFollowing. Error getPerson: %s", ex.what());
    // ros::Duration(1.0).sleep();
    // continue;
    return false;
  }
  p = transformStamped;
  return true;
}

control_msgs::PointHeadGoal Nhoa_head_following_action::computeLookAtPointGoal(
    const geometry_msgs::TransformStamped &p) {

  geometry_msgs::PointStamped pointStamped;
  pointStamped.header.frame_id = robot_head_frame_;
  pointStamped.header.stamp = ros::Time::now();
  pointStamped.point.x = p.transform.translation.x;
  pointStamped.point.y = p.transform.translation.y;
  pointStamped.point.z = 1.0; // p.transform.translation.z + 1.0; //

  // build the action goal
  control_msgs::PointHeadGoal goal;
  // control_msgs::PointHeadActionGoal goal;
  // the goal consists in making the Z axis of the cameraFrame to point towards
  // the pointStamped
  goal.pointing_frame = robot_head_frame_;
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.25;
  goal.target = pointStamped;
  return goal;
}

void Nhoa_head_following_action::headFollowingCallback(
    const nhoa_head_following_action::HeadFollowingGoal::ConstPtr &goal) {
  printf("¡¡¡¡¡¡¡Action HEAD FOLLOWING started!!!!!!\n");

  std::string id = goal->target_id;
  geometry_msgs::TransformStamped tfperson;
  std::string found_frame;
  person_detected_ = getPersonFromHRI(goal->target_id, found_frame, tfperson);
  headFeedback_.found = person_detected_;
  headFeedback_.person_id = found_frame;

  // std::string stop_head_manager =
  //     "rosservice call /pal_startup_control/stop + \" app : 'head_manager'
  //     \"";
  // std::string start_head_manager = "rosservice call
  // /pal_startup_control/start "
  //                                  "\" app : 'head_manager' args: ''\"";
  if (person_detected_) {
    // system(stop_head_manager.c_str());
    if (use_look_around_) {
      playMotionClient_->cancelAllGoals();
    }
    control_msgs::PointHeadGoal head_goal = computeLookAtPointGoal(tfperson);
    pointHeadClient_->sendGoal(head_goal);
  } else if (use_look_around_) {
    // system(start_head_manager.c_str());
    play_motion_msgs::PlayMotionGoal motion_goal;
    motion_goal.motion_name = "look_around";
    playMotionClient_->sendGoal(motion_goal);
  }

  ros::Rate r(control_frequency_);
  bool exit = false;
  ros::Time time_init = ros::Time::now();
  bool first = true;
  // nav_msgs::Odometry pose_init;
  // ros::WallTime startt;
  while (nh_.ok()) {
    // startt = ros::WallTime::now();

    if (HeadActionServer_->isPreemptRequested()) {
      if (HeadActionServer_->isNewGoalAvailable()) {
        nhoa_head_following_action::HeadFollowingGoal new_goal =
            *HeadActionServer_->acceptNewGoal();
        id = new_goal.target_id;
      } else {
        // Cancel?????
        // notify the ActionServer that we've successfully preempted
        headResult_.result = "Preempted";
        headResult_.value = 2;
        ROS_DEBUG_NAMED("nhoa_head_following_action",
                        "preempting the current goal");
        HeadActionServer_->setPreempted(headResult_,
                                        "Head Following preempted");
        // we'll actually return from execute after preempting
        return;
      }
    }

    person_detected_ = getPersonFromHRI(id, found_frame, tfperson);
    headFeedback_.found = person_detected_;
    headFeedback_.person_id = found_frame;
    if (person_detected_) {
      if (use_look_around_) {
        // system(stop_head_manager.c_str());
        playMotionClient_->cancelAllGoals();
      }
      control_msgs::PointHeadGoal head_goal = computeLookAtPointGoal(tfperson);
      pointHeadClient_->sendGoal(head_goal);
    } else if (use_look_around_) {
      // system(start_head_manager.c_str());
      play_motion_msgs::PlayMotionGoal motion_goal;
      motion_goal.motion_name = "look_around";
      playMotionClient_->sendGoal(motion_goal);
      use_look_around_ = false;
    }

    // Posible states:
    // PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    actionlib::SimpleClientGoalState state = pointHeadClient_->getState();

    HeadActionServer_->publishFeedback(headFeedback_);

    // ros::WallDuration dur = ros::WallTime::now() - startt;
    // printf("Loop time: %.4f secs\n", dur.toSec());

    r.sleep();
  }

  ROS_INFO("HeadFollowing - ABORTED state");
  headResult_.result = "Aborted. System is shuting down";
  headResult_.value = 3;
  HeadActionServer_->setAborted(
      headResult_, "HeadFollowing aborted because the node has been killed");
}

/*
bool Indires_macro_actions::reconfigureParameters(std::string node, std::string
param_name, std::string value, const datatype type)
{
  //printf("RECONFIGURE PARAMETERS METHOD\n");
  dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter param1;
    dynamic_reconfigure::BoolParameter param2;
    dynamic_reconfigure::DoubleParameter param3;
    dynamic_reconfigure::StrParameter param4;
    dynamic_reconfigure::Config conf;

    switch(type)
    {
    case INT_TYPE:
      param1.name = param_name.c_str();
      param1.value = stoi(value);
      conf.ints.push_back(param1);
      break;

    case DOUBLE_TYPE:
      param3.name = param_name.c_str();
      //printf("type double. Value: %s\n", param3.name.c_str());
      param3.value = stod(value);
      //printf("conversion to double: %.3f\n", param3.value);
      conf.doubles.push_back(param3);
      break;

    case BOOL_TYPE:
      param2.name = param_name.c_str();
      param2.value = stoi(value);
      conf.bools.push_back(param2);
      break;

    case STRING_TYPE:
      param4.name = param_name.c_str();
      param4.value = value;
      conf.strs.push_back(param4);
      break;

    default:
      ROS_ERROR("indires_macro_actions. ReconfigureParameters. datatype not
valid!");
  }
    srv_req.config = conf;

    std::string service = node + "/set_parameters";
    if (!ros::service::call(service, srv_req, srv_resp)) {
      ROS_ERROR("Could not call the service %s reconfigure the param %s to %s",
service.c_str(), param_name.c_str(), value.c_str());
      return false;
    }
    return true;
}
*/

// void Nhoa_approach_action::peopleCallback(
//     const people_msgs::People::ConstPtr &msg) {
//   pmutex_.lock();
//   people_ = *msg;
//   pmutex_.unlock();
// }

void Nhoa_head_following_action::idListCallback(
    const hri_msgs::IdsList::ConstPtr &msg) {
  lmutex_.lock();
  id_list_ = msg->ids;
  lmutex_.unlock();
}

geometry_msgs::PoseStamped Nhoa_head_following_action::transformPoseTo(
    const geometry_msgs::PoseStamped &pose_in, std::string frame_out) {
  geometry_msgs::PoseStamped in = pose_in;
  geometry_msgs::PoseStamped pose_out = pose_in;
  pose_out.header.stamp = ros::Time();
  in.header.stamp = ros::Time();
  try {
    tf_->transform(in, pose_out, frame_out);
  } catch (tf2::TransformException ex) {
    ROS_WARN("\n\nApproachAction. TransformException in method "
             "transformPoseTo: %s\n\n",
             ex.what());
  }
  return pose_out;
}

geometry_msgs::PointStamped Nhoa_head_following_action::transformPointTo(
    const geometry_msgs::PointStamped &point_in, std::string frame_out) {
  geometry_msgs::PointStamped in = point_in;
  in.header.stamp = ros::Time();
  geometry_msgs::PointStamped point_out;
  try {
    tf_->transform(in, point_out, frame_out.c_str());
  } catch (tf2::TransformException ex) {
    ROS_WARN("\n\nApproachAction. TransformException in method "
             "transformPointTo: %s\n\n",
             ex.what());
    point_out.header = in.header;
    point_out.header.stamp = ros::Time::now();
    point_out.point.x = 0.0;
    point_out.point.y = 0.0;
    point_out.point.z = 0.0;
  }

  return point_out;
}

// This method removes the initial slash from the frame names
// in order to compare the string names easily
void Nhoa_head_following_action::fixFrame(std::string &cad) {
  if (cad[0] == '/') {
    cad.erase(0, 1);
  }
}

float Nhoa_head_following_action::normalizeAngle(float val, float min,
                                                 float max) {
  float norm = 0.0;
  if (val >= min)
    norm = min + fmod((val - min), (max - min));
  else
    norm = max - fmod((min - val), (max - min));

  return norm;
}
