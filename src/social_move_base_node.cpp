#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <social_move_base/SocialMoveBaseAction.h>
#include <social_msgs/Locals.h>
#include <social_msgs/Local.h>

bool isName(const std::string& s, const social_msgs::Local& obj)
{ return obj.name == s; }

class SocialMoveBaseNode
{
private:

  ros::NodeHandle nh;
  actionlib::SimpleActionServer<social_move_base::SocialMoveBaseAction> as_social_navigation;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base;

  ros::Subscriber sub_locals;

  std::vector<social_msgs::Local> locals;

public:

  SocialMoveBaseNode() :
  as_social_navigation(nh, "social_navigation",
  boost::bind(&SocialMoveBaseNode::callback_action_social_navigation_server, this, _1), false),
  ac_move_base("move_base", true)
  {
    as_social_navigation.start();
    sub_locals = nh.subscribe("/locals", 1000, &SocialMoveBaseNode::callback_sub_locals, this);
    // locals = social_msgs::Locals();

    ROS_INFO("Waiting for action server to start.");
    ac_move_base.waitForServer(); //will wait for infinite time
    ROS_INFO("Social navigation ready!");

  }

  ~SocialMoveBaseNode()
  {
  }

  void callback_sub_locals(const social_msgs::Locals::ConstPtr& msg)
  {
    this->locals = msg->locals;
    // for (size_t i = 0; i < msg->locals.size(); i++) {
    //   ROS_INFO("I heard: [%s]", msg->locals[i].name.c_str());
    // }
  }

  void callback_action_social_navigation_server(const social_move_base::SocialMoveBaseGoalConstPtr &goal)
  {
    // ros::Rate r(1);
    // bool success = true;
    // social_move_base::SocialMoveBaseFeedback feedback_;
    social_move_base::SocialMoveBaseResult result_;

    std::string target_name = goal->target_name;
    ROS_INFO("target: %s",  target_name.c_str());

    std::vector<social_msgs::Local>::iterator it = find_if(locals.begin(), locals.end(),
      boost::bind(&isName, target_name, boost::placeholders::_1));


    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.frame_id = "map";
    mb_goal.target_pose.header.stamp = ros::Time::now();
    mb_goal.target_pose.pose = (*it).pose;
    ac_move_base.sendGoal(mb_goal,
      boost::bind(&SocialMoveBaseNode::callback_action_move_base_client_done, this, _1));

    ac_move_base.waitForResult();

    // TODO: inplement others states
    if(ac_move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      as_social_navigation.setSucceeded(result_);
      ROS_INFO("SUCCEEDED");
    }
    else
    {
      as_social_navigation.setAborted(result_);
      ROS_INFO("ABORTED");
    }
  }

  void callback_action_move_base_client_done(const actionlib::SimpleClientGoalState& state)
  {
    // ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
    //     if (ac_move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     {
    //          // do something as goal was reached
    //     }
    //     if (ac_move_base.getState() == actionlib::SimpleClientGoalState::ABORTED)
    //     {
    //         // do something as goal was canceled
    //     }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "social_move_base_node");

  SocialMoveBaseNode smbn;
  ros::spin();

  return 0;
}
