#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <maze/mazeSolverAction.h>
#include <maze/m_goal.h>

/**
 *@brief Action Server using Goal Callback Method
 **/ 

class act_server
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<maze::mazeSolverAction> as_;
	maze::mazeSolverFeedback feedback_;
	maze::mazeSolverResult result_;
	std::string action_name_;
	int res_,goal_;
	ros::Subscriber sub_;

public:
	act_server(std::string name):as_(nh_,name,false),action_name_(name)
	{
		as_.registerGoalCallback(boost::bind(&act_server::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&act_server::preemptCB, this));

		sub_ = nh_.subscribe("/mazesolver/goal",1,&act_server::callback,this);
		as_.start();
	}

	~act_server(void)
	{
	}
  
   //Goal is updated
	void goalCB()
	{
		goal_ = as_.acceptNewGoal()->goal;
	}

   //Action preempted
	void preemptCB()
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		as_.setPreempted();
	}

   //Checks if the process is complete; Publishes the feedback until the goal is reached
	void callback(const maze::m_goal::ConstPtr &msg)
	{
		if(!as_.isActive())
			return;

		res_ = msg->res;
		feedback_.feedback = "In Progress";
		as_.publishFeedback(feedback_);
		if(res_ == goal_)
		{
			result_.success = "Goal Reached";
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			as_.setSucceeded(result_);

		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc,argv, "action_server_node");
	act_server action_server_node("action_server_node");
	ros::spin();
	ros::waitForShutdown();
	return 0;
}