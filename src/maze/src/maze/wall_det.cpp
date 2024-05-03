#include <maze/wall_det.hpp>

using namespace std;

/**
 *@brief Wall Detection
 **/

WallDetect::WallDetect():_spinner(4)
{
  _spinner.start(); //async spinner

  _laser_sub = _nh.subscribe("/scan", 1, & WallDetect::laserCb, this);
  _odom_sub = _nh.subscribe("/odom", 1, & WallDetect::odomCb, this);

  _vel_pub = _nh.advertise < geometry_msgs::Twist > ("/cmd_vel", 5);

  _wall_det_service = _nh.advertiseService("mazesolver/wall_detection", & WallDetect::serviceCb, this);

  initParams();

}

WallDetect::~WallDetect()
{
	_spinner.stop();
}

void WallDetect::initParams()
{
	_flag = false;                   //To check if a wall has been detected
	_minLaserValues = 60;            //Number of laser points that must be next to each other
	_intervalEpsilon = 1.5;          //Upper and Lower bounds of the laser
	_distanceToWall = 0.5;           //Distance between wall and bot
	_angle = 1.57;                   //90 Degrees in radian
}

void WallDetect::laserCb(const sensor_msgs::LaserScan::ConstPtr &laser)
{
	WriteLock lock(_laser_mutex);
	_laser_ptr = laser;
}

void WallDetect::odomCb(const nav_msgs::Odometry::ConstPtr &odom)
{
	WriteLock lock(_odom_mutex);
	_odom_ptr = odom;
}

//Service for wall detection
bool WallDetect::serviceCb(maze::det_srv::Request  &req,  maze::det_srv::Response &res)
{
	bool flag;
	if(req.detection_flag)
	{
		flag = wallDetection(req.actMinLaserValue, req.turnSpeed);
		res.success = flag;
	}
}

//Service callback for wall detection
bool WallDetect::wallDetection(float actMinLaserValue, float turnSpeed)
{
	_det_mutex.lock();
	_flag = false;
	int j = 0;
	int end = 0;
	int getLaserIndex;
	float tmpAngle;
	auto ranges = _laser_ptr->ranges;
	int ranges_len = ranges.size();
	if(actMinLaserValue<_distanceToWall) //obstacle detected
	{
		rotate_angle(_angle, turnSpeed);
	}
	else
	{
		std::vector<float> arrVal1;
		std::vector<float> arrVal2;
		int i = 0;
		// Check if the laser values are within the interval epsilon
		while(i < ranges_len)
		{
			int valCounter = 0;
			for(int k = i+1; k < ranges_len; k++)
			{

				if((ranges[i]-_intervalEpsilon) <= ranges[k] <= (ranges[i]+_intervalEpsilon))
					valCounter += 1;
				else
				{
					//If the number of laser values are greater than the minimum laser values add those indexes to the vector
					if(valCounter>_minLaserValues)
					{
						arrVal1.push_back(i);
						arrVal2.push_back(k);
					}
					valCounter = 0;
					i = k;
				}
				if(k==ranges_len-1)
					i = ranges_len;
			}

		}

		//Computing the direction of possible wall location
		getLaserIndex = 0;
		if(arrVal1.empty()==false && arrVal2.empty()==false)
		{
			auto max_i = *max_element(arrVal1.begin(), arrVal1.end());
			auto max_k = *max_element(arrVal2.begin(), arrVal2.end());
			getLaserIndex = int(floor((float(max_i) + float(max_k)/2)));
		}
		if(getLaserIndex<=355 || getLaserIndex>=5)
		{
			tmpAngle = _laser_ptr->angle_min + (getLaserIndex * _laser_ptr->angle_increment);
			if(getLaserIndex<350)
			{
				rotate_angle(abs(tmpAngle),-0.3); //rotate right
			}
			else
			{
				rotate_angle(abs(tmpAngle),0.3); //rotate left
			}
		}
		_flag = true;
	}
	_det_mutex.unlock();
	return _flag;
}

void WallDetect::rotate_angle(float angle, float speed)
{
	_rot_mutex.lock();

	float angular_speed = speed;
	float desired_angle = angle;
	_vel.linear.x = 0.0;
	_vel.angular.z = angular_speed;

	float t0 = ros::Time::now().toSec();
	float current_angle = 0;

  //Rotate the bot
	while(current_angle < desired_angle)
	{
		_vel_pub.publish(_vel);
		float t1 = ros::Time::now().toSec();
		current_angle = abs(angular_speed)*(t1-t0);
	}

  //Bot stops after turning
	_vel.linear.x = 0.0;
	_vel.angular.z = 0;
	_vel_pub.publish(_vel);

	_rot_mutex.unlock();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wall_detection_node");
  WallDetect obj;
  ros::waitForShutdown();
  return 0;
}
