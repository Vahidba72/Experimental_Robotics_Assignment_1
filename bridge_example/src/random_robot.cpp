#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <iostream>

ros::Publisher pub;

int count=100;


double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
   {
	if (count==100){
		count=0;
	geometry_msgs::Twist vel;
        vel.linear.x = randMToN(-0.3, 0.3);
        vel.angular.z= randMToN(-0.2, 0.2);
        pub.publish(vel);
     }
     else{
		 count++;
	 }
 }



int main(int argc, char **argv)
{


 ros::init(argc, argv, "controller");
 ros::NodeHandle n;
 pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
 ros::Subscriber sub = n.subscribe("odom", 1, odomCallback);
 ros::Rate loop_rate(10);
 ros::spin();
 return 0;
}
