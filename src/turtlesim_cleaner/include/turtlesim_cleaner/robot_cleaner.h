#ifndef TURTLE_H_
#define TURTLE_H_

#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <algorithm>
#include <math.h>
#include <cmath>
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
// ros::NodeHandle nh;

const double x_min = 0;
const double y_min = 0;
const double x_max = 11;
const double y_max = 11;

double pos_x, pos_y, pos_theta;

const double PI = M_PI;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool clockwise);

bool instruction();

void cllbckPose(const turtlesim::PoseConstPtr &msg);

#endif