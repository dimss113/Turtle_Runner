#include "turtlesim_cleaner/robot_cleaner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::MultiThreadedSpinner spn;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pose_subscriber = nh.subscribe("/turtle1/pose", 1000, cllbckPose);

    while (ros::ok())
    {
        if (instruction())
        {
            return true;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    spn.spin();
}

void cllbckPose(const turtlesim::PoseConstPtr &msg)
{
    pos_x = msg->x;
    pos_y = msg->y;
    pos_theta = msg->theta;
    ROS_INFO("pos turtle: %.2f || %.2f || %.2f", pos_x, pos_y, pos_theta);
}

void move(double speed, double distance, bool isForward)
{
    geometry_msgs::Twist vel_msg;

    if (isForward)
    {
        vel_msg.linear.x = abs(speed);
    }
    else
    {
        vel_msg.linear.x = -abs(speed);
    }
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    velocity_publisher.publish(vel_msg);

    ROS_INFO("turtle is moving");

    double initialTime = ros::Time::now().toSec();
    double travelTime = distance / speed;
    double finalTime = initialTime + travelTime;

    while (ros::Time::now().toSec() < finalTime)
    {
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
    }

    vel_msg.linear.x = 0;

    velocity_publisher.publish(vel_msg);

    ROS_INFO("Turtle has reached its destinations");
}

void rotate(double angular_speed, double relative_angle, bool clockwise)
{
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    velocity_publisher.publish(vel_msg);

    ROS_INFO("robot is moving");

    if (!clockwise)
    {
        vel_msg.angular.z = abs(angular_speed);
    }
    else
    {
        vel_msg.angular.z = -abs(angular_speed);
    }

    double current_angle = 0.0;

    double initialTime = ros::Time::now().toSec();

    while (current_angle < relative_angle)
    {
        velocity_publisher.publish(vel_msg);
        double finalTime = ros::Time::now().toSec();
        current_angle = angular_speed * (finalTime - initialTime);
        ros::spinOnce();
    }

    vel_msg.angular.z = 0;

    velocity_publisher.publish(vel_msg);

    ROS_INFO("robot has reached goal");
}

bool instruction()
{
    int command;
    double distance, speed;
    bool isForward;
    int turn;

    double angular_speed, relative_angle;
    double point_x, point_y;
    bool clockwise;
    cout << "choose 1 for go straight, choose 2 for rotate,  choose for motion to point: ";
    cin >> command;

    if (command == 1)
    {
        cout << "enter the speed: ";
        cin >> speed;
        cout << "enter the distance: ";
        cin >> distance;
        cout << "forward? or backward? enter 1 for forward and enter 0 for backward: ";
        cin >> turn;
        if (turn == 1)
        {
            isForward = true;
        }
        else if (turn == 0)
        {
            isForward = false;
        }

        move(speed, distance, isForward);
    }

    else if (command == 2)
    {
        cout << "enter angular_speed: ";
        cin >> angular_speed;
        cout << "enter relative_angle: ";
        cin >> relative_angle;
        cout << "clockwise or reverseClockwise? enter 1 for reverse clockwise or 0 for clockwise: ";
        cin >> turn;
        if (turn == 1)
        {
            clockwise = false;
        }
        else
        {
            clockwise = true;
        }

        relative_angle = (relative_angle * M_PI) / 180;
        rotate(angular_speed, relative_angle, clockwise);
    }
    return true;
}
