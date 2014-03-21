#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "endlineturnmp.h"

cv::Mat scannedData;
float factor = 40.0f; // 1mt = 40px
float imageSize = 800;
cv::Point2f center(400,400);
std::vector<cv::Vec4i> lines;

cv::Point2f target, targetDirection;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scannedData = cv::Mat::zeros(cv::Size(800,800), CV_8UC1);

    float angle = msg->angle_min;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        float range = msg->ranges[i];
        if (range >= msg->range_min && range <= msg->range_max)
        {
            cv::Point2f pt(range * std::cos(angle), -1 * range * std::sin(angle));
            scannedData.at<uchar>(pt*factor + center) = 255;
        }

        angle = angle + msg->angle_increment;
    }

    HoughLinesP( scannedData, lines, 1, CV_PI/180, 20, 20, 20 );

    cv::Mat linesColor;
    cv::cvtColor(scannedData, linesColor, CV_GRAY2BGR);

    for( size_t i = 0; i < lines.size(); i++ )
    {
      cv::Vec4i l = lines[i];
      cv::line( linesColor,
                cv::Point(l[0], l[1]),
                cv::Point(l[2], l[3]),
                cv::Scalar(0,0,255), 3, CV_AA);
    }

    cv::circle(linesColor, target*factor+center, 3, cv::Scalar(255,0,0),3);

    cv::imshow("lines", linesColor);
    cv::waitKey(33);
}

int main(int argc, char** argv)
{
    cv::namedWindow("lines");
    target.x = 0.0f;
    target.y = 0.0f;

    std::cout << std::fixed << std::setprecision(6);
    setlocale(LC_NUMERIC, "C");

    //////
    /// Open config file
    std::string
        settfilename = argv[2];

    cv::FileStorage
        fs;

    fs.open(settfilename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cerr << "Could not open settings file: " << settfilename << std::endl;
        exit(-2);
    }

    nav::EndLineTurnMP mp(fs);

    ros::init(argc, argv, "egocentric_mp");

    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/Pioneer3at/cmd_vel", 1);
    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/Pioneer3AT/laserscan", 1, scan_callback);

    tf::StampedTransform datamatrix_to_base_link;

    tf::TransformListener tfListener;

    geometry_msgs::Twist base_msg;

    float frequency = 33;
    ros::Rate loop_rate(10.0); // 100Hz

    //std::cout << "x;y;phi;r;sigma;theta;curvature;linear;angular;" << std::endl;

    float r = 1.0, theta = 1.57, sigma = 1.57;

    scannedData = cv::Mat::zeros(cv::Size(800,800), CV_8UC1);

    while (ros::ok())
    {
        //float v, omega;
        //v = mp.computeLinearVelocity(r,theta,sigma);
        //omega = mp.computeAngularVelocity(v, r, theta, sigma);

        //std::cout << v << " - " << omega << std::endl;

        //base_msg.linear.x = v;
        //base_msg.angular.z = omega;

        //cmd_vel_pub.publish(base_msg);

        ros::Time a = ros::Time::now();
        ros::Duration offset(500);

        a -= offset;

        ros::Duration timeout(1000);
        bool isDatamatrix = tfListener.waitForTransform("/camera_link",
                                                        "/datamatrix_frame",
                                                        a,
                                                        timeout);

        if (isDatamatrix)
        {

            ROS_INFO("Setting target");
            std::cout << "st" << std::endl;
            tfListener.lookupTransform("/camera_link",
                                       "/datamatrix_frame",
                                       a,
                                       datamatrix_to_base_link);


            tf::Point a,b;

            a.setX(0);
            a.setY(0);
            a.setZ(0);
            b = datamatrix_to_base_link(a);

            target.x = b.x();
            target.y = b.y();
        }
        else
        {
            std::cout << "bad" << std::endl;
            ROS_INFO("tf not available");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
