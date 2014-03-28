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
float factor = 80.0f; // 1mt = 40px
float imageSize = 800;
cv::Point2f center(400,400);
std::vector<cv::Vec4i> lines;

cv::Point2f target, targetDirection;

float pointLineDistance(const cv::Point2f &point, const cv::Vec4i line)
{
    cv::Point pt = point;

    // take 2 points on the line
    cv::Point2f begin(line.val[2],line.val[3]);
    cv::Point2f end(line.val[0],line.val[1]);

    // translate to the origin
    pt = point - begin;
    end = end - begin;

    double area = pt.cross(end);
    return area / cv::norm(end);
}

int nearestLineIndex(const cv::Point2f &point, const std::vector<cv::Vec4i> &linesVector)
{
    float minDistance = std::numeric_limits<float>::max();
    int nearestLineIndex = -1;

    int count = 0;
    for (cv::Vec4i line : linesVector)
    {
        // compute line-point distance
        float currentDistance = std::abs(pointLineDistance(point, line));

        std::cout << currentDistance << " - ";

        if (currentDistance < minDistance)
        {
            nearestLineIndex = count;
            minDistance = currentDistance;
        }

        count = count + 1;
    }

    std::cout << std::endl << minDistance << std::endl;

    return nearestLineIndex;
}

cv::Point2f getSnapPointToLine(const cv::Point2f &point, const cv::Vec4i line)
{
    float distance = pointLineDistance(point, line);

    cv::Point2f A(line[0],line[1]);

    cv::Vec2f A_pointVector(point - A);

    float A_pointDistance = cv::norm(A_pointVector);

    float A_snapDistance = std::sqrt(A_pointDistance*A_pointDistance - distance*distance);

    cv::Vec2f lineVector(line[2]-line[0], line[3]-line[1]);
    lineVector = A_snapDistance * cv::normalize(lineVector);

    return cv::Point2f(A.x + lineVector.val[0], A.y + lineVector.val[1]);
}

void morphClosure(const cv::Mat &src, cv::Mat &dst)
{
    int dilationType = cv::MORPH_RECT;
    int dilationSize = 3;
    cv::Mat element = getStructuringElement( dilationType,
                                         cv::Size( 2*dilationSize + 1, 2*dilationSize+1 ),
                                         cv::Point( dilationSize, dilationSize ) );
    cv::Mat tmp;
    cv::dilate(src, tmp, element);
    cv::erode(tmp, dst, element);
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // get laser points
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

    // perform a closure (dilation+erosion)
    cv::Mat closedScannedData;
    morphClosure(scannedData, closedScannedData);

    // extract lines from laser points
    HoughLinesP( closedScannedData, lines, 2, CV_PI/90, 80, 25, 50 );

    // get nearest line index
    int idx = nearestLineIndex(target*factor+center, lines);
    float distance = pointLineDistance(target*factor+center, lines[idx]);

    std::cout << idx << " - (" << target.x << "," << target.y << ")" << " - " << lines[idx] << " - " << distance << std::endl;

    // plot lines, points...
    cv::Mat linesColor;
    cv::cvtColor(scannedData, linesColor, CV_GRAY2BGR);

    cv::circle(linesColor, cv::Point2f(0,0)*factor+center, 3, cv::Scalar(200,200,200),3);
    for( size_t i = 0; i < lines.size(); i++ )
    {
      cv::Vec4i l = lines[i];
      cv::line( linesColor,
                cv::Point2f(l[0], l[1]),
                cv::Point2f(l[2], l[3]),
                i==idx?cv::Scalar(0,255,0):cv::Scalar(0,0,255),
                1, CV_AA );
      cv::circle(linesColor, cv::Point2f(l[0], l[1]), 2, cv::Scalar(200,200,100));
      cv::circle(linesColor, cv::Point2f(l[2], l[3]), 2, cv::Scalar(100,200,200));
    }

    /// Move the target 1mt in front of the line
    /*// get an ortogonal vector
    cv::Vec2f lineVector((lines[idx])[0] - (lines[idx])[2],
                         (lines[idx])[1] - (lines[idx])[3]);
    /// TODO: check 0 values...
    cv::Vec2f ortoLineVector(-lineVector.val[1], lineVector.val[0]);
    ortoLineVector = ortoLineVector / cv::norm(ortoLineVector);*/

    cv::circle(linesColor, target*factor+center, 3, cv::Scalar(250,50,250),3);

    // snap target to the nearest line
    target = getSnapPointToLine(target*factor+center, lines[idx]);

    cv::circle(linesColor, target, 3, cv::Scalar(255,0,0),3);

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
        ros::Duration offset(0.5);
        ros::Duration timeout(0.15);
        bool isDatamatrix = false;

        while (!isDatamatrix)
        {

            a = ros::Time::now() - offset;
            isDatamatrix = tfListener.waitForTransform("camera_link",
                                                       "datamatrix_frame",
                                                       a,
                                                       timeout);

            ROS_INFO("no transform yet");
        }

        if (isDatamatrix)
        {

            ROS_INFO("Setting target");

            a = ros::Time::now() - offset;
            tfListener.lookupTransform("camera_link",
                                       "datamatrix_frame",
                                       a,
                                       datamatrix_to_base_link);


            tf::Point a,b;

            a.setX(0);
            a.setY(0);
            a.setZ(0);
            b = datamatrix_to_base_link(a);

            target.x = b.x();
            target.y = -1 * b.y();
        }
        else
        {
            ROS_INFO("tf not available");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
