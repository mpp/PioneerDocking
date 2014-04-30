#include <iostream>
#include <cmath>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "endlineturnmp.h"

cv::Mat scannedData;
float factor = 160.0f; // 1mt = 40px
float imageSize = 800;
cv::Point2f center(400,400);
std::vector<cv::Vec4f> lines;
nav::EndLineTurnMP *mp;

cv::Point2f target, targetDirection, datamatrixPoint;
std::mutex targetMutex;

// Kalman filter for target tracking
cv::KalmanFilter kfTarget;
cv::Mat_<float> kfState;
cv::Mat_<float> kfMeasurement;
cv::Mat kfProcessNoise;

bool screen = true;

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

std::vector<cv::Vec4f> extractLines(const cv::Mat &scannedData)
{
    std::vector<cv::Vec4i> lines_i;
    std::vector<cv::Vec4f> lines_f;
    // Extract lines with the Hough transform
    cv::HoughLinesP( scannedData, lines_i, 4, CV_PI/90, 100, 25, 50 );

    // Move lines from the "image" frame to the robot frame
    for (cv::Vec4f line : lines_i)
    {
        cv::Vec4f new_line_f;
        // Move the first point
        new_line_f.val[0] = (line.val[0] - center.x) / factor;
        new_line_f.val[1] = (line.val[1] - center.y) / factor;
        // Move the first point
        new_line_f.val[2] = (line.val[2] - center.x) / factor;
        new_line_f.val[3] = (line.val[3] - center.y) / factor;

        lines_f.push_back(new_line_f);
    }

    return lines_f;
}

int nearestLineIndex(const cv::Point2f &point, std::vector<cv::Vec4f> &linesVector)
{
    float minDistance = std::numeric_limits<float>::max();
    int nearestLineIndex = -1;

    int count = 0;
    for (cv::Vec4f line : linesVector)
    {
        // compute line-point distance
        float currentDistance = std::abs(pointLineDistance(point, line));

        //std::cout << currentDistance << " - ";

        if (currentDistance < minDistance)
        {
            nearestLineIndex = count;
            minDistance = currentDistance;
        }

        count = count + 1;
    }

    //std::cout << std::endl << minDistance << std::endl;

    // expand the nearest line
    cv::Vec4f *line = &(linesVector[nearestLineIndex]);
    cv::Vec2f lineVector((*line)[2]-(*line)[0], (*line)[3]-(*line)[1]);

    lineVector = cv::normalize(lineVector);

    cv::Vec2f
            a((*line)[0],(*line)[1]),
            b;
    b = a + 4.0f * lineVector;
    a = a - 4.0f * lineVector;

    (*line)[0] = a[0];
    (*line)[1] = a[1];
    (*line)[2] = b[0];
    (*line)[3] = b[1];

    return nearestLineIndex;
}

cv::Point2f getSnapPointToLine(const cv::Point2f &point, const cv::Vec4f line)
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

float normalizeAngle_PI(float angle)
{
    angle = angle > M_PI ? angle - 2 * M_PI : angle;
    angle = angle <= -M_PI ? angle + 2 * M_PI : angle;

    return angle;
}

void computeRThetaSigma(float &r, float &theta, float &sigma,
                        const cv::Point2f &robotPosition = cv::Point2f(0.0f,0.0f),
                        const float &robotBearing = 0.0f )
{
    float targetDirectionAngle;
    float targetAngle;
    targetMutex.lock();
    cv::Point2f currentTarget = target;
    cv::Point2f currentTargetDirection = targetDirection;
    targetMutex.unlock();

    targetDirectionAngle = std::atan2(-1 * (currentTargetDirection.y - robotPosition.y), (currentTargetDirection.x - robotPosition.x));
    targetAngle = std::atan2(-1 * (currentTarget.y - robotPosition.y), (currentTarget.x - robotPosition.x));

    targetDirectionAngle = normalizeAngle_PI(targetDirectionAngle - robotBearing);
    targetAngle = normalizeAngle_PI(targetAngle);

    r = cv::norm(currentTarget - robotPosition);
    theta = targetDirectionAngle - targetAngle;
    sigma = 0 - targetAngle;

    theta = normalizeAngle_PI(theta);

    //std::cout << "punti: " << targetDirection << " " << target << std::endl;
    //std::cout << "angoli: " << targetDirectionAngle << " " << targetAngle << std::endl;
    //std::cout << "r-t-s: " << r << " " << theta << " " << sigma << std::endl;

}

void drawPrevPath(cv::Mat &img)
{
    float dt = 0.01f;
    float currentTime = 0.0f;
    float maxTime = 200;
    cv::Point2f currentPosition(0.0f, 0.0f);
    float currentBearing = 0.0f;

    while (currentTime < maxTime)
    {
        currentTime = currentTime + dt;
        float r,theta,sigma;

        computeRThetaSigma(r, theta, sigma, currentPosition, currentBearing);

        float
            v = 0.0f,
            omega = 0.0f;
        v = mp->computeLinearVelocity(r,theta,sigma);
        omega = mp->computeAngularVelocity(v, r, theta, sigma);

        currentPosition.x = currentPosition.x + std::cos(currentBearing) * v * dt;
        currentPosition.y = currentPosition.y + std::sin(currentBearing) * v * dt;
        currentBearing = currentBearing - omega * dt;

        cv::Point2f pt = currentPosition * factor + center;
        if (pt.x >= 0 && pt.x < img.cols && pt.y >= 0 && pt.y <= img.rows)
        {
            img.at<cv::Vec3b>(pt)[0] = 200;
            img.at<cv::Vec3b>(pt)[1] = 200;
            img.at<cv::Vec3b>(pt)[2] = 200;
        }
        else
        {
            return;
        }

        if (v == 0.0f && omega == 0.0f || r <= mp->getEndEpsilon())
        {
            return;
        }
    }
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
            pt = pt * factor + center;

            if (pt.x >= 0 && pt.x <= scannedData.cols && pt.y >= 0 && pt.y <= scannedData.rows)
                scannedData.at<uchar>(pt) = 255;
        }

        angle = angle + msg->angle_increment;
    }
    targetMutex.lock();
    cv::Point2f currentTarget = datamatrixPoint;
    targetMutex.unlock();

    // perform a closure (dilation+erosion)
    cv::Mat closedScannedData;
    morphClosure(scannedData, closedScannedData);

    targetMutex.lock();
    // extract lines from laser points
    lines = extractLines(closedScannedData);

    // get nearest line index
    int idx = nearestLineIndex(currentTarget, lines);
    float distance = pointLineDistance(currentTarget, lines[idx]);
    targetMutex.unlock();

    //std::cout << idx << " - (" << target.x << "," << target.y << ")" << " - " << lines[idx] << " - " << distance << std::endl;

    // plot lines, points...
    cv::Mat linesColor;
    cv::cvtColor(scannedData, linesColor, CV_GRAY2BGR);

    cv::circle(linesColor, cv::Point2f(0,0)*factor+center, 3, cv::Scalar(200,200,200),3);
    for( size_t i = 0; i < lines.size(); i++ )
    {
      cv::Vec4f l = lines[i];
      cv::line( linesColor,
                cv::Point2f(l[0], l[1])*factor+center,
                cv::Point2f(l[2], l[3])*factor+center,
                i==idx?cv::Scalar(0,255,0):cv::Scalar(0,0,255),
                1, CV_AA );
      cv::circle(linesColor, cv::Point2f(l[0], l[1])*factor+center, 2, cv::Scalar(200,200,100));
      cv::circle(linesColor, cv::Point2f(l[2], l[3])*factor+center, 2, cv::Scalar(100,200,200));
    }

    cv::circle(linesColor, currentTarget*factor+center, 3, cv::Scalar(250,50,250),3);

    // snap target to the nearest line
    currentTarget = getSnapPointToLine(currentTarget, lines[idx]);

    cv::circle(linesColor, currentTarget*factor+center, 3, cv::Scalar(255,0,0),3);

//    // Do the prediction update of the Kalman filter
//    cv::Mat predicted = kfTarget.predict();

//    // Use newCentroid to correct the Kalman filter prediction
//    kfMeasurement.at<float>(0) = target.x;
//    kfMeasurement.at<float>(1) = target.y;

//    //std::cout << std::endl << "-------" << std::endl << "ID:\t" << id_ << std::endl;
//    //std::cout << "STATE:\t" << kf_pole_.statePost << std::endl;
//    //std::cout << "PREDICTED:\t" << predicted << std::endl;
//    //std::cout << "MEASURED:\t" << kf_measurement_ <<std::endl;

//    cv::Mat estimated = kfTarget.correct(kfMeasurement);

//    kfState = estimated;

//    //std::cout << "ESTIMATED:\t" << estimated << std::endl;

//    target = cv::Point2f(estimated.at<float>(0),estimated.at<float>(1));
    cv::Point2f currentTargetDirection = currentTarget;
    targetMutex.lock();
    targetDirection = currentTargetDirection;
    targetMutex.unlock();

    cv::circle(linesColor, currentTarget*factor+center, 3, cv::Scalar(255,150,150),3);

    /// Move the target 1mt in front of the line
    // get an ortogonal vector
    cv::Vec2f lineVector((lines[idx])[0] - (lines[idx])[2],
                         (lines[idx])[1] - (lines[idx])[3]);
    /// TODO: check 0 values...
    cv::Vec2f ortoLineVector(-lineVector.val[1], lineVector.val[0]);
    ortoLineVector = ortoLineVector / cv::norm(ortoLineVector);

    cv::Point2f t1,t2;
    t1 = currentTarget + 0.5 * cv::Point2f(ortoLineVector[0], ortoLineVector[1]);
    t2 = currentTarget - 0.5 * cv::Point2f(ortoLineVector[0], ortoLineVector[1]);

    if (cv::norm(t1) < cv::norm(t2))
    {
        currentTarget = t1;
    }
    else
    {
        currentTarget = t2;
    }

    targetMutex.lock();
    target = currentTarget;
    targetMutex.unlock();

    cv::circle(linesColor, currentTarget*factor+center, 3, cv::Scalar(0,255,0),3);

    std::cout << "target distance: " << cv::norm(currentTarget) << std::endl;

    cv::line( linesColor,
              currentTarget*factor+center,
              currentTargetDirection*factor+center,
              cv::Scalar(0,255,0),
              2, CV_AA );

    drawPrevPath(linesColor);

    if (screen)
    {
        cv::imwrite("_" + std::to_string(ros::Time::now().toSec()) + ".png", linesColor);
        screen = false;
    }

    cv::imshow("lines", linesColor);
    cv::waitKey(33);
}

int main(int argc, char** argv)
{
    cv::namedWindow("lines");
    targetMutex.lock();
    target = cv::Point2f(0.0f, 0.0f);
    targetDirection = cv::Point2f(0.0f, 0.0f);
    datamatrixPoint = cv::Point2f(0.0f, 0.0f);
    targetMutex.unlock();

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

    mp = new nav::EndLineTurnMP(fs);

//    // Setup kalman filter for target
//    kfTarget.init(4,2,2);
//    kfState = cv::Mat(4, 1, CV_32F);
//    kfProcessNoise = cv::Mat(4, 1, CV_32FC1);
//    kfMeasurement = cv::Mat::zeros(2, 1, CV_32F);

//    kfState << target.x,target.y,0,0;

//    kfTarget.statePost.at<float>(0) = target.x;
//    kfTarget.statePost.at<float>(1) = target.y;
//    kfTarget.statePost.at<float>(2) = 0;
//    kfTarget.statePost.at<float>(3) = 0;


//    kfTarget.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);

//    // Given that I have no control matrix, I have to rely more on the measurement.
//    // This means to have a low measureNoiseCovariance and an high processNoiseCovariance.
//    cv::setIdentity(kfTarget.measurementMatrix);
//    cv::setIdentity(kfTarget.processNoiseCov, cv::Scalar::all(1e-4));
//    cv::setIdentity(kfTarget.measurementNoiseCov, cv::Scalar::all(1e-3));
//    cv::setIdentity(kfTarget.errorCovPost, cv::Scalar::all(0.1));

//    kfTarget.gain *(cv::Mat_<float>(4, 2) << 0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9);



    ros::init(argc, argv, "egocentric_mp");

    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/Pioneer3AT/rqt/cmd_vel", 1);
    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/Pioneer3AT/laserscan", 1, scan_callback);

    tf::StampedTransform datamatrix_to_base_link;

    tf::TransformListener tfListener;

    geometry_msgs::Twist base_msg;

    float frequency = 10;
    ros::Rate loop_rate(frequency);

    //std::cout << "x;y;phi;r;sigma;theta;curvature;linear;angular;" << std::endl;
    float r = 0.0, theta = 0.0, sigma = 0.0;

    scannedData = cv::Mat::zeros(cv::Size(800,800), CV_8UC1);

    while (ros::ok())
    {
        ros::Time a = ros::Time::now();
        ros::Duration timeout(0.2);
        bool isDatamatrix = false;

        //while (!isDatamatrix)
        //{

            a = ros::Time::now();
            isDatamatrix = tfListener.waitForTransform("camera_link",
                                                       "datamatrix_frame",
                                                       a,
                                                       timeout);

            //ROS_INFO("no transform yet");
        //}

        if (isDatamatrix)
        {

            //ROS_INFO("Setting target");

            //a = ros::Time::now();
            tfListener.lookupTransform("camera_link",
                                       "datamatrix_frame",
                                       a,
                                       datamatrix_to_base_link);

            tf::Point pa,pb;

            pa.setX(0);
            pa.setY(0);
            pa.setZ(0);
            pb = datamatrix_to_base_link(pa);

            datamatrixPoint = cv::Point2f(pb.x(), -1 * pb.y());
        }
        else
        {
            ROS_INFO("tf not available");
        }

        ros::spinOnce();
        loop_rate.sleep();

        float
                v = 0.0f,
                omega = 0.0f;
        bool finish = false;
        if (isDatamatrix)
        {
            computeRThetaSigma(r, theta, sigma);
            std::cout << "r-t-s: " << r << " " << theta << " " << sigma << std::endl;
            v = mp->computeLinearVelocity(r,theta,sigma);
            omega = mp->computeAngularVelocity(v, r, theta, sigma);

            if (r <= mp->getEndEpsilon())
            {
                v = 0.0f;
                omega = 0.0f;
                finish = true;
            }
        }


        if (omega == 0.15)
        {
            //v = 0.0f;
        }
        base_msg.linear.x = v;
        base_msg.angular.z = omega;

        std::cout << "(linear, angular) = (" << v << ", " << omega << ")" << std::endl;


        cmd_vel_pub.publish(base_msg);
        if (finish)
        {
            ROS_INFO("Done.");
            break;
        }
    }

    ROS_INFO("Advance over the coil.");

    float meters = 0.15 + cv::norm(target);
    std::cout << "Distance to travel: " << meters << std::endl;
    float velocity = 0.05;

    float duration = meters / velocity;

    ros::Duration a(duration);
    ros::Time end = ros::Time::now() + a;
    while (ros::Time::now() <= end)
    {
        base_msg.linear.x = velocity;
        base_msg.angular.z = 0.0f;

        cmd_vel_pub.publish(base_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }


    base_msg.linear.x = 0.0f;
    base_msg.angular.z = 0.0f;

    cmd_vel_pub.publish(base_msg);
    ros::spinOnce();
    loop_rate.sleep();
    cmd_vel_pub.publish(base_msg);
    ros::spinOnce();
    loop_rate.sleep();
    cmd_vel_pub.publish(base_msg);
    ros::spinOnce();
    loop_rate.sleep();

    ROS_INFO("Stop. Is the robot over the coil?");

    return 0;
}
