#ifndef ENDLINETURNMP_H
#define ENDLINETURNMP_H

#include <cmath>

#include <opencv2/opencv.hpp>

namespace nav {

class EndLineTurnMP
{
public:
    EndLineTurnMP(const cv::FileStorage &fs);

    /*!
     * \brief computeLinearVelocity -> http://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
     * \param r the distance robot-target
     * \return the linear velocity
     */
    float computeLinearVelocity(const float r,
                                const float theta,
                                const float sigma);

    /*!
     * \brief computeAngularVelocity -> http://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
     * \param v the linear velocity
     * \param r the distance robot-target
     * \param theta the angle between the target orientation and the line robot-target
     * \param sigma the angle between the robot orientation and the line robot-target
     * \return the angular velocity
     */
    float computeAngularVelocity(const float v,
                                 const float r,
                                 const float theta,
                                 const float sigma);

private:

    float               // see formula 14 of the paper
        k1_,
        k2_;

    float               // see formula 15 of the paper
        beta_,
        lambda_;

    float
        epsilon_,
        k3_;            // if r < epsilon -> v = k3_*r

    float
        curvature_;

// public constants
public:
    const float kMaxV;
    //const float kMaxOmega;    // the max omega become M_PI/4 * kMaxV

};

} // namespace nav

#endif // ENDLINETURNMP_H
