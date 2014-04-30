#include "endlineturnmp.h"

namespace nav {

EndLineTurnMP::EndLineTurnMP(const cv::FileStorage &fs)
    : k1_(fs["Uturn"]["k1"]),
      k2_(fs["Uturn"]["k2"]),
      k3_(fs["Uturn"]["k3"]),
      kMaxV(fs["globalMP"]["maxV"]),
      end_epsilon_(fs["Uturn"]["endEpsilon"]),
      end_gamma_(fs["Uturn"]["endGamma"]),
      beta_(fs["Uturn"]["beta"]),
      lambda_(fs["Uturn"]["lambda"])
{
}

float EndLineTurnMP::computeLinearVelocity(const float r,
                                           const float theta,
                                           const float sigma)
{
    float rInv = -1 / r;

    float z = sigma - std::atan(-1 * k1_ * theta);

    curvature_ = rInv * (k2_ * z + (1 + k1_ / (1 + k1_*theta*k1_*theta)) * std::sin(sigma));

    float v = 0.0f;

    if (r < end_epsilon_)
    {
        v = 0.01f;
    }
    else
    {
        v = kMaxV / (1 + beta_ * std::pow(std::abs(curvature_), lambda_));
    }

    return v;
}


float normalizeAngle_PI(float angle)
{
    angle = angle > M_PI ? angle - 2 * M_PI : angle;
    angle = angle <= -M_PI ? angle + 2 * M_PI : angle;

    return angle;
}

float EndLineTurnMP::computeAngularVelocity(const float v,
                                            const float r,
                                            const float theta,
                                            const float sigma)
{
    float omega = curvature_ * v;


    float sign = omega>0 ? 1 : -1;

    omega = std::abs(omega)>0.15f ? sign*0.15f : omega;

    if (r <= end_epsilon_)
    {
        // lentamente mi oriento correttamente
        float diff = normalizeAngle_PI(theta - sigma);

        if (diff > end_gamma_)
        {
            omega = diff < 0 ? -0.05 : 0.05;
        }
        else
        {
            omega = 0;
        }
    }

    return omega;
}

} // namespace nav
