#include "eigen_rigidbody.hpp"
#include "eigen_utils_common.hpp"

namespace eigen_utils {

using namespace Eigen;

std::ostream& operator<<(std::ostream& output, const RigidBodyState & state)
{
  output << "angularVelocity: " << state.angularVelocity().transpose();
  output << ", velocity: " << state.velocity().transpose();
  output << ", chi: " << state.chi().transpose();
  output << ", position: " << state.position().transpose();
  output << ", acceleration: " << state.acceleration().transpose();
  output << ", RPY: " << bot_to_degrees(state.getEulerAngles().transpose());
  return output;
}

/*
 * returns the skew symmetric matrix corresponding to vec.cross(<other vector>)
 */
Eigen::Matrix3d skewHat(const Eigen::Vector3d & vec)
{
  Eigen::Matrix3d skew_hat;
  skew_hat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_hat;
}

/**
 * returns the exponential coordinates of quat1 - quat2
 * (quat2.inverse() * quat1)
 */
Eigen::Vector3d subtractQuats(const Eigen::Quaterniond & quat1, const Eigen::Quaterniond & quat2)
{
  Eigen::Quaterniond quat_resid = quat2.inverse() * quat1;
  Eigen::AngleAxisd angle_axis_resid(quat_resid);

  double angle = angle_axis_resid.angle();
  angle = bot_mod2pi(angle);
  return angle_axis_resid.axis() * angle;
}

Eigen::Vector3d subtractRotations(const Eigen::Matrix3d & rot1, const Eigen::Matrix3d & rot2)
{
  Eigen::AngleAxisd angle_axis_resid = Eigen::AngleAxisd(rot2.inverse() * rot1);
  double angle = angle_axis_resid.angle();
  angle = bot_mod2pi(angle);
  return angle_axis_resid.axis() * angle;
}

void quaternionToBotDouble(double bot_quat[4], const Eigen::Quaterniond & eig_quat)
{
  bot_quat[0] = eig_quat.coeffs()(3);
  bot_quat[1] = eig_quat.coeffs()(0);
  bot_quat[2] = eig_quat.coeffs()(1);
  bot_quat[3] = eig_quat.coeffs()(2);
}

void botDoubleToQuaternion(Eigen::Quaterniond & eig_quat, const double bot_quat[4])
{
  eig_quat.coeffs()(3) = bot_quat[0];
  eig_quat.coeffs()(0) = bot_quat[1];
  eig_quat.coeffs()(1) = bot_quat[2];
  eig_quat.coeffs()(2) = bot_quat[3];
}

Eigen::Quaterniond setQuatEulerAngles(const Eigen::Vector3d & eulers)
{
  Eigen::Quaterniond quat = Eigen::AngleAxisd(eulers(2), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(eulers(1), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(eulers(0), Eigen::Vector3d::UnitX());

  return quat;
}

Eigen::Vector3d getEulerAngles(const Eigen::Quaterniond & quat)
{
  return quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse(); //ypr
}

Eigen::Vector3d getEulerAngles(const Eigen::Matrix3d & rot)
{
  return rot.eulerAngles(2, 1, 0).reverse(); //ypr
}

Eigen::Affine3d getTransTwistUnscaled(const Eigen::Vector3d & unscaledAngularVelocity,
    const Eigen::Vector3d & unscailedLinearVelocity)
{
  double t = unscaledAngularVelocity.norm();
  Affine3d trans;
  if (t < 0.000000001) {
    trans = Translation3d(unscailedLinearVelocity);
  }
  else {
    Vector3d omega = unscaledAngularVelocity / t;
    Vector3d v = unscailedLinearVelocity / t;

    trans = AngleAxisd(t, unscaledAngularVelocity.normalized());
    trans.translation() = (Matrix3d::Identity() - trans.rotation()) * omega.cross(v) + omega.dot(v) * omega * t;
  }
  return trans;
}

Eigen::Affine3d getTransTwist(const Eigen::Vector3d & angularVelocity, const Eigen::Vector3d & linearVelocity,
    double time)
{
  return getTransTwistUnscaled(time * angularVelocity, time * linearVelocity);
}


}

