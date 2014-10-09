#ifndef __eigen_gl_h__
#define __eigen_gl_h__

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <math.h>
#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace eigen_utils {

void bot_gl_cov_ellipse(const Eigen::Matrix2d & cov, double scale);

void bot_gl_draw_vector(const Eigen::Vector3d & vec, const Eigen::Vector3d & pos = Eigen::Vector3d::Zero(), double head_width = 0.1,
    double head_length = .25, double body_width = .05);

void bot_gl_draw_axes_cov(Eigen::Matrix3d & chi_cov, double nsigma = 1);

void bot_gl_draw_axes(const Eigen::Quaterniond & orientation, const Eigen::Vector3d & pos,
    double scale);

void bot_gl_draw_axes(const Eigen::Affine3d & trans, double scale);

void bot_gl_cov_ellipse_3d(const Eigen::Matrix3d & pos_cov, const Eigen::Vector3d & mu = Eigen::Vector3d::Zero(),
    double scale = 1);

void bot_gl_mult_quat_pos(const Eigen::Quaterniond & orientation, const Eigen::Vector3d & pos);

//FIXME doesn't belong in eigen utils, but i couldn't find a home elsewhere since it ties dependencies between BotTrans and bot-vis
/**
 * bot_gl_mult_BotTrans
 *
 * multiply BotTrans derived matrix
 */
void
bot_gl_mult_BotTrans(const BotTrans * trans);

}

#endif
