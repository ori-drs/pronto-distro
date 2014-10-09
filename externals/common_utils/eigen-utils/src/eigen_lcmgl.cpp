#include "eigen_lcmgl.hpp"
#include "eigen_utils_common.hpp"

namespace eigen_utils {

using namespace Eigen;

void bot_lcmgl_vertex3d(bot_lcmgl_t * lcmgl, Eigen::Vector3d & vec)
{
  bot_lcmgl_vertex3d(lcmgl, vec(0), vec(1), vec(2));
}

void bot_lcmgl_draw_vector(bot_lcmgl_t * lcmgl, const Eigen::Vector3d & vec, const Eigen::Vector3d & pos,
    double head_width,
    double head_length, double body_width)
{
  Eigen::Quaterniond toVectorX;
  toVectorX.setFromTwoVectors(Eigen::Vector3d::UnitX(), vec);
  Eigen::AngleAxisd ang_ax(toVectorX);

  double length = vec.norm();
  lcmglPushMatrix();
  lcmglTranslated(pos(0), pos(1), pos(2));
  lcmglRotated(bot_to_degrees(ang_ax.angle()), ang_ax.axis()(0), ang_ax.axis()(1), ang_ax.axis()(2));
  lcmglTranslated(length / 2, 0, 0);
  //  bot_gl_draw_arrow_2d(vec_mag * scale, head_width, head_length, body_width, fill);

  if (head_length > length)
    head_length = length;

  bot_lcmgl_draw_arrow_3d(lcmgl, length, head_width, head_length, body_width);
  lcmglPopMatrix();
}

void bot_lcmgl_cov_ellipse(bot_lcmgl_t * lcmgl, const Eigen::Matrix2d & cov, const Eigen::Vector3d & mu3d,
    double scale, bool fill)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(cov);
  Eigen::Vector2d eig_vals = eigen_solver.eigenvalues();
  Eigen::Matrix2d eig_vecs = eigen_solver.eigenvectors();

  double a = sqrt(eig_vals(0, 0)) * scale;
  double b = sqrt(eig_vals(1, 0)) * scale;
  Eigen::Vector2d A = eig_vecs.block<2, 1>(0, 0);
  Eigen::Vector2d B = eig_vecs.block<2, 1>(0, 1);
  A.normalize();
  B.normalize();
  A *= a;
  B *= b;
  double h = mu3d(0);
  double k = mu3d(1);
  double phi = atan2Vec(A);

  if (fill)
    bot_lcmgl_begin(lcmgl, GL_POLYGON);
  else
    bot_lcmgl_begin(lcmgl, GL_LINE_LOOP);

  double d_theta = M_PI / 30;
  double t = 0;
  while (t < 2 * M_PI) {
    double x = h + a * cos(t) * cos(phi) - b * sin(t) * sin(phi);
    double y = k + b * sin(t) * cos(phi) + a * cos(t) * sin(phi);
    bot_lcmgl_vertex3d(lcmgl, x, y, mu3d(2));
    t += d_theta;
  }
  bot_lcmgl_end(lcmgl);

  lcmglBegin(GL_LINES);
  lcmglVertex2d(-A(0), -A(1));
  lcmglVertex2d(A(0), A(1));
  lcmglEnd();

  lcmglBegin(GL_LINES);
  lcmglVertex2d(-B(0), -B(1));
  lcmglVertex2d(B(0), B(1));
  lcmglEnd();
}

void bot_lcmgl_cov_ellipse(bot_lcmgl_t * lcmgl, const Eigen::Matrix2d & cov, const Eigen::Vector2d & mu2d,
    double scale, bool fill)
{
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  xyz.block<2, 1>(0, 0) = mu2d;
  bot_lcmgl_cov_ellipse(lcmgl, cov, xyz, scale, fill);
}

void bot_lcmgl_cov_ellipse_3d(bot_lcmgl_t * lcmgl, const Eigen::Matrix3d & pos_cov, const Eigen::Vector3d & mu,
    double nsigma)
{

  //new way does it with reoriented and scaled orthcircles which looks better but doesn't give marginal ellipses in each plane
  bool new_way = true;

  if (new_way) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(pos_cov);
    Eigen::Vector3d eig_vals = eigen_solver.eigenvalues();
    Eigen::Matrix3d eig_vecs = eigen_solver.eigenvectors();

    double a = sqrt(eig_vals(0)) * nsigma;
    double b = sqrt(eig_vals(1)) * nsigma;
    double c = sqrt(eig_vals(2)) * nsigma;

    eig_vecs.col(0).normalize();
    eig_vecs.col(1).normalize();
    eig_vecs.col(2) = eig_vecs.col(0).cross(eig_vecs.col(1));
    Transform<double, 3, Affine> trans(Transform<double, 3, Affine>::Identity());
    trans.linear() = eig_vecs;

    lcmglPushMatrix();
    lcmglTranslated(mu(0), mu(1), mu(2));
    lcmglMultMatrixd(trans.data());
    lcmglScalef(a, b, c);
    bot_lcmgl_draw_ortho_circles_3d(lcmgl);
    lcmglPopMatrix();
  }
  else {
    Matrix2d plane_cov;
    Vector2d zero_vec = Vector2d::Zero();

    lcmglTranslated(mu(0), mu(1), mu(2));

    //xy covariance
    plane_cov = pos_cov.topLeftCorner<2, 2>();
    bot_lcmgl_cov_ellipse(lcmgl, plane_cov, zero_vec, nsigma);

    //xz covariance
    plane_cov(0, 0) = pos_cov(0, 0);
    plane_cov(1, 1) = pos_cov(2, 2);
    plane_cov(1, 0) = pos_cov(2, 0);
    plane_cov(0, 1) = pos_cov(0, 2);
    lcmglPushMatrix();
    lcmglRotated(90, 1, 0, 0);
    //  glRotated(180, 1, 1, 0); //switch the xy axes in our drawing frame
    bot_lcmgl_cov_ellipse(lcmgl, plane_cov, zero_vec, nsigma);
    lcmglPopMatrix();

    //yz covariance
    plane_cov = pos_cov.bottomRightCorner<2, 2>();
    plane_cov(1, 0) *= -1;
    plane_cov(0, 1) *= -1;
    lcmglPushMatrix();
    lcmglRotated(90, 0, 1, 0);
    lcmglRotated(180, 1, 1, 0);
    //switch the xy axes in our drawing frame
    bot_lcmgl_cov_ellipse(lcmgl, plane_cov, zero_vec, nsigma);
    lcmglPopMatrix();
  }

}

void bot_lcmgl_mult_quat_pos(bot_lcmgl_t * lcmgl, const Eigen::Quaterniond & orientation, const Eigen::Vector3d & pos)
{
  Eigen::Transform<double, 3, Affine> trans = Translation3d(pos) * orientation;
  lcmglMultMatrixd(trans.data());
}

void bot_lcmgl_draw_axes(bot_lcmgl_t * lcmgl, const Eigen::Quaterniond & orientation, const Eigen::Vector3d & pos,
    double scale)
{
  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_mult_quat_pos(lcmgl, orientation, pos);
  bot_lcmgl_scalef(lcmgl, scale, scale, scale);
  bot_lcmgl_draw_axes(lcmgl);
  bot_lcmgl_pop_matrix(lcmgl);
}

void bot_lcmgl_draw_axes(bot_lcmgl_t * lcmgl, const Eigen::Affine3d & trans, double scale)
{
  lcmglPushMatrix();
  lcmglMultMatrixd(trans.data());
  lcmglScalef(scale, scale, scale);
  bot_lcmgl_draw_axes(lcmgl);
  lcmglPopMatrix();
}

void bot_lcmgl_transform_frame(bot_lcmgl_t * lcmgl, const Eigen::Vector3d & xyt)
{
  bot_lcmgl_translated(lcmgl, xyt(0), xyt(1), 0);
  bot_lcmgl_rotated(lcmgl, bot_to_degrees(xyt(2)), 0, 0, 1);
}

}
