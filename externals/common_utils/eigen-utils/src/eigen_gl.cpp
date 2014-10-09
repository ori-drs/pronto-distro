#include "eigen_gl.hpp"
#include "eigen_utils_common.hpp"



namespace eigen_utils {

void bot_gl_draw_axes()
{
  //x-axis
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glEnd();

  //y-axis
  glBegin(GL_LINES);
  glColor3f(0, 1, 0);
  glVertex3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glEnd();

  //z-axis
  glBegin(GL_LINES);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glEnd();

} //FIXME Not sure why this needed to be copied for gl_util.h in libbot, namespacing c++ issue?

using namespace Eigen;

void bot_gl_cov_ellipse(const Eigen::Matrix2d & cov, double scale)
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
  double phi = atan2Vec(A);

  glBegin(GL_LINES);
  glVertex2d(-A(0), -A(1));
  glVertex2d(A(0), A(1));
  glEnd();

  glBegin(GL_LINES);
  glVertex2d(-B(0), -B(1));
  glVertex2d(B(0), B(1));
  glEnd();

  bot_gl_draw_ellipse(b, a, phi, 100);
}

void bot_gl_draw_vector(const Eigen::Vector3d & vec, const Vector3d & pos, double head_width, double head_length,
    double body_width)
{
  Eigen::Quaterniond toVectorX;
  toVectorX.setFromTwoVectors(Eigen::Vector3d::UnitX(), vec);
  Eigen::AngleAxisd ang_ax(toVectorX);

  double length = vec.norm();
  glPushMatrix();
  glTranslated(pos(0), pos(1), pos(2));
  glRotated(bot_to_degrees(ang_ax.angle()), ang_ax.axis()(0), ang_ax.axis()(1), ang_ax.axis()(2));
  glTranslated(length / 2, 0, 0);
  //  bot_gl_draw_arrow_2d(vec_mag * scale, head_width, head_length, body_width, fill);

  if (head_length > length)
    head_length = length;

  bot_gl_draw_arrow_3d(length, head_width, head_length, body_width);
  glPopMatrix();
}

void bot_gl_draw_axes_cov(Eigen::Matrix3d & chi_cov, double nsigma)
{
  Matrix2d axis_cov;

  //z point, xy covariance
  axis_cov = chi_cov.topLeftCorner<2, 2>();
  axis_cov(1, 0) *= -1;
  axis_cov(0, 1) *= -1;
  glPushMatrix();
  glTranslated(0, 0, 1); //get to tip of z axis
  glRotated(180, 1, 1, 0); //switch the xy axes in our drawing frame
  glColor4d(0, 0, 1, 1);
  bot_gl_cov_ellipse(axis_cov, nsigma);
  glPopMatrix();

  //y point, xz covariance
  axis_cov(0, 0) = chi_cov(0, 0);
  axis_cov(1, 1) = chi_cov(2, 2);
  axis_cov(1, 0) = chi_cov(2, 0);
  axis_cov(0, 1) = chi_cov(0, 2);
  glPushMatrix();
  glRotated(90, -1, 0, 0);
  glTranslated(0, 0, 1); //get to tip of z axis
  glRotated(180, 1, 1, 0); //switch the xy axes in our drawing frame
  glColor4d(0, 1, 0, 1);
  bot_gl_cov_ellipse(axis_cov, nsigma);
  glPopMatrix();

  //x point, yz covariance
  axis_cov = chi_cov.bottomRightCorner<2, 2>();
//  axis_cov(1, 0) *= -1;
//  axis_cov(0, 1) *= -1;
  glPushMatrix();
  glRotated(90, 0, 1, 0);
  glTranslated(0, 0, 1); //get to tip of z axis
//  glRotated(180, 1, 1, 0); //switch the xy axes in our drawing frame
  glColor4d(1, 0, 0, 1);
  bot_gl_cov_ellipse(axis_cov, nsigma);
  glPopMatrix();
}

void bot_gl_draw_axes(const Eigen::Quaterniond & orientation, const Eigen::Vector3d & pos,
    double scale)
{
  glPushMatrix();
  bot_gl_mult_quat_pos(orientation, pos);
  glScalef(scale, scale, scale);
  bot_gl_draw_axes();
  bot_gl_print_current_matrix();
  glPopMatrix();
}

void bot_gl_draw_axes(const Eigen::Affine3d & trans, double scale)
{
  glPushMatrix();
  glMultMatrixd(trans.data());
  glScalef(scale, scale, scale);
  bot_gl_draw_axes();
  glPopMatrix();
}

void bot_gl_cov_ellipse_3d(const Eigen::Matrix3d & pos_cov, const Eigen::Vector3d & mu, double nsigma)
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

    glPushMatrix();
    glTranslated(mu(0), mu(1), mu(2));
    glMultMatrixd(trans.data());
    glScaled(a, b, c);
    bot_gl_draw_ortho_circles_3d();
    glPopMatrix();
  }
  else {
    glPushMatrix();
    glTranslated(mu(0), mu(1), mu(2));
    Matrix2d plane_cov;

    //xy covariance
    plane_cov = pos_cov.topLeftCorner<2, 2>();
    bot_gl_cov_ellipse(plane_cov, nsigma);

    //xz covariance
    plane_cov(0, 0) = pos_cov(0, 0);
    plane_cov(1, 1) = pos_cov(2, 2);
    plane_cov(1, 0) = pos_cov(2, 0);
    plane_cov(0, 1) = pos_cov(0, 2);
    glPushMatrix();
    glRotated(90, 1, 0, 0);
//  glRotated(180, 1, 1, 0); //switch the xy axes in our drawing frame
    bot_gl_cov_ellipse(plane_cov, nsigma);
    glPopMatrix();

    //yz covariance
    plane_cov = pos_cov.bottomRightCorner<2, 2>();
    plane_cov(1, 0) *= -1;
    plane_cov(0, 1) *= -1;
    glPushMatrix();
    glRotated(90, 0, 1, 0);
    glRotated(180, 1, 1, 0); //switch the xy axes in our drawing frame
    bot_gl_cov_ellipse(plane_cov, nsigma);
    glPopMatrix();

    glPopMatrix();
  }
}

void bot_gl_mult_quat_pos(const Eigen::Quaterniond & orientation, const Eigen::Vector3d & pos)
{
  Eigen::Transform<double, 3, Affine> trans = Translation3d(pos) * orientation;
  glMultMatrixd(trans.data());
}

void bot_gl_mult_BotTrans(const BotTrans * trans)
{
  double curr_quat_m[16];
  double curr_quat_m_opengl[16];
  bot_quat_pos_to_matrix(trans->rot_quat, trans->trans_vec, curr_quat_m);
  bot_matrix_transpose_4x4d(curr_quat_m, curr_quat_m_opengl);
  glMultMatrixd(curr_quat_m_opengl);
}

}

