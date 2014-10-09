#include <stdlib.h>
#include <string.h>
#include <glib.h>

#include "small_linalg.h"
#include "rotations.h"
#include "trans.h"
#include "math_util.h"

void
bot_trans_set_identity(BotTrans *btrans)
{
    btrans->rot_quat[0] = 1;
    btrans->rot_quat[1] = 0;
    btrans->rot_quat[2] = 0;
    btrans->rot_quat[3] = 0;
    btrans->trans_vec[0] = 0;
    btrans->trans_vec[1] = 0;
    btrans->trans_vec[2] = 0;
}

void
bot_trans_copy(BotTrans * dest, const BotTrans *src)
{
    memcpy(dest, src, sizeof(BotTrans));
}

void
bot_trans_set_from_quat_trans(BotTrans *btrans, const double rot_quat[4],
        const double trans_vec[3])
{
    // TODO check that rot_quat is a valid quaternion
    memcpy(btrans->rot_quat, rot_quat, 4*sizeof(double));
    memcpy(btrans->trans_vec, trans_vec, 3*sizeof(double));
}

void
bot_trans_set_from_velocities(BotTrans *dest, const double angular_rate[3],
    const double velocity[3], double dt)
{
  //see Frazolli notes, Aircraft Stability and Control, lecture 2, page 15
  if (dt == 0) {
    bot_trans_set_identity(dest);
    return;
  }

  double identity_quat[4] = { 1, 0, 0, 0 };
  double norm_angular_rate = bot_vector_magnitude_3d(angular_rate);

  if (norm_angular_rate == 0) {
    double delta[3];
    memcpy(delta, velocity, 3 * sizeof(double));
    bot_vector_scale_3d(delta, dt);
    bot_trans_set_from_quat_trans(dest, identity_quat, delta);
    return;
  }

  //"exponential of a twist: R=exp(skew(omega*t));
  //rescale vel, omega, t so ||omega||=1
  //trans =  (I-R)*(omega \cross v) + (omega \dot v) *omega* t
  //                term2                       term3
  // R*(omega \cross v)
  //     term1
  //rescale
  double t = dt * norm_angular_rate;
  double omega[3], vel[3];
  memcpy(omega, angular_rate, 3 * sizeof(double));
  memcpy(vel, velocity, 3 * sizeof(double));
  bot_vector_scale_3d(omega, 1.0/norm_angular_rate);
  bot_vector_scale_3d(vel, 1.0/norm_angular_rate);

  //compute R (quat in our case)
  bot_angle_axis_to_quat(t, omega, dest->rot_quat);

  //cross and dot products
  double omega_cross_vel[3];
  bot_vector_cross_3d(omega, vel, omega_cross_vel);
  double omega_dot_vel = bot_vector_dot_3d(omega, vel);


  double term1[3];
  double term2[3];
  double term3[3];

  //(I-R)*(omega \cross v) = term2
  memcpy(term1, omega_cross_vel, 3 * sizeof(double));
  bot_quat_rotate(dest->rot_quat, term1);
  bot_vector_subtract_3d(omega_cross_vel, term1, term2);

  //(omega \dot v) *omega* t
  memcpy(term3, omega, 3 * sizeof(double));
  bot_vector_scale_3d(term3, omega_dot_vel * t);

  bot_vector_add_3d(term2, term3, dest->trans_vec);
}

void
bot_trans_apply_trans(BotTrans *dest, const BotTrans * src)
{
    bot_quat_rotate(src->rot_quat, dest->trans_vec);
    double qtmp[4];
    bot_quat_mult(qtmp, src->rot_quat, dest->rot_quat);
    memcpy(dest->rot_quat, qtmp, 4*sizeof(double));
    dest->trans_vec[0] += src->trans_vec[0];
    dest->trans_vec[1] += src->trans_vec[1];
    dest->trans_vec[2] += src->trans_vec[2];
}

void bot_trans_apply_trans_to(const BotTrans * src1, const BotTrans * src2, BotTrans * dest){
  BotTrans tmp; //allow in place operation
  bot_trans_copy(&tmp,src2);
  bot_trans_apply_trans(&tmp,src1);
  bot_trans_copy(dest,&tmp);
  return;
}


void
bot_trans_invert(BotTrans * btrans)
{
    btrans->trans_vec[0] = -btrans->trans_vec[0];
    btrans->trans_vec[1] = -btrans->trans_vec[1];
    btrans->trans_vec[2] = -btrans->trans_vec[2];
    bot_quat_rotate_rev(btrans->rot_quat, btrans->trans_vec);
    btrans->rot_quat[1] = -btrans->rot_quat[1];
    btrans->rot_quat[2] = -btrans->rot_quat[2];
    btrans->rot_quat[3] = -btrans->rot_quat[3];
}


void bot_trans_invert_and_compose(const BotTrans * curr, const BotTrans * prev, BotTrans * delta){
  bot_trans_copy(delta,prev);
  bot_trans_invert(delta);
  bot_trans_apply_trans_to(delta,curr,delta);
  return;
}

void
bot_trans_interpolate(BotTrans *dest, const BotTrans * trans_a,
        const BotTrans * trans_b, double weight_b)
{
    bot_vector_interpolate_3d(trans_a->trans_vec, trans_b->trans_vec,weight_b,
        dest->trans_vec);
    bot_quat_interpolate(trans_a->rot_quat, trans_b->rot_quat, weight_b,
            dest->rot_quat);
}

void
bot_trans_rotate_vec(const BotTrans * btrans,
        const double src[3], double dst[3])
{
    bot_quat_rotate_to(btrans->rot_quat, src, dst);
//    bot_matrix_vector_multiply_3x3_3d(btrans->rot_mat, src, dst);
}

void
bot_trans_apply_vec(const BotTrans * btrans, const double src[3],
        double dst[3])
{
    bot_quat_rotate_to(btrans->rot_quat, src, dst);
    dst[0] += btrans->trans_vec[0];
    dst[1] += btrans->trans_vec[1];
    dst[2] += btrans->trans_vec[2];
}

void
bot_trans_get_rot_mat_3x3(const BotTrans * btrans, double rot_mat[9])
{
    bot_quat_to_matrix(btrans->rot_quat, rot_mat);
//    memcpy(rot_mat, btrans->rot_mat, 9*sizeof(double));
}

void
bot_trans_get_mat_4x4(const BotTrans *btrans, double mat[16])
{
    double tmp[9];
    bot_quat_to_matrix(btrans->rot_quat, tmp);

    // row 1
    memcpy(mat+0, tmp+0, 3*sizeof(double));
    mat[3] = btrans->trans_vec[0];

    // row 2
    memcpy(mat+4, tmp+3, 3*sizeof(double));
    mat[7] = btrans->trans_vec[1];

    // row 3
    memcpy(mat+8, tmp+6, 3*sizeof(double));
    mat[11] = btrans->trans_vec[2];

    // row 4
    mat[12] = 0;
    mat[13] = 0;
    mat[14] = 0;
    mat[15] = 1;
}

void
bot_trans_get_mat_3x4(const BotTrans *btrans, double mat[12])
{
    double tmp[9];
    bot_quat_to_matrix(btrans->rot_quat, tmp);

    // row 1
    memcpy(mat+0, tmp+0, 3*sizeof(double));
    mat[3] = btrans->trans_vec[0];

    // row 2
    memcpy(mat+4, tmp+3, 3*sizeof(double));
    mat[7] = btrans->trans_vec[1];

    // row 3
    memcpy(mat+8, tmp+6, 3*sizeof(double));
    mat[11] = btrans->trans_vec[2];
}

void
bot_trans_get_trans_vec(const BotTrans * btrans, double trans_vec[3])
{
    memcpy(trans_vec, btrans->trans_vec, 3*sizeof(double));
}

void bot_trans_print_trans(const BotTrans * tran)
{
  double rpy[3];
  bot_quat_to_roll_pitch_yaw(tran->rot_quat, rpy);
  printf("t=(%f %f %f) rpy=(%f,%f,%f)", tran->trans_vec[0], tran->trans_vec[1],
      tran->trans_vec[2], bot_to_degrees(rpy[0]), bot_to_degrees(rpy[1]), bot_to_degrees(rpy[2]));
}
