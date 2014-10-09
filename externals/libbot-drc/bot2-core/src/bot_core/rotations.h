#ifndef __bot_rotations_h_
#define __bot_rotations_h_

/**
 * @defgroup BotCoreRotations Rotations
 * @ingroup BotCoreMathGeom
 * @brief Rotation Utilities
 * @include: bot_core/bot_core.h
 *
 * Functions for working with rotations and converting between
 * different rotation representations.
 *
 * The representations are:
 *
 * Quaternion           double[4]
 * Angle/Axis           double, double[3]
 * Roll/Pitch/Yaw       double[3]
 * Rotation matrix      double[9]
 * Transform matrix     double[16]
 *
 * Note that these conventions have the property that each
 * representation yields a different function signature.
 *
 * Linking: `pkg-config --libs bot2-core`
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Multiply quaternions a and b, storing the result in c.
 *
 * When composing rotations, the resulting quaternion represents 
 * first rotating by b, then rotation by a.
 */
void bot_quat_mult (double c[4], const double a[4], const double b[4]);

/**
 * bot_quat_rotate:
 * @rot: Unit quaternion that specifies the rotation.
 * @v:   3-vector that is rotated according to the quaternion @rot
 *       and modified in place with the result.
 *
 * Rotates a vector @v from one coordinate system to another as
 * specified by a unit quaternion @rot.
 */
void bot_quat_rotate (const double quat[4], double v[3]);
void bot_quat_rotate_rev (const double quat[4], double v[3]);

/**
 * bot_quat_rotate_to:
 *
 * same as bot_quat_rotate, but not necessarily in place (although %v and
 * %result can be the same)
 */
void bot_quat_rotate_to (const double quat[4], const double v[3], 
        double result[3]);

/**
 * rotate a vector %v by %quat, and then add a translational offset %trans
 */
void bot_quat_rotate_and_translate (const double quat[4], 
        const double trans[3], const double v[3], double result[3]);

int bot_quat_to_matrix(const double quat[4], double rot[9]);

int bot_matrix_to_quat(const double rot[9], double quat[4]);

int bot_quat_pos_to_matrix(const double quat[4], const double pos[3], 
        double m[16]);

/** quat_from_angle_axis:
 *
 * populates a quaternion so that it represents a rotation of theta radians
 * about the axis <x,y,z>
 **/
void bot_angle_axis_to_quat (double theta, const double axis[3], double q[4]);

void bot_quat_to_angle_axis (const double q[4], double *theta, double axis[3]);

/**
 * converts a rotation from RPY representation (radians) into unit quaternion
 * representation
 *
 * rpy[0] = roll
 * rpy[1] = pitch
 * rpy[2] = yaw
 */
void bot_roll_pitch_yaw_to_quat(const double rpy[3], double q[4]);

/**
 * converts a rotation from unit quaternion representation to RPY
 * representation.  Resulting values are in radians.
 *
 * If any of roll, pitch, or yaw are NULL, then they are not set.
 *
 * rpy[0] = roll
 * rpy[1] = pitch
 * rpy[2] = yaw
 */
void bot_quat_to_roll_pitch_yaw (const double q[4], double rpy[3]);
  
/* These doesn't truly belong with the quaternion functions, but are useful
 * and sort of fits in here.
 */
void bot_roll_pitch_yaw_to_angle_axis (const double rpy[3], double *angle,
				   double axis[3]);

void bot_angle_axis_to_roll_pitch_yaw (double angle, const double axis[3],
				   double rpy[3]);

void bot_rodrigues_to_quat(const double r[3], double q[4]);

void bot_quat_to_rodrigues(const double q[4], double r[3]);

// runs some sanity checks
int bot_quaternion_test(void);

/**
 * bot_quat_interpolate:
 * Spherical linear interpolation of two unit quaternions.  Computes the
 * quaternion that lies at point (q0 * (1-u) + q1 * u) on the unit quaternion
 * sphere.
 */
void bot_quat_interpolate(const double q0[4], const double q1[4], double u, 
        double result[4]);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
