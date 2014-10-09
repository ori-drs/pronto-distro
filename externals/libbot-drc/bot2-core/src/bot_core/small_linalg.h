#ifndef __bot_small_linalg_h__
#define __bot_small_linalg_h__

/**
 * @defgroup BotCoreLinalg Linear Algebra
 * @ingroup BotCoreMathGeom
 * @brief Convenience functions for small linear algebra operations
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Determinant of a 3x3 matrix
 */
static inline double
bot_matrix_determinant_3x3d (const double m[9])
{
    return m[0]*(m[4]*m[8] - m[5]*m[7])
         - m[1]*(m[3]*m[8] - m[5]*m[6])
         + m[2]*(m[3]*m[7] - m[4]*m[6]);
}

/**
 * Determinant of a 2x2 matrix
 */
static inline double
bot_matrix_determinant_2x2(const double m[4])
{
    return m[0]*m[3] - m[1]*m[2];
}

/*
 * Convenience function for computing the cross product of 2D vectors
 */
static inline double
bot_matrix_determinant_2x2_by_row(const double m_1[2], const double m_2[2])
{
    return m_1[0]*m_2[1] - m_1[1]*m_2[0];
}

/**
 * Inverse of a 4x4 matrix.
 *
 * @returns 0 on success, -1 if matrix is singular
 */
int bot_matrix_inverse_4x4d (const double m[16], double inv[16]);

/**
 * Inverse of a 3x3 matrix
 *
 * @returns 0 on success, -1 if matrix is singular
 */
static inline int
bot_matrix_inverse_3x3d (const double m[9], double inverse[9])
{
    double det = bot_matrix_determinant_3x3d (m);
    if (det == 0) return -1;
    double det_inv = 1.0 / det;
    inverse[0] = det_inv * (m[4]*m[8] - m[5]*m[7]);
    inverse[1] = det_inv * (m[2]*m[7] - m[1]*m[8]);
    inverse[2] = det_inv * (m[1]*m[5] - m[2]*m[4]);
    inverse[3] = det_inv * (m[5]*m[6] - m[3]*m[8]);
    inverse[4] = det_inv * (m[0]*m[8] - m[2]*m[6]);
    inverse[5] = det_inv * (m[2]*m[3] - m[0]*m[5]);
    inverse[6] = det_inv * (m[3]*m[7] - m[4]*m[6]);
    inverse[7] = det_inv * (m[1]*m[6] - m[0]*m[7]);
    inverse[8] = det_inv * (m[0]*m[4] - m[1]*m[3]);

    return 0;
}

/**
 * Inverse of a 2x2 matrix
 *
 * @returns 0 on success, -1 if matrix is singular
 */
static inline int
bot_matrix_inverse_2x2d(const double m[4], double inverse[4])
{
    double det = m[0]*m[3] - m[1]*m[2];
    if (det == 0) return -1;

    inverse[0] = m[3] / det;
    inverse[1] = -m[1] / det;
    inverse[2] = -m[2] / det;
    inverse[3] = m[0] / det;
    return 0;
}

static inline void
bot_matrix_vector_multiply_2x2_2d (const double m[4], const double v[2],
        double result[2])
{
    result[0] = m[0]*v[0] + m[1]*v[1];
    result[1] = m[2]*v[0] + m[3]*v[1];
}

static inline void
bot_matrix_vector_multiply_3x3_3d (const double m[9], const double v[3],
        double result[3])
{
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
    result[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
    result[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

static inline void
bot_vector_affine_transform_3d (const double A[9], const double b[3],
        const double v[3], double result[3])
{
    result[0] = A[0]*v[0] + A[1]*v[1] + A[2]*v[2] + b[0];
    result[1] = A[3]*v[0] + A[4]*v[1] + A[5]*v[2] + b[1];
    result[2] = A[6]*v[0] + A[7]*v[1] + A[8]*v[2] + b[2];
}

// applies an affine transformation to a 3-d vector, where the first 3 rows and
// columns of a matrix represent the linear transformation, and the
// last column of the matrix is the translational offset
static inline void
bot_vector_affine_transform_3x4_3d(const double m[12], const double v[3],
                               double result[3]) {
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11];
}

static inline void
bot_matrix_vector_multiply_4x4_4d (const double m[16], const double v[4],
        double result[4])
{
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3]*v[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7]*v[3];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
    result[3] = m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3];
}

static inline void
bot_vector_add_3d (const double v1[3], const double v2[3], double result[3])
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

static inline void
bot_vector_add_2d (const double v1[2], const double v2[2], double result[2])
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
}

static inline void
bot_vector_add_nd (const double * v1, const double * v2, int N, double * result)
{
    int i;
    for (i = 0; i < N; i++)
        result[i] = v1[i] + v2[i];
}

static inline void
bot_vector_sub_nd (const double * v1, const double * v2, int N, double * result)
{
    int i;
    for (i = 0; i < N; i++)
        result[i] = v1[i] - v2[i];
}

/**
 * bot_vector_subtract_3d:
 *
 * computes v1 - v2
 */
static inline void
bot_vector_subtract_3d (const double v1[3], const double v2[3], double result[3])
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

static inline void
bot_vector_subtract_2d (const double v1[2], const double v2[2], double result[2])
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
}

static inline double
bot_vector_dot_2d (const double v1[2], const double v2[2])
{
    return v1[0]*v2[0] + v1[1]*v2[1];
}

static inline double
bot_vector_dot_3d (const double v1[3], const double v2[3])
{
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

static inline void
bot_vector_cross_3d (const double v1[3], const double v2[3], double result[3])
{
    result[0] = v1[1]*v2[2] - v1[2]*v2[1];
    result[1] = v1[2]*v2[0] - v1[0]*v2[2];
    result[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

static inline double
bot_vector_magnitude_squared_2d (const double v[2])
{
    return v[0]*v[0] + v[1]*v[1];
}

static inline double
bot_vector_magnitude_2d (const double v[2])
{
    return sqrt(v[0]*v[0] + v[1]*v[1]);
}

static inline double
bot_vector_magnitude_3d (const double v[3])
{
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static inline double
bot_vector_magnitude_squared_3d (const double v[3])
{
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

static inline double
bot_vector_dist_3d (const double v1[3], const double v2[3])
{
    double v[3];
    bot_vector_subtract_3d (v1, v2, v);
    return bot_vector_magnitude_3d (v);
}

static inline double
bot_vector_dist_squared_3d (const double v1[3], const double v2[3])
{
    double v[3];
    bot_vector_subtract_3d (v1, v2, v);
    return bot_vector_magnitude_squared_3d (v);
}

static inline double
bot_vector_dist_2d (const double v1[2], const double v2[2])
{
    double v[2];
    bot_vector_subtract_2d (v1, v2, v);
    return bot_vector_magnitude_2d (v);
}

static inline double
bot_vector_dist_squared_2d (const double v1[2], const double v2[2])
{
    return (v1[0]-v2[0])*(v1[0]-v2[0]) + (v1[1]-v2[1])*(v1[1]-v2[1]);
}

static inline void
bot_vector_normalize_2d (double v[2])
{
    double mag = bot_vector_magnitude_2d (v);
    v[0] /= mag;
    v[1] /= mag;
}

static inline void
bot_vector_normalize_3d (double v[3])
{
    double mag = bot_vector_magnitude_3d (v);
    v[0] /= mag;
    v[1] /= mag;
    v[2] /= mag;
}

static inline void
bot_vector_scale_2d (double v[2], double s)
{
    v[0] *= s;
    v[1] *= s;
}

static inline void
bot_vector_scale_3d (double v[3], double s)
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
}

static inline void
bot_vector_scale_nd (double v[],int N, double s)
{
  int i;
  for (i = 0; i < N; i++)
    v[i] *= s;
}

// z = a * x + y
static inline void
bot_vector_saxpy_3d (double a, const double x[3], const double y[3], double z[3])
{
    double t[3] = { a * x[0] + y[0], a * x[1] + y[1], a * x[2] + y[2] };
    z[0] = t[0]; z[1] = t[1]; z[2] = t[2];
}

static inline double
bot_vector_angle_2d (const double v1[2], const double v2[2])
{
    double mag1 = bot_vector_magnitude_2d (v1);
    double mag2 = bot_vector_magnitude_2d (v2);
    double dot = bot_vector_dot_2d (v1, v2);

    double costheta = dot / ( mag1 * mag2);
    if (costheta > 1) return 0;
    double theta = acos (costheta);

    if ( (v1[0]*v2[1] - v1[1]*v2[0]) < 0) {
        return -theta;
    }

    return theta;
}

static inline double
bot_vector_angle_3d (const double v1[3], const double v2[3])
{
    double mag1 = bot_vector_magnitude_3d (v1);
    double mag2 = bot_vector_magnitude_3d (v2);
    double dot = bot_vector_dot_3d (v1, v2);
    double costheta = dot / (mag1*mag2);
    if (costheta > 1) return 0;
    return acos (costheta);
}

static inline void
bot_vector_interpolate_3d(const double ta[3], const double tb[3], double weight_b,
        double result[3]){
    result[0] = ta[0] * (1-weight_b) + tb[0] * weight_b;
    result[1] = ta[1] * (1-weight_b) + tb[1] * weight_b;
    result[2] = ta[2] * (1-weight_b) + tb[2] * weight_b;

}

static inline void
bot_matrix_transpose_3x3d (const double m[9], double result[9])
{
    result[0] = m[0];
    result[1] = m[3];
    result[2] = m[6];
    result[3] = m[1];
    result[4] = m[4];
    result[5] = m[7];
    result[6] = m[2];
    result[7] = m[5];
    result[8] = m[8];
}

static inline void
bot_matrix_transpose_4x4d (const double m[16], double result[16])
{
    result[0] = m[0];
    result[1] = m[4];
    result[2] = m[8];
    result[3] = m[12];
    result[4] = m[1];
    result[5] = m[5];
    result[6] = m[9];
    result[7] = m[13];
    result[8] = m[2];
    result[9] = m[6];
    result[10] = m[10];
    result[11] = m[14];
    result[12] = m[3];
    result[13] = m[7];
    result[14] = m[11];
    result[15] = m[15];
}

static inline void
bot_matrix_multiply_4x4_4x4 (const double a[16], const double b[16], double r[16])
{
    int i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            double acc = 0;
            for (k = 0; k < 4; k++)
                acc += a[4*i + k] * b[j + 4*k];
            r[i*4+j] = acc;
        }
    }
}

static inline void
bot_matrix_multiply_3x3_3x3 (const double a[9], const double b[9], double r[9])
{
    int i, j, k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            double acc = 0;
            for (k = 0; k < 3; k++)
                acc += a[3*i + k] * b[j + 3*k];
            r[i*3+j] = acc;
        }
    }
}

static inline void
bot_matrix_multiply (const double *a, int a_nrows, int a_ncols,
        const double *b, int b_nrows, int b_ncols,
        double *result)
{
    int i, j, r;
    assert (a_ncols == b_nrows);
    for (i=0; i<a_nrows; i++) {
        for (j=0; j<b_ncols; j++) {
            double acc = 0;
            for (r=0; r<a_ncols; r++) {
                acc += a[i*a_ncols + r] * b[r*b_ncols + j];
            }
            result[i*b_ncols + j] = acc;
        }
    }
}

static inline int
bot_matrix_rigid_body_transform_inverse_4x4d (const double m[16], double inv[16])
{
    // TODO handle case where m[15] != 1

    double rot_inv_1[3] = { m[0], m[4], m[8] };
    double rot_inv_2[3] = { m[1], m[5], m[9] };
    double rot_inv_3[3] = { m[2], m[6], m[10] };
    double t_inv[3] = { -m[3], -m[7], -m[11] };

    inv[0] = m[0];
    inv[1] = m[4];
    inv[2] = m[8];
    inv[3] = bot_vector_dot_3d (rot_inv_1, t_inv);
    inv[4] = m[1];
    inv[5] = m[5];
    inv[6] = m[9];
    inv[7] = bot_vector_dot_3d (rot_inv_2, t_inv);
    inv[8] = m[2];
    inv[9] = m[6];
    inv[10] = m[10];
    inv[11] = bot_vector_dot_3d (rot_inv_3, t_inv);
    inv[12] = inv[13] = inv[14] = 0;
    inv[15] = m[15];
    return 0;
}

static inline void
bot_matrix_rigid_body_transform_get_rotation_matrix_4x4d (const double m[16],
        double rot[16])
{
    rot[0]  = m[0]; rot[1]  = m[1]; rot[2]  = m[2];   rot[3] = 0;
    rot[4]  = m[4]; rot[5]  = m[5]; rot[6]  = m[6];   rot[7] = 0;
    rot[8]  = m[8]; rot[9]  = m[9]; rot[10] = m[10];  rot[11] = 0;
    rot[12] =       rot[13] =       rot[14] = 0;      rot[15] = 1;
}

static inline void
bot_matrix_rigid_body_transform_get_translation_matrix_4x4d (const double m[16],
        double t[16])
{
    t[0]  = 1; t[1]  = 0; t[2]  = 0; t[3]  = m[3];
    t[4]  = 0; t[5]  = 1; t[6]  = 0; t[7]  = m[7];
    t[8]  = 0; t[9]  = 0; t[10] = 1; t[11] = m[11];
    t[12] =    t[13] =    t[14] = 0; t[15] = 1;
}

static inline void
bot_matrix_print(const double *a, int rows, int cols)
{
    int r, c;
    for (r = 0; r < rows; r++) {
        for (c = 0; c < cols; c++)  {
            printf("%10f ", a[r*cols + c]);
        }
        printf("\n");
    }
}

static inline void
bot_vector_print_3d(const double a[3])
{
    int i;
    for (i = 0; i < 3; i++)
        printf("%15f\n", a[i]);
    printf("\n");
}

static inline void
bot_vector_vector_outer_product_3d (const double v1[3], const double v2[3],
        double result[9])
{
    result[0] = v1[0] * v2[0];
    result[1] = v1[1] * v2[0];
    result[2] = v1[2] * v2[0];
    result[3] = result[1];
    result[4] = v1[1] * v2[1];
    result[5] = v1[2] * v2[1];
    result[6] = result[2];
    result[7] = result[5];
    result[8] = v1[2] * v2[2];
}

/**
 * Finds the vector x that minimizes ||Ax - b||, where A is a %m x 3 matrix
 *
 * Computes and solves (A'A)x = A'b
 *
 * returns: 0 on sucess, -1 on failure
 */
int bot_linear_least_squares_3d (const double *A, int m, const double b[3],
        double x[3]);

void bot_matrix_mean_cov_3d (const double *X, int n, double mean[3], double cov[9]);

void bot_matrix_mean_cov_2d (const double *X, int n, double mean[2], double cov[4]);

/*
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
