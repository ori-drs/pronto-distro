#include "small_linalg.h"

int bot_matrix_inverse_4x4d (const double m[16], double inv[16])
{
    // thank you matlab.

    double det = m[0]*m[5]*m[10]*m[15]-m[0]*m[5]*m[11]*m[14]-
        m[0]*m[9]*m[6]*m[15]+m[0]*m[9]*m[7]*m[14]+m[0]*m[13]*m[6]*m[11]-
        m[0]*m[13]*m[7]*m[10]-m[4]*m[1]*m[10]*m[15]+
        m[4]*m[1]*m[11]*m[14]+m[4]*m[9]*m[2]*m[15]-
        m[4]*m[9]*m[3]*m[14]-m[4]*m[13]*m[2]*m[11]+
        m[4]*m[13]*m[3]*m[10]+m[8]*m[1]*m[6]*m[15]-
        m[8]*m[1]*m[7]*m[14]-m[8]*m[5]*m[2]*m[15]+
        m[8]*m[5]*m[3]*m[14]+m[8]*m[13]*m[2]*m[7]-
        m[8]*m[13]*m[3]*m[6]-m[12]*m[1]*m[6]*m[11]+
        m[12]*m[1]*m[7]*m[10]+m[12]*m[5]*m[2]*m[11]-
        m[12]*m[5]*m[3]*m[10]-m[12]*m[9]*m[2]*m[7]+
        m[12]*m[9]*m[3]*m[6];
    
    if (det == 0)
        return -1;

    inv[0] = m[5]*m[10]*m[15]-m[5]*m[11]*m[14]-m[9]*m[6]*m[15]+m[9]*m[7]*m[14]+m[13]*m[6]*m[11]-m[13]*m[7]*m[10];
    inv[4] =  -m[4]*m[10]*m[15]+m[4]*m[11]*m[14]+m[8]*m[6]*m[15]-m[8]*m[7]*m[14]-m[12]*m[6]*m[11]+m[12]*m[7]*m[10];
    inv[8] = m[4]*m[9]*m[15]-m[4]*m[11]*m[13]-m[8]*m[5]*m[15]+m[8]*m[7]*m[13]+m[12]*m[5]*m[11]-m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14]+m[4]*m[10]*m[13]+m[8]*m[5]*m[14]-m[8]*m[6]*m[13]-m[12]*m[5]*m[10]+m[12]*m[6]*m[9];
    inv[1] = -m[1]*m[10]*m[15]+m[1]*m[11]*m[14]+m[9]*m[2]*m[15]-m[9]*m[3]*m[14]-m[13]*m[2]*m[11]+m[13]*m[3]*m[10];
    inv[5] = m[0]*m[10]*m[15]-m[0]*m[11]*m[14]-m[8]*m[2]*m[15]+m[8]*m[3]*m[14]+m[12]*m[2]*m[11]-m[12]*m[3]*m[10];
    inv[9] = -m[0]*m[9]*m[15]+m[0]*m[11]*m[13]+m[8]*m[1]*m[15]-m[8]*m[3]*m[13]-m[12]*m[1]*m[11]+m[12]*m[3]*m[9];
    inv[13] = m[0]*m[9]*m[14]-m[0]*m[10]*m[13]-m[8]*m[1]*m[14]+m[8]*m[2]*m[13]+m[12]*m[1]*m[10]-m[12]*m[2]*m[9];
    inv[2] = m[1]*m[6]*m[15]-m[1]*m[7]*m[14]-m[5]*m[2]*m[15]+m[5]*m[3]*m[14]+m[13]*m[2]*m[7]-m[13]*m[3]*m[6];
    inv[6] = -m[0]*m[6]*m[15]+m[0]*m[7]*m[14]+m[4]*m[2]*m[15]-m[4]*m[3]*m[14]-m[12]*m[2]*m[7]+m[12]*m[3]*m[6];
    inv[10] = m[0]*m[5]*m[15]-m[0]*m[7]*m[13]-m[4]*m[1]*m[15]+m[4]*m[3]*m[13]+m[12]*m[1]*m[7]-m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14]+m[0]*m[6]*m[13]+m[4]*m[1]*m[14]-m[4]*m[2]*m[13]-m[12]*m[1]*m[6]+m[12]*m[2]*m[5];
    inv[3] = -m[1]*m[6]*m[11]+m[1]*m[7]*m[10]+m[5]*m[2]*m[11]-m[5]*m[3]*m[10]-m[9]*m[2]*m[7]+m[9]*m[3]*m[6];
    inv[7] = m[0]*m[6]*m[11]-m[0]*m[7]*m[10]-m[4]*m[2]*m[11]+m[4]*m[3]*m[10]+m[8]*m[2]*m[7]-m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11]+m[0]*m[7]*m[9]+m[4]*m[1]*m[11]-m[4]*m[3]*m[9]-m[8]*m[1]*m[7]+m[8]*m[3]*m[5];
    inv[15] = m[0]*m[5]*m[10]-m[0]*m[6]*m[9]-m[4]*m[1]*m[10]+m[4]*m[2]*m[9]+m[8]*m[1]*m[6]-m[8]*m[2]*m[5];

    for (int i = 0; i < 16; i++)
        inv[i] /= det;

    return 0;
}

int 
bot_linear_least_squares_3d (const double *A, int m, const double b[3], 
        double x[3])
{
    double AtA[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    double Atb[3] = { 0, 0, 0 };
    double row_outer_product[9];
    for (int i=0; i<m; i++) {
        const double *Arow = A + 3 * i;
        bot_vector_vector_outer_product_3d (Arow, Arow, row_outer_product);
        for (int j=0; j<9; j++) {
            AtA[j] += row_outer_product[j];
        }
        Atb[0] += Arow[0] * b[i];
        Atb[1] += Arow[1] * b[i];
        Atb[2] += Arow[2] * b[i];
    }
    double AtA_inv[9];
    if (0 != bot_matrix_inverse_3x3d (AtA, AtA_inv)) {
        x[0] = x[1] = x[2] = NAN;
        return -1;
    }

    bot_matrix_vector_multiply_3x3_3d (AtA_inv, Atb, x);
    return 0;
}

void
bot_matrix_mean_cov_3d (const double *X, int n, double mean[3], double cov[9])
{
    mean[0] = mean[1] = mean[2] = 0;
    for (int i=0; i<n; i++) {
        mean[0] += X[i*3+0];
        mean[1] += X[i*3+1];
        mean[2] += X[i*3+2];
    }
    mean[0] /= n;
    mean[1] /= n;
    mean[2] /= n;

    cov[0] = cov[1] = cov[2] = cov[4] = cov[5] = cov[8] = 0;

    for (int i=0; i<n; i++) {
        double da = X[i*3+0] - mean[0];
        double db = X[i*3+1] - mean[1];
        double dc = X[i*3+2] - mean[2];
        cov[0] += da*da;
        cov[1] += da*db;
        cov[2] += da*dc;
        cov[4] += db*db;
        cov[5] += db*dc;
        cov[8] += dc*dc;
    }
    cov[0] /= n;
    cov[1] /= n;
    cov[2] /= n;
    cov[4] /= n;
    cov[5] /= n;
    cov[8] /= n;
    cov[3] = cov[1];
    cov[6] = cov[2];
    cov[7] = cov[5];
}

void 
bot_matrix_mean_cov_2d (const double *X, int n, double mean[2], double cov[4])
{
    mean[0] = mean[1] = 0;
    for (int i=0; i<n; i++) {
        mean[0] += X[i*2+0];
        mean[1] += X[i*2+1];
    }
    mean[0] /= n;
    mean[1] /= n;

    cov[0] = cov[1] = cov[3] = 0;
    for (int i=0; i<n; i++) {
        double da = X[i*2+0] - mean[0];
        double db = X[i*2+1] - mean[1];
        cov[0] += da*da;
        cov[1] += da*db;
        cov[3] += db*db;
    }
    cov[0] /= n - 1;
    cov[1] /= n - 1;
    cov[3] /= n - 1;
    cov[2] = cov[1];
}
