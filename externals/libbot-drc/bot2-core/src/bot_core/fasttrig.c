#define _GNU_SOURCE
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

#include <time.h>
#include <sys/time.h>
static int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

#include "fasttrig.h"

#define K_MSB_BITS 10
#define K_LSB_BITS 10

#define K_MSB_TABLE_SIZE (1 << K_MSB_BITS)
#define K_LSB_TABLE_SIZE (1 << K_LSB_BITS)
#define K_BITS  (K_MSB_BITS + K_LSB_BITS)

struct tsincos
{
    float m_sin, m_cos;
};

static struct tsincos msb_table[K_MSB_TABLE_SIZE];
static struct tsincos lsb_table[K_LSB_TABLE_SIZE];
static int initialized = 0;

void bot_fasttrig_init(void)
{
    for (int i = 0; i < K_MSB_TABLE_SIZE; i++) {
        double theta = (2 * M_PI * i / K_MSB_TABLE_SIZE);
        msb_table[i].m_sin = sin(theta);
        msb_table[i].m_cos = cos(theta);
    }

    for (int i = 0; i < K_LSB_TABLE_SIZE; i++) {
        double theta = (2 * M_PI * i / (K_MSB_TABLE_SIZE * K_LSB_TABLE_SIZE));
        lsb_table[i].m_sin = sin(theta);
        lsb_table[i].m_cos = cos(theta);
    }

    initialized = 1;
}

/** compute sincos, accurate to one in 1<<(K_MSB_BITS + K_LSB_BITS - 3) **/
void bot_fasttrig_sincos(double theta, double *s, double *c)
{
    if (!initialized)
        bot_fasttrig_init();

    uint32_t idx = (K_MSB_TABLE_SIZE * K_LSB_TABLE_SIZE / (2*M_PI)) * theta;

    // rewrite theta = M + L, where L is very small.
    int lsb_idx = idx & (K_LSB_TABLE_SIZE - 1);
    float sinL, cosL;

    if (0) {
        // compute sinL/cosL using small angle approximation.
        // Less memory (don't need LSB table), but a bit slower.
        float L = (2.0 * M_PI / K_MSB_TABLE_SIZE / K_LSB_TABLE_SIZE) * lsb_idx;
        sinL = L;
        cosL = 1.0 - L*L*.5;
    } else {
        // compute sinL/cosL using lookup table
        sinL = lsb_table[lsb_idx].m_sin;
        cosL = lsb_table[lsb_idx].m_cos;
    }

    int msb_idx = (idx >> K_LSB_BITS) & (K_MSB_TABLE_SIZE - 1);
    float sinM = msb_table[msb_idx].m_sin;
    float cosM = msb_table[msb_idx].m_cos;

    // angle sum formulas
    // we lose a few bits of precision here... about 3
    *s = sinL*cosM + sinM*cosL;
    *c = cosL*cosM - sinL*sinM;
}

// use only the MSB table. This is a bit faster, but not worth the
// substantial decrease in accuracy.
static inline void 
bot_fasttrig_sincos_coarse(double theta, double *s, double *c)
{
    uint32_t idx = (K_MSB_TABLE_SIZE / (2*M_PI)) * theta;

    int msb_idx = (idx & (K_MSB_TABLE_SIZE - 1));
    *s = msb_table[msb_idx].m_sin;
    *c = msb_table[msb_idx].m_cos;
}

#define ATAN2_LUT_SIZE 64
static int atan2_lut_initted = 0;
static double atan2_lut[ATAN2_LUT_SIZE+2];

// do binlinear filtering on LUT? (LUT can be way smaller!)
#define ATAN2_FASTER 0

// accurate to about 0.0012 degrees with ATAN2_FASTER = 0 and LUT_SIZE==64
// returns mod2pi answer
double bot_fasttrig_atan2(double y, double x)
{
    if (!atan2_lut_initted) {
        for (int i = 0; i < ATAN2_LUT_SIZE; i++) {
            double v = ((double) i) / ATAN2_LUT_SIZE;
            atan2_lut[i] = atan(v);
            assert(atan2_lut[i] >= 0);
        }
        atan2_lut[ATAN2_LUT_SIZE] = M_PI/4;
        atan2_lut[ATAN2_LUT_SIZE+1] = M_PI/4;
        atan2_lut_initted = 1;
    }

    // basic idea: atan is well-behaved over first 45 degrees, so we
    // do a reduction of all atan2 operations to the first 45 degrees
    // by possibly swapping the x and y axes.  We've precomputed a
    // lookup table over this range, and we bilinearly interpolate the
    // answer, then do the necessary twiddling to get back the right
    // answer.
    double yabs = fabs(y);
    double xabs = fabs(x);

    if (xabs >= yabs) {
        double S1 = (y >= 0) ? 1 : -1;
        double A = (x < 0) ? M_PI : 0;
        double S2 = (x*y >= 0) ? 1 : -1;

        double didx = ATAN2_LUT_SIZE * yabs / xabs;

        double rho;
        int idx = (int) didx;
        if (idx >= ATAN2_LUT_SIZE || idx < 0) {
            printf("ATAN2: called with %f, %f\n", x, y);
            return 0;
        }

        if (ATAN2_FASTER)
            rho = atan2_lut[idx];
        else {
            double drem = didx - idx;
            rho = (1-drem)*atan2_lut[idx] + drem*atan2_lut[idx+1];
        }

        return (S1*A + S2*rho);
    } else {
        double S1A = (y>0) ? M_PI/2 : -M_PI/2;
        double S2 = (x*y >= 0) ? -1 : 1;

        double didx = ATAN2_LUT_SIZE * xabs / yabs;
        double rho;

        int idx = (int) didx;
        if (idx >= ATAN2_LUT_SIZE || idx < 0) {
            printf("ATAN2: called with %f, %f\n", x, y);
            return 0;
        }

        if (ATAN2_FASTER)
            rho = atan2_lut[idx];
        else {
            double drem = didx - idx;
            rho = (1-drem)*atan2_lut[idx] + drem*atan2_lut[idx+1];
        }

        return (S1A + S2*rho);
    }
}

// good to 4.1 degrees, returns mod2pi answer.
static inline double 
bot_fasttrig_atan2_coarse(double y, double x) 
{
	double abs_y = fabs(y);
	double angle;

	if (x >= 0) {
		double r = (x - abs_y) / (x + abs_y);
		angle = M_PI/4.0 * (1 - r);
	} else {
		double r = (x + abs_y) / (abs_y - x);
		angle = M_PI/4.0 * ( 3 - r );
	}
	return y < 0 ? -angle : angle;
}

static inline void 
bot_fasttrig_sincos_test()
{
    bot_fasttrig_init();

    double eps = 1.0/(1<<17);

//    int limit = 30000000;
    int limit = INT32_MAX;

    for (int iters = 0; iters < limit; iters++) {
        double theta = rand()/ 1000.0;
        double s = sin(theta),  c = cos(theta);
        double s2, c2;
        bot_fasttrig_sincos(theta, &s2, &c2);

        if (1) {
            double s_err = fabs(s-s2), c_err = fabs(c-c2);
            if (s_err > eps)
                printf("sin %20.5f : %15.12f %15.12f %15.12f %10.5f\n", theta, s, s2, s_err, s_err/eps);
            if (c_err > eps)
                printf("cos %20.5f : %15.12f %15.12f %15.12f %10.5f\n", theta, c, c2, c_err, c_err/eps);
        }
    }
    exit(0);
}

static inline float randf()
{
    return ((float) rand()) / (RAND_MAX + 1.0);
}
static inline float signed_randf()
{
    return randf()*2 - 1;
}
#define to_degrees(x) ( (x) * (180.0 / M_PI ))

static inline void 
bot_fasttrig_atan2_test()
{
    double max_err = 0;

    int64_t start = _timestamp_now();

    for (int i = 0; i < 10000000; i++) {
        double x = signed_randf();
        double y = signed_randf();
        
        double a = atan2(y,x);
        double b = bot_fasttrig_atan2(y,x);

        double err = fabs(a-b);
        if (err > max_err) {
            printf("max err: %15f deg (%15f %15f %15f %15f)\n", to_degrees(err), x, y, a, b);
            max_err = err;
        }
    }
    int64_t stop = _timestamp_now();

    printf("dt: %15f\n", (stop-start)/1000000.0);
}

/*
int main(int argc, char *argv[])
{
    bot_fasttrig_atan2_test();
}
*/
