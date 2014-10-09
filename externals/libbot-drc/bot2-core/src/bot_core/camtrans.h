#ifndef __BOT_CAMTRANS_H__
#define __BOT_CAMTRANS_H__

/**
 * @defgroup BotCoreCamTrans CamTrans
 * @ingroup BotCoreMathGeom
 * @brief Perspective camera projection and distortion models
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct _BotCamTrans BotCamTrans;

    typedef int (*undist_func_t)(const void*, const double, const double,
                                 double[3]);

    typedef int (*dist_func_t)(const void*, const double[3],
                               double*, double*);

    typedef void (*destroy_func_t)(void *);

    typedef struct {
        undist_func_t undist_func;
        dist_func_t dist_func;
        destroy_func_t destroy_func;
    } BotDistortionFuncs;

    typedef struct {
        void *params;
        BotDistortionFuncs *funcs;
    } BotDistortionObj;

    /**
     * bot_spherical_distortion_create:
     * @a: TODO
     *
     * TODO description
     *
     * Returns: TODO
     */
    BotDistortionObj*
    bot_spherical_distortion_create (const double a);

    /**
     * bot_angular_lookup_distortion_create:
     * @num_dist: TODO
     * @dist_vals: TODO
     * @dist_step: TODO
     * @num_undist: TODO
     * @undist_vals: TODO
     * @undist_step: TODO
     *
     * TODO description
     *
     * Returns: TODO
     */
    BotDistortionObj*
    bot_angular_lookup_distortion_create (const int num_dist,
                                          const double* dist_vals,
                                          const double dist_step,
                                          const int num_undist,
                                          const double* undist_vals,
                                          const double undist_step);

    /**
     * bot_angular_poly_distortion_create:
     * @coeffs: array of odd polynomial coefficients
     * @num_coeffs: number of coefficients in array
     *
     * model is:
     *  theta_undist = theta_dist + c0*theta_dist^3 + c1*theta_dist^5 + ...
     *  where theta is the angle between the principal ray and the input ray
     *
     * Returns: distortion model object for use in camtrans
     */
    BotDistortionObj*
    bot_angular_poly_distortion_create (const double* coeffs,
                                        const int num_coeffs);

    /**
     * bot_null_distortion_create:
     * 
     * TODO description
     *
     * Returns: TODO
     */
    BotDistortionObj*
    bot_null_distortion_create(void);
    

    /**
     * bot_plumb_bob_distortion_create:
     * @k1: TODO
     * @k2: TODO
     * @k3: TODO
     * @p1: TODO
     * @p2: TODO
     * 
     * TODO description
     *
     * Returns: TODO
     */
    BotDistortionObj*
    bot_plumb_bob_distortion_create (const double k1, const double k2,
                                     const double k3,const double p1,
                                     const double p2);

    /**
     * bot_camtrans_new:
     * @name: TODO
     * @width: TODO
     * @height: TODO
     * @fx: TODO
     * @fy: TODO
     * @cx: TODO
     * @cy: TODO
     * @skew: TODO
     * @distortion_obj: TODO
     *
     * TODO description
     *
     * Returns: TODO
     */
    BotCamTrans* bot_camtrans_new (const char *name,
                                   double width, double height,
                                   double fx, double fy,
                                   double cx, double cy, double skew,
                                   BotDistortionObj * distortion_obj);

    /**
     * bot_camtrans_destroy:
     *
     * TODO description
     *
     * Returns: TODO
     */
    void bot_camtrans_destroy (BotCamTrans *t);

    double bot_camtrans_get_focal_length_x (const BotCamTrans *t);
    double bot_camtrans_get_focal_length_y (const BotCamTrans *t);
    double bot_camtrans_get_image_width (const BotCamTrans *t);
    double bot_camtrans_get_image_height (const BotCamTrans *t);
    double bot_camtrans_get_principal_x (const BotCamTrans *t);
    double bot_camtrans_get_principal_y (const BotCamTrans *t);
    double bot_camtrans_get_width (const BotCamTrans *t);
    double bot_camtrans_get_height (const BotCamTrans *t);
    double bot_camtrans_get_skew (const BotCamTrans *t);
    const char *bot_camtrans_get_name(const BotCamTrans *t);

    /**
     * bot_camtrans_project_point:
     * @self: TODO
     * @p: TODO
     * @im_xyz: TODO
     *
     * TODO description
     *
     * Returns: TODO
     */
    int bot_camtrans_project_point(const BotCamTrans *self, const double p[3],
                                   double im_xyz[3]);

    /**
     * bot_camtrans_unproject_pixel:
     * @self: TODO
     * @im_x: TODO
     * @im_y: TODO
     * @ray: TODO
     *
     * TODO description
     *
     * Returns: TODO
     */
    int bot_camtrans_unproject_pixel(const BotCamTrans *self, double im_x,
                                     double im_y, double ray[3]);

    /**
     * bot_camtrans_scale_image:
     * @self: TODO
     * @scale_factor: TODO
     *
     * TODO description
     *
     * Returns: TODO
     */
    void bot_camtrans_scale_image (BotCamTrans *self, const double scale_factor);

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
