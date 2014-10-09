#ifndef __bot_gps_linearize_h__
#define __bot_gps_linearize_h__

/**
 * @defgroup BotCoreGPSLinearize GPS Linearization
 * @ingroup BotCoreMathGeom
 * @brief Linearizing GPS coordinates
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _BotGPSLinearize BotGPSLinearize;
struct _BotGPSLinearize
{
    double lon0_deg, lat0_deg;
    double radius_ns, radius_ew;
};

void bot_gps_linearize_init(BotGPSLinearize *gl, const double ll_deg[2]);
int bot_gps_linearize_to_xy(BotGPSLinearize *gl, const double ll_deg[2], double xy[2]);
int bot_gps_linearize_to_lat_lon(BotGPSLinearize *gl, const double xy[2], double ll_deg[2]);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
