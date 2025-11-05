#include "stm32h7xx_hal.h"
#include "Control.h"
#include <stdio.h>
#include <math.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

float calc_angle(float target_long, float target_lat, float current_long, float current_lat) {
    // Convert to radians
    float lat1 = current_lat * DEG_TO_RAD;
    float lon1 = current_long * DEG_TO_RAD;
    float lat2 = target_lat * DEG_TO_RAD;
    float lon2 = target_long * DEG_TO_RAD;

    // Compute difference in longitude
    float dLon = lon2 - lon1;

    // Compute bearing
    float x = sin(dLon) * cos(lat2);
    float y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    float bearing = atan2(x, y); // result in radians
    float bearing_deg = fmod((bearing * RAD_TO_DEG + 360.0), 360.0); // convert to 0–360°

    return bearing_deg;
}

/**
 * @brief Normalizes an angle to the range [-180, 180].
 * @param angle The angle in degrees.
 * @return The normalized angle.
 */
float normalize_angle(float angle) {
    angle = fmod(angle, 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle <= -180.0) {
        angle += 360.0;
    }
    return angle;
}


