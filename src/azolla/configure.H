/**
*  @file   simulation_window.H
*  @brief  Contains all robot and simulation configurations
*  @author Auralius Manurung <auralius@gmx.com>
*  @date   2/19/2011
*/

#ifndef CONFIGURE_H_
#define CONFIGURE_H_

#include <math.h>


/// Scale, 1 meter equals to how many pixels?
const double    SCALE_FACTOR = 192.0;


/// Simulation window refresh rate (in seconds)
const double    SCREEN_TIMEOUT = 0.10;


/// Diameter size of the robot (in meters)
const double    ROBOT_SIZE   = 0.1 * SCALE_FACTOR;
const double    ROBOT_SIZE_2 = ROBOT_SIZE/2;


/// A grid area for probability of occupancy
const int       GRID_MAP_H = 500;
const int       GRID_MAP_W = 500;


/// Laser sensor properties
const double    LIDAR_VAR_2       = 0.1 * SCALE_FACTOR;   // Standard deviation (meters)
const double    LIDAR_VAR         = sqrt(LIDAR_VAR_2);    // Variance
const double    LIDAR_START_ANGLE = -M_PI_2;              // Starting angle of the first ray
const double    LIDAR_SWEEP_ANGLE = M_PI;                 // Angle of area swept by the LIDAR
const int       LIDAR_RAYS        = 100;                  // Number of rays
const double    LIDAR_MAX         = 1.0 * SCALE_FACTOR;   // Maximum distance (meters)


/// Position based odometry
const int       CHOSEN_PARTICLE = 0;                    // Select one particle
const int       ODOM_SAMPLES    = 1000;
const double    KT              = 0.10 / SCALE_FACTOR;  // Translation error parameter
const double    KD              = 0.10 / SCALE_FACTOR;  // Drift error parameter
const double    KR              = 0.08 / SCALE_FACTOR;  // Rotation error parameter
const double    MT              = 0.0;                  // Translation error mean
const double    MD              = 0.0;                  // Drift error mean
const double    MR              = 0.0;                  // Rotation error mean

/// Time delay for simulation,
/// accessed from menu bar: Simulation >> Speed >> Slow/Medium/Fast
#define SLOW     200
#define MEDIUM   50
#define FAST     20

#endif // CONFIGURE_H_
