#pragma once


// Simulation window refresh rate (in seconds)
const double    SCREEN_TIMEOUT = 0.10;


// Diameter size of the robot (in pixels)
const double    ROBOT_SIZE   = 20;
const double    ROBOT_SIZE_2 = ROBOT_SIZE/2;


// A grid area for probability of occupancy
const int       GRID_MAP_H = 500;
const int       GRID_MAP_W = 500;


// Laser sensor properties
const double    LIDAR_VAR_2       = 10;                // Standard deviation
const double    LIDAR_VAR         = sqrt(LIDAR_VAR_2); // Variance
const double    LIDAR_START_ANGLE = -M_PI_2;           // Starting angle of the first line segment (ray)
const double    LIDAR_SWEEP_ANGLE = M_PI;              // Angle of area swept by the LIDAR
const int       LIDAR_RAYS        = 100;               // Number of line segments (rays)
const double    LIDAR_MAX         = 200.0;             // Maximum distance


// Position based odometry
const int       ODOM_SAMPLES = 1000;
const double    KT           = 0.1;  // Translation error parameter
const double    KD           = 0.01; // Drift error parameter
const double    KR           = 1.0;  // Rotation error parameter
const double    MT           = 0.0;  // Translation error mean
const double    MD           = 0.0;  // Drift error mean
const double    MR           = 0.0;  // Rotation error mean

// Time delay for simulation,
//   accessed from menu bar: Simulation >> Speed >> Slow/Medium/Fast
#define SLOW     200
#define MEDIUM   50
#define FAST     20

