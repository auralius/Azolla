#ifndef AZ_CONFIG_H_
#define AZ_CONFIG_H_

/**
 *  @file   config_struct.H
 *  @brief  Contains data structure for robot parameters
 *  @author Auralius Manurung <auralius@gmx.com>
 *  @date   3/24/2011
 */

struct AZ_CONFIG
{
    /// Scale, 1 meter equals to how many pixels
    double SCALE_FACTOR;

    /// Simulation window refresh rate (seconds)
    double SCREEN_TIMEOUT;

    /// Robot diameter (meters)
    double ROBOT_DIAMETER;

    /// Robot radius (meters)
    double ROBOT_RADIUS;

    /// Grid map height
    int GRID_MAP_H;

    /// Grid map width
    int GRID_MAP_W;

    /// Variance
    double LIDAR_STDEV;

    /// Starting angle of the first ray
    double LIDAR_START_ANGLE;

    /// Angle of the area swept by LIDAR
    double LIDAR_SWEEP_ANGLE;

    /// Number of rays
    int LIDAR_RAYS ;

    /// Maximum distance that sensor can measure
    double LIDAR_MAX;

    /// Choose one sample to represent robot position with odometry error
    int CHOSEN_SAMPLE;

    /// Number of samples to representate possible robot position
    int ODOM_SAMPLES;

    /// Translational error
    double KT;

    /// Drift error
    double KD;

    /// Rotational error
    double KR;

    /// Translational error mean
    double MT;

    /// Drift error mean
    double MD;

    /// Rotational error mean
    double MR;
};

#endif // CANVAS_H_
