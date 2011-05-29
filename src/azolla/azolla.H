/**
 *  @file   azolla.H
 *  @brief  Contains all available commands to the robot.
 *  @author Auralius Manurung <auralius@gmx.com>
 *  @date   2/19/2011
 */

#ifndef AZOLLA_H_
#define AZOOLA_H_

#include "configure.H"
#include "simulation_window.H"

#include <boost/thread.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <iostream>
#include <fstream>

class CAzolla
{
public:

    // Process

    /**
     * Constructor
     */
    CAzolla(CSimulationWindow *w);

	/**
     * Destructor
     */
    ~CAzolla(void);

    /**
     * Read configuration file.
     * @param fn Configuration file name
     * @return 0 if it is successfull, else return -1
     */
    int az_read_config_file(const char *fn);

    /**
     * Update robot pose and sensor reading.
     */
    virtual void az_step();

    /**
     * Simulation function defined by user.
     * This function will be excecuted continously while the
     * simulation is running.
     */
    virtual void az_sim_fn() = 0;

    /**
     * Acquire data from odometer.
     */
    void az_update_odometry();

    /**
     * Write local map data to disk.
     * @param fn  File name
     */
    void az_log_local_map(const char *fn);

    /**
     * Write global map data to disk.
     * @param fn  File name
     */
    void az_log_global_map(const char *fn);

    /**
     * Write odometry data to disk.
     * @param fn  File name
     */
    void az_log_odom(const char *fn);

    /**
     * Write sensor data to disk.
     * @param fn  File name
     */
    void az_log_sensor(const char *fn);

    // Set

    /**
     * Set robot location and angular position.
     * @param x Position in x coordinate
     * @param y Position in y coordinate
     * @param th Angular position
     */
    void az_set_location(double x, double y, double th);

    /**
     * Set robot left wheel speed.
     * @param s Desired left wheel speed
     */
    void az_set_lspeed(double s);

    /**
     * Set robot right wheel speed.
     * @param s Desired right wheel speed
     */
    void az_set_rspeed(double s);

    /**
     * Set both wheel's speed to zero (stop the robot).
     */
    void az_set_stop();

    /**
     * Set time step for simulaiton.
     * @param t Desired time step
     */
    void az_set_time_step(double t);

     /**
     * Set address of particle data to be displayed on simulation window.
     * @param p Data address
     */
    void az_set_addr_particle_data(CPose *p);

    // Get

    /**
     * Get measured distance by i-th laser.
     * @param i Index
     * @return Measured distance
     */
    double az_get_sensor_data(double th);

    /**
     * Get left wheel speef.
     * @return Left wheel speed
     */
    double az_get_lspeed();

    /**
     * Get right wheel speef.
     * @return Right wheel speed
     */
    double az_get_rspeed();

    /**
     * Get robot position in x axis.
     * @return Robot position in x
     */
    double az_get_pos_x();

    /**
     * Get robot previous position in x axis.
     * @return Robot position in x
     */
    double az_get_pos_x_prev();

    /**
     * Get robot position in y axis.
     * @return Robot position in y
     */
    double az_get_pos_y();

    /**
     * Get robot previous position in y axis.
     * @return Robot position in y
     */
    double az_get_pos_y_prev();

    /**
     * Get robot angular position (radians).
     * @return Robot angular position
     */
    double az_get_angle();

    /**
     * Get simulation time step.
     * @return Simulation time step
     */
    double az_get_time_step();

    /**
     * Get sensor reading.
     * param i Index of sensor
     * @return Sensor reading
     */
    double az_get_sensor(int i);

    /**
     * Get local map, map is represented as occupancy grid.
     * To create local map. the algorithm reads all sensor measurements.
     * @return Local map as an array of char
     */
    unsigned char *az_get_local_map(bool with_error);

    /**
     * Get global map, which describes the environment in occupancy grid.
     * To create global map, local map data collected over time are combined.
     * In type 1, if multiple updates happen, the cell will accept any changes.
     * @param with_error Use error model on odometry
     * @return Local map as an array of char
     */
    unsigned char *az_get_global_map1(bool with_error);

    /**
     * Get global map, which describes the environment in occupancy grid.
     * To create global map, local map data collected over time are combined.
     * In type 2, if multiple updates happen, the cell will multiply its
     * current value with the new value.
     * @return Local map as an array of char
     */
    unsigned char *az_get_global_map2(bool with_error);

    /**
     * Get all readings by position odometry.
     * These readings describe all possible robot position because
     * of accumulated noise in odometry sensor.
     * @return Odometry data
     */
     CPose *az_get_odometry_data();

    /**
     * Get simulation step numbers.
     * @return Step numbers
     */
     int az_get_step_number();

    // Calculation

    /**
     * We are using pseudo-multi-dimension array in which we use 1D array
     * to representate 2D array. The index can be computed as:
     * array2d[x][y] = array1d[x + y * array_width].
     * @return 1D array index
     */
    int az_calc_index(int x, int y);

private:
    /// Prepare everything needed for simulation.
    void init_sim();

    /// Static function to call simulation_thread_worker.
    static void simulation_thread(void* param);

    /// A thread to handle simulation, this thread executes az_sim_fn.
    void simulation_thread_worker();

    /// Static function to odometry_thread_worker
    static void odometry_thread(void* param);

    /// Static function to odometry_thread_worker.
    void odometry_thread_worker();

    /// Function for laser range sensor.
    void lidar_sensor();

    /// Function for sonar range sensor.
    void sonar_sensor();

    /// Compute occupancy probabilty based on distance sensor reading.
    void calc_occupancy(bool with_error);

    /// Initialize local map with a certain occupancy probability.
    inline void init_local_map(unsigned char v);

    /// Initialize global map with a certain occupancy probability.
    inline void init_global_map(unsigned char v);

    /// Initialize all of the odometry data (all of the the particles).
    inline void init_odom_data();

    /// Update a cell in the global map with a certain occupancy probability.
    inline void update_global_map(int x, int y, unsigned char occ);

    /// Provide acces to simulation window.
    CSimulationWindow *m_window;

    /// Left wheel angular speed.
    double m_speed_l;

    /// Right wheel angular speed.
    double m_speed_r;

    /// Robot current pose.
    CPose m_pose;

    /// Robot previous pose.
    CPose m_pose0;

    /// Storage for noisy odometry data (particles).
    CPose *m_particles;

    /// Simulation time step.
    double m_time_step;

    /// Number of simulation steps that already happened so far.
    int m_step_num;

    /// Flag to see if this class has been loaded.
    bool m_global_flag;

    /// First time iteration of the simulation loop.
    bool m_first_time_flag;

    /// Sensor reading data.
    CSensor *m_sensor_data;

    /// Local map contains current occupancy reading.
    unsigned char *m_local_map;

    /// Robot memory containg overall grid occupancy data.
    unsigned char *m_global_map;

    /// Global map update type.
    int m_map_update_type;

    /// Robot, map, and simulation configuration.
    AZ_CONFIG m_cfg;
};

#endif // AZOLLA_H_
