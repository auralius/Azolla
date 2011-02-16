//
// auralius (auralius@gmx.com)
//

#include "simulation_window.H"
#include "sensor.H"
#include <boost/thread.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <vector>
#include <iostream>
#include <fstream>

class CAzolla
{
public:
    typedef void (*FUN)(CAzolla *);

    // Process
    CAzolla(CSimulationWindow *w);
    ~CAzolla(void);
    void az_step();
    void az_update_odometer();
    void az_log_local_map(const char *fn);
    void az_log_global_map(const char *fn);
    void az_log_odom(const char *fn);
    void az_log_sensor(const char *fn);

    // Set
    void az_set_simulation_fn(FUN fun);
    void az_set_location(double x, double y, double th);
    void az_set_lspeed(double s);
    void az_set_rspeed(double s);
    void az_set_time_step(double t);

    // Get
    double az_get_lspeed();
    double az_get_rspeed();
    double az_get_pos_x();
    double az_get_pos_y();
    double az_get_angle();
    double az_get_time_step();
    double az_get_sensor(int i);
    unsigned char *az_get_local_map();
    unsigned char *az_get_global_map1();
    unsigned char *az_get_global_map2();
    void az_get_odometer_data(CPose &p);

    // Calculation
    int az_calc_index(int x, int y);

private:
    static void simulation_thread(void* param);
    void simulation_thread_worker();
    static void odometry_thread(void* param);
    void odometry_thread_worker();
    inline void lidar_sensor();
    inline void sonar_sensor();
    inline void init_local_map(unsigned char v);
    inline void init_global_map(unsigned char v);
    inline void init_odom_data();
    inline void update_global_map(int x, int y, unsigned char occ);
    inline void calc_occ();

    // Provide acces to simulation window
    CSimulationWindow *m_window;
    // Left wheel angular speed
    double m_speed_l;
    // Right wheel angular speed
    double m_speed_r;
    // Robot current pose
    CPose m_pose;
    // Robot previous pose
    CPose m_pose0;
    // Storage for noisy odometry data for a certain sampling number
    CPose m_odom[ODOM_SAMPLES];
    // Simulation time step
    double m_time_step;
    // Flag to see if this class has been loaded
    bool m_global_flag;
    // First time iteration of the simulation loop
    bool m_first_time_flag;
    // Simulation callback function sent as parameter
    FUN m_fun;
    // Sensor reading data
    CSensor m_sensor_data[LIDAR_RAYS];
    // Local map contains current occupancy reading
    unsigned char m_local_map[GRID_MAP_W * GRID_MAP_H];
    // Robot memory containg overall grid occupancy data
    unsigned char m_global_map[GRID_MAP_W * GRID_MAP_H];
    // Global map update type
    int m_map_update_type;
};

// TODO : Robot geometry => robot can have shape
