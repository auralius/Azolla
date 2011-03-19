/**
*  @file   simulation_window.H
*  @brief  Canvas where eveything is drawn
*  @author Auralius Manurung <auralius@gmx.com>
*  @date   2/19/2011
*/

#ifndef CANVAS_H_
#define CANVAS_H_

#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/fl_draw.H>

#include "configure.H"
#include "point2d.H"
#include "pose.H"
#include "sensor.H"


class CCanvas:public Fl_Box
{
public:
    CCanvas(int x, int y, int w,int h,const char *l);

    /**
     * Set drawing mode to
     * 1: Draw sensor strike
     * 2: Draw grid occupancy probability
     * @param m Mode
     */
    void set_drawing_mode(int m);

    /**
     * Set pointer m_pose to point to m_pose in CAzolla
     * @param p Pointer to m_pose in CAzolla
     */
    void set_addr_robot_pose(CPose *p);

    /**
     * Set pointer m_sensor_data to point to m_sensor_data in CAzolla
     * @param p Pointer to m_sensor_data in CAzolla
     */
    void set_addr_sensor_data(CSensor *p);

    /**
     * Set pointer m_grid_map to point to m_global_map in CAzolla
     * @param p Pointer to m_global_map in CAzolla
     */
    void set_addr_grid_data(unsigned char *p);

    /**
     * Set pointer m_particles to point to m_particles in CAzolla
     * @param p Pointer to m_particles in CAzolla
     */
    void set_addr_particle_data(CPose *p);

    /**
     * Notify if simulation is running, this is done by CSimulationWindow
     * @param s Flag for simulation
     */
    void notify_simulation_status(bool s);

private:
    /// Handle mouse click event to change robot position
    int handle(int e) ;

    /// Callback function when redraw() is called
    void draw();

    /// Timer callback, used to call redraw()
    static void timeout_cb(void *userdata);

    /// Draw the robot
    void draw_robot();

    /// Draw sensor hitmark and grid occupancy
    void draw_sensor_beam();

    /// Draw robot position sensed by odometer
    void draw_odom();

    /// We have 2 drawing mode: sensor strike and grid memory
    int m_drawing_mode;

    /// If simulation is running, mouse click event should be disabled
    bool m_simulation_running;

    /// These data are going to be drawn
    /// Robot parameters: x, y and angular position, also incluiding sensor,
    /// grid occupancy map and noisy odometry data. All data are pointers
    /// which point to the real data at class CAzolla (sharing same data).
    CPose           *m_pose;
    CSensor         *m_sensor_data;
    unsigned char   *m_grid_map;
    CPose           *m_particles;
};

#endif // CANVAS_H_
