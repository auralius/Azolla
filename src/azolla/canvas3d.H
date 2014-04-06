/**
 *  @file   canvas3d.H
 *  @brief  Contains class for drawing in 3D
 *  @author Auralius Manurung <auralius@gmx.com>
 *  @date   6/5/2011
 */

#ifndef CANVAS3D_H_
#define CANVAS3D_H_

#include <FL/Fl.H>
#include <FL/gl.h>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Shared_Image.H>
#include <FL/Fl_PNG_Image.H>

#include <GL/glu.h>
#include <GL/glpng.h>
#include "../ext/zpr.h"

#include "configure.H"
#include "point2d.H"
#include "pose.H"
#include "sensor.H"


class CCanvas3D : public Fl_Gl_Window
{
public:
    CCanvas3D(int x,int y,int w,int h,const char *l = 0);

    void set_drawing_mode(int m);

    /**
     * Set pointer m_pose to point to m_pose in CAzolla.
     * @param p Pointer to m_pose in CAzolla
     */
    void set_addr_robot_pose(CPose *p);

    /**
     * Set pointer m_sensor_data to point to m_sensor_data in CAzolla.
     * @param p Pointer to m_sensor_data in CAzolla
     */
    void set_addr_sensor_data(CSensor *p);

    /**
     * Set pointer m_grid_map to point to m_global_map in CAzolla.
     * @param p Pointer to m_global_map in CAzolla
     */
    void set_addr_grid_data(unsigned char *p);

    /**
     * Set pointer m_particles to point to m_particles in CAzolla.
     * @param p Pointer to m_particles in CAzolla
     */
    void set_addr_particle_data(CPose *p);

	/**
     * Set pointer m_cfg to point to m_cfg in CAzolla.
     * @param p Pointer to m_cfg in CAzolla
     */
	void set_cfg_data(const AZ_CONFIG *cfg);

    /**
     * Calculate and set scale factor.
     * In OpenGL, the axis value is from 0 (min) to 1 (max).
     * We need to scale down the map.
     * @param f File name
     */
    void set_map_scale_factor();

    /**
     * Notify if simulation is running, this is done by CSimulationWindow.
     * @param s Flag for simulation
     */
    void notify_simulation_status(bool s);

    /**
     * Load a PNG file for map as a texture using OpenGL.
     * @param f File name
     */
    void load_map_as_texture(const char *f);

private:
    /// Handle mouse event (for zoom, pane and rotate)
    int handle(int e) ;

    /// Timer callback, used to call redraw().
    static void timeout_cb(void *userdata);

    /// Callback function when redraw() is called.
    void draw();

    /// Draw x and y axis (floor plane)
    void draw_axis();

    /// Draw cirlce using OpenGL
    void draw_circle(double x, double y, double radius, int n_points);

    /// Draw the robot.
    void draw_robot();

    /// Draw sensor hitmark and grid occupancy
    void draw_sensor_beam();

    /// Draw robot position sensed by odometer.
    void draw_odom();

    /// Draw floor where map is rendered as texture
    void draw_floor();

    /// Flag to check if map file is already loaded
    bool m_map_file_loaded;

    /// Flag for first time indicator
    bool m_first_time;

    /// Map width
    double m_map_w;

    /// Map height
    double m_map_h;

    /// In OpenGL, canvas is mapped from 0 to 1, so we need scale factor here.
    double m_scale_factor;

    /// We have 2 drawing mode: sensor strike and grid memory.
    int m_drawing_mode;

    /// If simulation is running, mouse click event should be disabled.
    bool m_simulation_running;

    /// Last position when mouse left key is down,
    CPoint2D m_last_click;

     /// Robot, map, and simulation configuration.
    const AZ_CONFIG	*m_cfg;

	/**
     * Below are the data needed for drawing purpose
     * Robot current position (x, y and angular position), also incluiding
	 * sensor,grid occupancy map and noisy odometry data. All data are
	 * pointers which point to the real data at class CAzolla
	 * (sharing same data).
	 */
    CPose *m_pose;
    CSensor *m_sensor_data;
    unsigned char *m_grid_map;
    CPose *m_particles;
};

#endif
