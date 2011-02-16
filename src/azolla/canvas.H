//
// auralius (aurlaius@gmx.com)
//

#pragma once

#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/fl_draw.H>
#include <FL/math.h>

#include "configure.H"
#include "point2d.H"
#include "sensor.H"
#include <string.h>

class CCanvas:public Fl_Box
{
public:
    CCanvas(int x, int y, int w,int h,const char *l);
    void set_drawing_mode(int m);
    void set_robot_pose(double x, double y, double th);
    void set_addr_sensor_data(CSensor *p);
    void set_addr_grid_data(unsigned char *p);
    void set_addr_odom_data(double *x, double *y);

private:
    void draw();
    static void timeout_cb(void *userdata);
    void draw_robot();
    void draw_sensor_beam();
    void draw_odom();

    // We have 2 drawing mode: sensor strike and grid memory
    int m_drawing_mode;


    // These data are going to be drawn
    // Robot parameters: x, y and angular position,
    // also incluiding sensor and grid occupancy map.
    // Sensor and grid occupancy are pointers which point to
    // the real data at class CAzolla.
    CPoint2D m_pos;
    double m_theta;
    CSensor *m_sensor_data;
    unsigned char *m_grid_map;

    // We are also going to draw noisy odometry data
    double *m_odom_x;
    double *m_odom_y;
};
