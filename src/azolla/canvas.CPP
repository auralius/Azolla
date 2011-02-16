#include "canvas.H"

CCanvas::CCanvas(int x, int y, int w,int h,const char *l)
    :Fl_Box(x, y, w, h, l)
{
    box(FL_FLAT_BOX);
    color(FL_WHITE);
    m_drawing_mode = 1;

    // Canvas refresh reate
    Fl::add_timeout(SCREEN_TIMEOUT, timeout_cb, (void*)this);
}

//
// SET
//

void CCanvas::set_drawing_mode(int m)
{
    m_drawing_mode = m;
}

void CCanvas::set_robot_pose(CPose &p)
{
    m_pose = p;
}

void CCanvas::set_addr_sensor_data(CSensor *p)
{
    m_sensor_data = p;
}

void CCanvas::set_addr_grid_data(unsigned char *p)
{
    m_grid_map = p;
}

void CCanvas::set_addr_odom_data(CPose *p)
{
    m_odom = p;
}

//
//
//

void CCanvas::draw()
{
    Fl_Box::draw();
    draw_sensor_beam();
    draw_odom();
    draw_robot();
}

void CCanvas::draw_robot()
{
    // Draw a robot as a circle with a line
    fl_color(FL_BLUE);
    CPoint2D p0(m_pose.x, m_pose.y);
    CPoint2D p1(p0.translate(ROBOT_SIZE_2, m_pose.th));
    fl_line(m_pose.x, m_pose.y, p1.x(), p1.y());
    fl_circle(m_pose.x, m_pose.y, ROBOT_SIZE_2);
}

void CCanvas::draw_sensor_beam()
{
    const int drawing_offset = 3;

    // Sensor hit mark
    if (m_drawing_mode == 1)
    {
        fl_color(FL_RED);
        for (int i = 0; i < LIDAR_RAYS; i = i + 1)
            fl_circle(m_sensor_data[i].get_hit_point().x(), m_sensor_data[i].get_hit_point().y()+3, 1.0);
    }

    // Occupancy grid
    // Draw only grid with OCC != 127, OCC = 127 means unknown
    else if (m_drawing_mode == 2)
    {
        for (int y = 0; y < h(); y = y + 1) {
            for (int x = 0; x < w(); x = x + 1) {
                if (m_grid_map[x + y * GRID_MAP_W] != 127) {
                    unsigned char c = 255 - m_grid_map[x + y * GRID_MAP_W];
                    fl_color(c, c, c);
                    fl_point(x,y);
                } //end if
            } // end for x
        } // end for y
    } // end else if
}

void CCanvas::draw_odom()
{
    for (int i = 0; i < ODOM_SAMPLES; i ++) {
        fl_color(FL_MAGENTA);
        fl_point((int) m_odom[i].x, (int) m_odom[i].y);
    }
}

void CCanvas::timeout_cb(void *userdata)
{
    CCanvas *o = (CCanvas*)userdata;
    o->redraw();
    Fl::repeat_timeout(SCREEN_TIMEOUT, timeout_cb, userdata);
}
