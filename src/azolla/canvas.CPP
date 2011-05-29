#include "canvas.H"

CCanvas::CCanvas(int x, int y, int w,int h,const char *l)
    :Fl_Box(x, y, w, h, l)
{
    box(FL_FLAT_BOX);
    color(fl_rgb_color(127,127,127));

    m_drawing_mode = 1;
    m_simulation_running = false;
}

int CCanvas::handle(int e)
{
    if ( e == FL_PUSH && m_simulation_running == false) {
        m_pose->set_x(Fl::event_x());
        m_pose->set_y(Fl::event_y());
    }
    return(Fl_Box::handle(e));
}

//
// SET
//

void CCanvas::set_drawing_mode(int m)
{
    m_drawing_mode = m;
}

void CCanvas::set_addr_robot_pose(CPose *p)
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

void CCanvas::set_addr_particle_data(CPose *p)
{
    m_particles = p;
}

void CCanvas::set_cfg_data(const AZ_CONFIG *cfg)
{
	m_cfg = cfg;

	// Canvas refresh reate
    Fl::add_timeout(m_cfg->SCREEN_TIMEOUT, timeout_cb, (void*)this);
}

void CCanvas::notify_simulation_status(bool s)
{
    m_simulation_running = s;
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
    fl_color(FL_BLACK);
    CPoint2D p0(m_pose->x(), m_pose->y());
    CPoint2D p1(p0.translate(m_cfg->ROBOT_RADIUS, m_pose->th()));
    fl_line(m_pose->x(), m_pose->y(), p1.x(), p1.y());
    fl_circle(m_pose->x(), m_pose->y(), m_cfg->ROBOT_RADIUS);
}

void CCanvas::draw_sensor_beam()
{
    const int drawing_offset = 4; // Offset happens on y, don't know why??

    // Sensor hit mark
    if (m_drawing_mode == 1) {
        fl_color(FL_RED);
        for (int i = 0; i < m_cfg->LIDAR_RAYS; i = i + 1)
            fl_circle(m_sensor_data[i].get_hit_point().x(),
                      m_sensor_data[i].get_hit_point().y() + drawing_offset,
                      1.0); // Size of sensor hit mark
    }

    // Occupancy grid
    // Draw only grid with OCC != 127, OCC = 127 means unknown
    else if (m_drawing_mode == 2) {
        // Hide the real map
        fl_rectf(0, 25, w(), h() + 25, 127,127,127);

        // Scan m_gridmap and draw
        // We have offset of 25 pixels because of menu bar
        for (int y = 0; y < h() + 25; y = y + 1) {
            for (int x = 0; x < w(); x = x + 1) {
                if (m_grid_map[x + y * m_cfg->GRID_MAP_W] != 127) {
                    unsigned char c = 255 - m_grid_map[x + y * m_cfg->GRID_MAP_W];
                    fl_color(c, c, c);
                    fl_point(x, y + drawing_offset);
                } //end if
            } // end for x
        } // end for y
    } // end else if
}

void CCanvas::draw_odom()
{
    // Draw particles
    fl_color(FL_BLUE);
    for (int i = 0; i < m_cfg->ODOM_SAMPLES; i ++)
        fl_point((int) m_particles[i].x(), (int) m_particles[i].y());

    // Choose one particle to draw as a robot
    // Draw with dash style line
    const int choose = m_cfg->CHOSEN_SAMPLE;

    fl_color(FL_RED);
    fl_line_style(FL_DASH);
    CPoint2D p0(m_particles[choose].x(), m_particles[choose].y());
    CPoint2D p1(p0.translate(m_cfg->ROBOT_RADIUS, m_particles[choose].th()));
    fl_line(m_particles[choose].x(), m_particles[choose].y(), p1.x(), p1.y());
    fl_circle(m_particles[choose].x(), m_particles[choose].y(), m_cfg->ROBOT_RADIUS);
    fl_line_style(FL_SOLID);
}

void CCanvas::timeout_cb(void *userdata)
{
    CCanvas *o = (CCanvas*)userdata;
    o->redraw();
    Fl::repeat_timeout(o->m_cfg->SCREEN_TIMEOUT, timeout_cb, userdata);
}
