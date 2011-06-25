#include "canvas3d.H"


// Constructor
CCanvas3D::CCanvas3D(int x, int y, int w, int h, const char *l)
    : Fl_Gl_Window(x, y, w, h, l)
{
    m_map_file_loaded = false;
    m_first_time = true;
    m_drawing_mode = AZ_DRAW_SENSOR_STRIKE;
    m_simulation_running = false;
    m_scale_factor = 1.0;
    end();
}

void CCanvas3D::set_drawing_mode(int m)
{
    m_drawing_mode = m;
}

void CCanvas3D::set_addr_robot_pose(CPose *p)
{
    m_pose = p;
}

void CCanvas3D::set_addr_sensor_data(CSensor *p)
{
    m_sensor_data = p;
}

void CCanvas3D::set_addr_grid_data(unsigned char *p)
{
    m_grid_map = p;
}

void CCanvas3D::set_addr_particle_data(CPose *p)
{
    m_particles = p;
}

void CCanvas3D::set_cfg_data(const AZ_CONFIG *cfg)
{
	m_cfg = cfg;

	// Canvas refresh reate
    Fl::add_timeout(m_cfg->SCREEN_TIMEOUT, timeout_cb, (void*)this);
}

void CCanvas3D::set_map_scale_factor()
{
    if (w() > h())
        m_scale_factor = (double) w();
    else
        m_scale_factor = (double) h();

    // Window is resizeable so keep the original value
    m_map_w = (double) w();
    m_map_h = (double) h();
}

void CCanvas3D::notify_simulation_status(bool s)
{
    m_simulation_running = s;
}

void CCanvas3D::load_map_as_texture(const char *f)
{
    GLuint  texture;
    pngInfo info;

    // Load PNG file as texture
    texture = pngBind(f, PNG_NOMIPMAP, PNG_ALPHA, &info, GL_CLAMP, GL_NEAREST, GL_NEAREST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);

    // Done, set the flag
    m_map_file_loaded = true;
}

//
// Private
//

int CCanvas3D::handle(int e)
{
	int ret = Fl_Gl_Window::handle(e);
	int x = Fl::event_x();
	int y = Fl::event_y();

    switch (e) {
        case FL_PUSH:
        	if (Fl::event_button() == FL_LEFT_MOUSE)
        		zprMouse(BUTTON_LEFT, BUTTON_DOWN, x, y);

        	else if (Fl::event_button() == FL_RIGHT_MOUSE)
				zprMouse(BUTTON_RIGHT, BUTTON_DOWN, x, y);

        	else if (Fl::event_button() == FL_MIDDLE_MOUSE)
        		zprMouse(BUTTON_MIDDLE, BUTTON_DOWN, x, y);

        	return (1);

        case FL_RELEASE:
        	if (Fl::event_button() == FL_LEFT_MOUSE)
        		zprMouse(BUTTON_LEFT, BUTTON_UP, x, y);

        	else if (Fl::event_button() == FL_RIGHT_MOUSE)
        		zprMouse(BUTTON_RIGHT, BUTTON_UP, x, y);

        	else if (Fl::event_button() == FL_MIDDLE_MOUSE)
        	    zprMouse(BUTTON_MIDDLE, BUTTON_UP, x, y);

        	return (1);

        case FL_DRAG:
        	zprMotion(x, y);
        	return (1);
    }
    return ret;
}

void CCanvas3D::timeout_cb(void *userdata)
{
    CCanvas3D *o = (CCanvas3D*)userdata;
    o->redraw();
    Fl::repeat_timeout(o->m_cfg->SCREEN_TIMEOUT, timeout_cb, userdata);
}

void CCanvas3D::draw()
{
    if (!valid()) {
        valid(1);
        zprReshape(w(), h());
    }

    if (m_first_time == true) {
    	zprInit();
    	zprReshape(w(), h());

//        glClearColor(0.5, 0.5, 0.5, 1.0);
//        glClear(GL_COLOR_BUFFER_BIT);
//        glMatrixMode(GL_PROJECTION);
//        glLoadIdentity();
//
//        gluLookAt(0.1, 0.1, 0.1,   /* position of camera */
//                  0.0, 0.0, 0.0,   /* where camera is pointing at */
//                  0.0, 0.0, 1.0);  /* which direction is up */
//
//        glMatrixMode(GL_MODELVIEW);

        m_first_time = false;
    }

    glClearColor(0.5, 0.5, 0.5, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glPushMatrix();
    glTranslated(-0.5, -0.5, 0.0);
    draw_axis();
    if (m_map_file_loaded == true && m_drawing_mode == AZ_DRAW_SENSOR_STRIKE)
        draw_floor();
    draw_sensor_beam();
    draw_odom();
    draw_robot();
    glPopMatrix();

}

void CCanvas3D::draw_axis()
{
	glBegin(GL_LINES);
    // X-axis
    glColor3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(1.0, 0.0, 0.0);
    // Y-axis
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 1.0, 0.0);
    glEnd();
}

void CCanvas3D::draw_circle( double x, double y, double radius, int n_points)
{
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < n_points; i++)
    {
        double angle = i * (2.0 * M_PI / n_points);
        double X = cos(angle) * radius + x;
        double Y = sin(angle) * radius + y;
        glVertex3d(X, Y, 0);
    }
    glEnd();
}

void CCanvas3D::draw_robot()
{
    // Draw a robot as a circle with a line
    const int drawing_offset = -10; // Offset happens on y, don't know why??

    glColor3d(0.0, 0.0, 0.0); // Black
    draw_circle(m_pose->x() / m_scale_factor,
                (m_pose->y() + drawing_offset) / m_scale_factor,
                m_cfg->ROBOT_RADIUS / m_scale_factor,
                100);

    CPoint2D p0(m_pose->x(), m_pose->y());
    CPoint2D p1(p0.translate(m_cfg->ROBOT_RADIUS, m_pose->th()));

    glBegin(GL_LINE);
    glVertex3d(p0.x() / m_scale_factor, (p0.y() + drawing_offset) / m_scale_factor, 0);
    glVertex3d(p1.x() / m_scale_factor, (p1.y() + drawing_offset) / m_scale_factor, 0);
    glEnd();
}

void CCanvas3D::draw_sensor_beam()
{
    const int drawing_offset = -10; // Offset happens on y, don't know why??

    // Sensor hit mark
    if (m_drawing_mode == AZ_DRAW_SENSOR_STRIKE) {
        glColor3d(1.0, 0.0, 0.0); // Red
        glBegin(GL_POINTS);
        for (int i = 0; i < m_cfg->LIDAR_RAYS; i = i + 1)
            glVertex3d(m_sensor_data[i].get_hit_point().x() / m_scale_factor,
                       (m_sensor_data[i].get_hit_point().y() + drawing_offset) / m_scale_factor,
                       0.0);
        glEnd();
    }

    // Occupancy grid
    // Draw only grid with OCC != 127, OCC = 127 means unknown
    else if (m_drawing_mode == AZ_DRAW_GRID_MAP) {
        // Scan m_gridmap and draw
        // We have offset of 25 pixels because of menu bar
        glBegin(GL_POINTS);
        for (int y = 0; y < m_map_h + 25; y = y + 1) {
            for (int x = 0; x < m_map_w; x = x + 1) {
                if (m_grid_map[x + y * m_cfg->GRID_MAP_W] != 127) {
                    double c = (255.0 - m_grid_map[x + y * m_cfg->GRID_MAP_W]) / 255.0;
                    glColor3d(c, c, c);
                    glVertex3d(x / m_scale_factor, (y + drawing_offset) / m_scale_factor, 0.0);
                } //end if
            } // end for x
        } // end for y
        glEnd();
    } // end else if
}

void CCanvas3D::draw_odom()
{
    // Draw particles
    const int drawing_offset = -10;

    glColor3d(0.0, 0.0, 1.0); // Blue
    glBegin(GL_POINTS);
    for (int i = 0; i < m_cfg->ODOM_SAMPLES; i ++)
        glVertex3d(m_particles[i].x() / m_scale_factor, (m_particles[i].y() + drawing_offset) / m_scale_factor, 0.0);
    glEnd();

    // Choose one particle to draw as a robot
    // Draw with dash style line
    const int choose = m_cfg->CHOSEN_SAMPLE;

    glColor3d(1.0, 0.0, 0.0);

    draw_circle(m_particles[choose].x() / m_scale_factor,
                (m_particles[choose].y() + drawing_offset) / m_scale_factor,
                m_cfg->ROBOT_RADIUS / m_scale_factor,
                100);

    CPoint2D p0(m_particles[choose].x(), m_particles[choose].y());
    CPoint2D p1(p0.translate(m_cfg->ROBOT_RADIUS, m_particles[choose].th()));

    glBegin(GL_LINE);
    glVertex3d(p0.x() / m_scale_factor, (p0.y() + drawing_offset) / m_scale_factor, 0);
    glVertex3d(p1.x() / m_scale_factor, (p1.y() + drawing_offset) / m_scale_factor, 0);
    glEnd();

}

void CCanvas3D::draw_floor()
{
	// Create vertex for floor and attach map file as texture
    glBegin(GL_QUADS);

    glColor3f(0.5, 0.5, 0.5); // Gray

    glTexCoord2f(0.0, 0.0); // Top corner left
    glVertex3d(0.0, 0.0, 0.0);

    glTexCoord2f(1.0, 0.0); // Top corner right
    glVertex3d(m_map_w / m_scale_factor, 0.0, 0.0);

    glTexCoord2f(1.0, 1.0); // Bottom corner right
    glVertex3d(m_map_w / m_scale_factor, m_map_h / m_scale_factor, 0.0);

    glTexCoord2f(0.0, 1.0); // Bottom corner left
    glVertex3d(0.0, m_map_h / m_scale_factor, 0.0);

    glEnd();

}
