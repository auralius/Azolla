#include "azolla.H"


///////////////////////////////////////////////////////////////////////////////
// PROCESS
///////////////////////////////////////////////////////////////////////////////

CAzolla::CAzolla(CSimulationWindow *w)
{
    m_window = w;
    m_speed_l = 0.0;
    m_speed_r = 0.0;
    m_pose0 = m_pose; // Initial previous robot pose
    m_time_step = 0.02; // Default time step
    m_step_num = 0;
    m_global_flag = true; // Global flag, this class is running
    m_first_time_flag = true;
    m_map_update_type = 0;
}

CAzolla::~CAzolla(void)
{
    // Clear the global flag, telling that this class has been destroyed
    m_global_flag = false;

    fprintf(stdout, "Cleaning memory [CAzolla]\n");
    fflush(stdout);

	delete [] m_particles;
	delete [] m_sensor_data;
	delete [] m_local_map;
	delete [] m_global_map;
}

int CAzolla::az_read_config_file(const char *fn)
{
    // Read configuration file
    CConfig c;
    int ret = c.read_config_file(fn);
    if (ret == -1 ) {
    	fprintf(stderr, "Error loading config file: %s.\nFile not found!\n" \
    			"Using default configuration.\n" , fn);
    	fflush(stderr);

    }

    // Store to memory
    m_cfg.SCALE_FACTOR = c.get_scale_factor();
    m_cfg.SCREEN_TIMEOUT = c.get_time_out();
    m_cfg.ROBOT_DIAMETER = c.get_robot_diameter();
    m_cfg.ROBOT_RADIUS = c.get_robot_radius();
    m_cfg.GRID_MAP_H = c.get_grid_map_h();
    m_cfg.GRID_MAP_W = c.get_grid_map_h();
    m_cfg.LIDAR_STDEV = c.get_lidar_stdev();
    m_cfg.LIDAR_START_ANGLE = c.get_lidar_start_angle();
    m_cfg.LIDAR_SWEEP_ANGLE = c.get_lidar_sweep_angle();
    m_cfg.LIDAR_RAYS  = c.get_lidar_ray_num();
    m_cfg.LIDAR_MAX = c.get_lidar_max_dist();
    m_cfg.CHOSEN_SAMPLE = c.get_chosen_sample();
    m_cfg.ODOM_SAMPLES = c.get_odom_samples();
    m_cfg.KT = c.KT();
    m_cfg.KD = c.KD();
    m_cfg.KR = c.KR();
    m_cfg.MT = c.MT();
    m_cfg.MD = c.MD();
    m_cfg.MR = c.MR();

	// Iniatialize simulation based on loaded configuration
    init_sim();

	return ret;
}

void CAzolla::az_step()
{
    m_step_num++;

    // update sensor value
    lidar_sensor();

    // Robot's kinematic
    // Make it fast: compute--store--use--reuse!
    double plus_factor = m_speed_r + m_speed_l;
    double minus_factor = m_speed_l - m_speed_r;
    double half_plus_factor = plus_factor / 2;
    double a = m_cfg.ROBOT_RADIUS * plus_factor / minus_factor;
    double b = m_pose.th() + minus_factor * m_time_step / m_cfg.ROBOT_DIAMETER;
    double cos_theta = cos(m_pose.th());
    double sin_theta = sin(m_pose.th());

    // Division by zero, robot moves on a straight line
    if (std::abs(minus_factor) > 0.001) {
        m_pose.set_th(b);
        m_pose.set_x(m_pose.x() + a * (sin(b) - sin_theta));
        m_pose.set_y(m_pose.y() - a * (cos(b) - cos_theta));
    }

    else {
        m_pose.set_x(m_pose.x() + half_plus_factor * cos_theta * m_time_step);
        m_pose.set_y(m_pose.y() + half_plus_factor * sin_theta * m_time_step);
    }
}

void CAzolla::az_log_local_map(const char *fn)
{
    std::ofstream f(fn,std::ios::out);

    if (f.is_open()) {
        for ( int y = 0; y < m_window->h(); y++ ) {
            for ( int x = 0; x < m_window->w(); x++ ) {
                f << (int) m_local_map[x + y * m_cfg.GRID_MAP_W];
                f << "\t";
            } // end for x
            f << "\n";
        } // end for y
    f.close();
    } // end if
}

void CAzolla::az_log_global_map(const char *fn)
{
    std::ofstream f(fn);

    if (f.is_open()) {
        for ( int y = 0; y < m_window->h(); y++ ) {
            for ( int x = 0; x < m_window->w(); x++ ) {
                f << (int) m_global_map[x + y * m_cfg.GRID_MAP_W];
                f << "\t";
            } // end for x
            f << "\n";
        } // end for y
    f.close();
    }  // end if
}

void CAzolla::az_log_odom(const char *fn)
{
    std::ofstream f(fn);

    if (f.is_open()) {
        for (int i = 0; i < m_cfg.ODOM_SAMPLES; i ++) {
            f << m_particles[i].x() << "\t";
            f << m_particles[i].y() << "\t";
            f << m_particles[i].th() << "\n";
        }
        f.close();
    }
}

void CAzolla::az_log_sensor(const char *fn)
{
    std::ofstream f;
    f.open(fn, std::ios::app);

    if (f.is_open()) {
        for (int i = 0; i < m_cfg.LIDAR_RAYS; i++) {
            f << m_sensor_data[i].get_value() << "\t";
        }
        f << "\n";
    }
    f.close();
}

////////////////////////////////////////////////////////////////////////////////
// SET
////////////////////////////////////////////////////////////////////////////////

void CAzolla::az_set_location(double x, double y, double th)
{
    m_pose = CPose(x * m_cfg.SCALE_FACTOR, y * m_cfg.SCALE_FACTOR, th);
    init_odom_data();
}

void CAzolla::az_set_lspeed(double s)
{
    m_speed_l = s * m_cfg.SCALE_FACTOR;
}

void CAzolla::az_set_rspeed(double s)
{
    m_speed_r = s * m_cfg.SCALE_FACTOR;
}

void CAzolla::az_set_stop()
{
    m_speed_l = 0.0;
    m_speed_r = 0.0;
}

void CAzolla::az_set_time_step(double t)
{
    m_time_step = t;
}

void CAzolla::az_set_addr_particle_data(CPose *p)
{
    m_window->get_canvas()->set_addr_particle_data(p);
}

////////////////////////////////////////////////////////////////////////////////
// GET
////////////////////////////////////////////////////////////////////////////////

double CAzolla::az_get_sensor_data(double th)
{
    // Outside of range
    if (th < m_cfg.LIDAR_START_ANGLE ||
        th > m_cfg.LIDAR_START_ANGLE + m_cfg.LIDAR_SWEEP_ANGLE)
        return 0;

    // Given theta, compute sensor index
    int index = (th - m_cfg.LIDAR_START_ANGLE) / m_cfg.LIDAR_SWEEP_ANGLE * (m_cfg.LIDAR_RAYS -1);
    return m_sensor_data[index].get_value() / m_cfg.SCALE_FACTOR;
}

double CAzolla::az_get_lspeed()
{
    return m_speed_l / m_cfg.SCALE_FACTOR;
}

double CAzolla::az_get_rspeed()
{
    return m_speed_r / m_cfg.SCALE_FACTOR;
}

double CAzolla::az_get_pos_x()
{
    return m_pose.x() / m_cfg.SCALE_FACTOR;
}

double CAzolla::az_get_pos_x_prev()
{
    return m_pose0.x() / m_cfg.SCALE_FACTOR;
}

double CAzolla::az_get_pos_y()
{
    return m_pose.y() / m_cfg.SCALE_FACTOR;
}

double CAzolla::az_get_pos_y_prev()
{
    return m_pose0.y() / m_cfg.SCALE_FACTOR;
}

double CAzolla::az_get_angle()
{
    return m_pose.th();
}

double CAzolla::az_get_time_step()
{
    return m_time_step;
}

unsigned char * CAzolla::az_get_local_map(bool with_error)
{
    calc_occupancy(with_error);
    return m_local_map;
}

unsigned char * CAzolla::az_get_global_map1(bool with_error)
{
    m_map_update_type = 1;
    calc_occupancy(with_error);
    return m_global_map;
}

unsigned char * CAzolla::az_get_global_map2(bool with_error)
{
    m_map_update_type = 2;
    calc_occupancy(with_error);
    return m_global_map;
}

CPose *CAzolla::az_get_odometry_data()
{
    return m_particles;
}

int CAzolla::az_get_step_number()
{
    return m_step_num;
}

////////////////////////////////////////////////////////////////////////////////
// Calculation
////////////////////////////////////////////////////////////////////////////////

int CAzolla::az_calc_index(int x, int y)
{
    return x + y * m_cfg.GRID_MAP_W;
}

void CAzolla::az_update_odometry()
{
    boost::thread odom(odometry_thread, (void*) this);
}

///////////////////////////////////////////////////////////////////////////////
// PRIVATE MEMBERS
///////////////////////////////////////////////////////////////////////////////

void CAzolla::init_sim()
{
	// Set m_cfg of CSimulationWindow to point m_cfg of CAzolla
	m_window->set_cfg_data(&m_cfg);

	// Create necessary arrays
	m_particles = new CPose[m_cfg.ODOM_SAMPLES];
	m_sensor_data = new CSensor[m_cfg.LIDAR_RAYS];
	m_local_map = new unsigned char[m_cfg.GRID_MAP_W * m_cfg.GRID_MAP_H];
	m_global_map = new unsigned char[m_cfg.GRID_MAP_W * m_cfg.GRID_MAP_H];

    init_local_map(127);
    init_global_map(127);
    init_odom_data();

    m_window->get_canvas()->set_addr_robot_pose(&m_pose);
    m_window->get_canvas()->set_addr_grid_data(m_global_map);
    m_window->get_canvas()->set_addr_sensor_data(m_sensor_data);
    m_window->get_canvas()->set_addr_particle_data(m_particles);

	// Initial position: center
	az_set_location(1, 1, 0.0);

    boost::thread sim(simulation_thread, (void*) this);
}

void CAzolla::simulation_thread(void *param)
{
    CAzolla *o = (CAzolla*)param;
    o->simulation_thread_worker();
}

void CAzolla::simulation_thread_worker()
{
    while (m_global_flag) {
        if (m_window->get_simulation_flag()) {
            // Do these at first time of the iteration only:
            //  tell the sensor the binary map it has to scan
            //  record robot initial pose
            //  initialize odometry data
            //  initialize sensor value
            if (m_first_time_flag) {
                for (int i = 0; i < m_cfg.LIDAR_RAYS; i++)
                    m_sensor_data[i].binary_map_to_scan(m_window->get_map_width(),
                                                        m_window->get_map_height(),
                                                        m_window->get_binary_map());
                    m_pose0 = m_pose;
                    init_odom_data();
                    m_step_num = 0;
                    lidar_sensor();

                m_first_time_flag = false;
            } // end if
            az_sim_fn();
        } // end if

        else
            m_first_time_flag = true;

#		ifdef WIN32
			Sleep(m_window->get_simulation_speed());
#		else
			usleep(m_window->get_simulation_speed());
#		endif
    } // end while
}

void CAzolla::odometry_thread(void *param)
{
    CAzolla *o = (CAzolla*)param;
    o->odometry_thread_worker();
}

void CAzolla::odometry_thread_worker()
{
    // We should store the data first to local vaiables since this function is
    // a separated thread.
    double x = m_pose.x();
    double y = m_pose.y();
    double th = m_pose.th();

    // Random number generator
    // Using generator: "lagged_fibonacci607"
    // http://www.boost.org/doc/libs/1_38_0/libs/random/random-performance.html
    boost::lagged_fibonacci607 rng(static_cast<unsigned int>(std::time(0)));

    // How far the robot moved at last 1 time sampling
    double dx = x - m_pose0.x();
    double dy = y - m_pose0.y();
    double dth = th - m_pose0.th();
    double dp  = sqrt(dx * dx + dy * dy);

    // Normal distribution
    boost::normal_distribution<double> Etrs(m_cfg.MT * dp, m_cfg.KT * dp);
    boost::normal_distribution<double> Edrft(m_cfg.MD * dp, m_cfg.KD * dp);
    boost::normal_distribution<double> Erot(m_cfg.MR * dp, m_cfg.KR * std::abs(dth));

    boost::variate_generator<boost::lagged_fibonacci607&,
                             boost::normal_distribution<double>
                            > vgtrs(rng, Etrs);
    boost::variate_generator<boost::lagged_fibonacci607&,
                            boost::normal_distribution<double>
                            > vgdrft(rng, Edrft);
    boost::variate_generator<boost::lagged_fibonacci607&,
                             boost::normal_distribution<double>
                            > vgrot(rng, Edrft);

    for (int i = 0; i < m_cfg.ODOM_SAMPLES; i++) {
        // TODO: Validate the generated random numbers!!!
        double trs = vgtrs();
        double drft = vgdrft();
        double rot = vgrot();

        m_particles[i].set_th(m_particles[i].th() + dth + rot);
        m_particles[i].set_th(m_particles[i].th() + drft);
        m_particles[i].set_x(m_particles[i].x() + (dp + trs) * cos(m_particles[i].th()));
        m_particles[i].set_y(m_particles[i].y() + (dp + trs) * sin(m_particles[i].th()));
    }

    // Store previous robot pose
    m_pose0 = CPose(x, y, th);
}

void CAzolla::sonar_sensor()
{

}

void CAzolla::lidar_sensor()
{
    const double factor = m_cfg.LIDAR_SWEEP_ANGLE / m_cfg.LIDAR_RAYS;
    CPoint2D po(0,0); // po = robot current position, where the sensors located
    CPoint2D pt(0,0); // pt = laser ray farthest end point

    // For every sensor
    for (int i = 0; i < m_cfg.LIDAR_RAYS; i++) {
        po = CPoint2D(m_pose.x() + m_cfg.ROBOT_RADIUS, m_pose.y());
        pt = po.translate(m_cfg.LIDAR_MAX,
                          m_pose.th() +  factor * i + m_cfg.LIDAR_START_ANGLE);

        // Input data to sensor and update its value
        m_sensor_data[i].set_start_point(po);
        m_sensor_data[i].set_end_point(pt);
        m_sensor_data[i].update_value();
    }
}

void CAzolla::calc_occupancy(bool with_error)
{
    // Compute occupancy probability of all bresenham points of each ray.
    // If sensor actual value is s, then r is distance form a bresenham point
    // to s, and OCC = exp(-0.5 * r ^2 / var^2).

    // Initialize with probabilty of 127, which means 0.5
    init_local_map(127);

    const double factor = m_cfg.LIDAR_SWEEP_ANGLE / m_cfg.LIDAR_RAYS;
    CPoint2D po(0,0); // po = robot current position, where the sensors located
    CPoint2D pt(0,0); // pt = laser ray farthest end point
    CPoint2D ph(0,0); // ph = sensor hit point

    // Fore every ray
    for (int i = 0; i < m_cfg.LIDAR_RAYS; i++) {
        // Handle error on odometry which will affect mapping
        if (with_error) {
            double th = m_particles[m_cfg.CHOSEN_SAMPLE].th() +  factor * i + m_cfg.LIDAR_START_ANGLE;
            po = m_particles[m_cfg.CHOSEN_SAMPLE];
            pt = po.translate(m_cfg.LIDAR_MAX, th);
            ph = po.translate(m_sensor_data[i].get_value(), th);
            m_sensor_data[i].set_start_point(po);
            m_sensor_data[i].set_end_point(pt);
            m_sensor_data[i].set_hit_point(ph);
            m_sensor_data[i].bresenham_line();
        }

        const std::vector<CPoint2D> br_pt_vect = m_sensor_data[i].get_br_pt();
        const double dist = m_sensor_data[i].get_value();

        // Compute probability for every bresenham point
        for (int j = 0; j < br_pt_vect.size(); j++) {
            // TODO : Check again this part!!!
            double r = m_sensor_data[i].get_br_dist_to_hit_pt(j);
            unsigned char occ = (unsigned char) (exp(-0.5*r*r/m_cfg.LIDAR_STDEV) * 255.0);

            if  (m_sensor_data[i].get_br_dist_to_start_pt(j) > dist && occ >= 0.5)
                break;

            int x = br_pt_vect.at(j).x();
            int y = br_pt_vect.at(j).y();
            int index = x + y * m_cfg.GRID_MAP_W;

            //if (index >= 0 && index < m_window->get_area()) {
                m_local_map[x + y * m_cfg.GRID_MAP_W] = occ;
                update_global_map(x, y, occ);
            //} //end if
        } // end for j
    } // end for i

    m_map_update_type = 0;
}

inline void CAzolla::update_global_map(int x, int y, unsigned char occ)
{
    if (m_map_update_type == 1)
        m_global_map[x + y * m_cfg.GRID_MAP_W] = occ;

    else if (m_map_update_type == 2)
        m_global_map[x + y * m_cfg.GRID_MAP_W] *= occ;
}

inline void CAzolla::init_local_map(unsigned char v)
{
    for (int y = 0; y < m_cfg.GRID_MAP_H; ++y)
        for (int x = 0; x < m_cfg.GRID_MAP_W; ++x)
            m_local_map[x + y * m_cfg.GRID_MAP_W] = v;
}

inline void CAzolla::init_global_map(unsigned char v)
{
    for (int y = 0; y < m_cfg.GRID_MAP_H; ++y)
        for (int x = 0; x < m_cfg.GRID_MAP_W; ++x)
            m_global_map[x + y * m_cfg.GRID_MAP_W] = v;
}

inline void CAzolla::init_odom_data()
{
    for (int i = 0; i < m_cfg.ODOM_SAMPLES; i++)
        m_particles[i] = m_pose;
}
