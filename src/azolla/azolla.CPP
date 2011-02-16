#include "azolla.H"

////////////////////////////////////////////////////////////////////////////////
// PROCESS
////////////////////////////////////////////////////////////////////////////////

CAzolla::CAzolla(CSimulationWindow *w)
{
    m_window = w;
    m_time_step = 0.02; // Default time step
    m_global_flag = true; // Global flag telling this class is running
    m_first_time_flag = true;
    m_map_update_type = 0;
    m_speed_l = 0.0;
    m_speed_r = 0.0;
    //az_set_location(w->h()/2, w->w()/2, 0.0); // Initial position: at the center
    az_set_location(50, w->w()/2, 0.0); // Initial position: at the center

    m_pos_x0 = m_pos_x;
    m_pos_y0 = m_pos_y;
    m_pos_th0 = m_pos_th;
}

CAzolla::~CAzolla(void)
{
    // Clear the global flag, telling that this class has been destroyed
    m_global_flag = false;
}

inline void CAzolla::sonar_sensor()
{

}

inline void CAzolla::lidar_sensor()
{
    const double factor = LIDAR_SWEEP_ANGLE / LIDAR_RAYS;

    // For every sensor
    for (int i = 0; i < LIDAR_RAYS; i++) {
        // po = robot current position
        // pt = laser ray farthest end point
        CPoint2D po(m_pos_x, m_pos_y);
        CPoint2D pt = po.translate(LIDAR_MAX,
                                   m_pos_th +  factor * i + LIDAR_START_ANGLE);

        // Input data to sensor and update its value
        m_sensor_data[i].set_start_point(po);
        m_sensor_data[i].set_end_point(pt);
        m_sensor_data[i].update_value();
    }
}

void CAzolla::az_step()
{
    // update sensor value
    lidar_sensor();

    // Robot's kinematic
    // Make it fast: compute--store--use--reuse!
    double plus_factor = m_speed_r + m_speed_l;
    double minus_factor = m_speed_l - m_speed_r;
    double half_plus_factor = plus_factor / 2;
    double a = ROBOT_SIZE_2 * plus_factor / minus_factor;
    double b = m_pos_th + minus_factor * m_time_step / ROBOT_SIZE;
    double cos_theta = cos(m_pos_th);
    double sin_theta = sin(m_pos_th);

    // Division by zero!
    if (minus_factor != 0.0)
    {
        m_pos_th =  b;
        m_pos_x += a * (sin(b) - sin_theta);
        m_pos_y -= a * (cos(b) - cos_theta);
    }
    // Robot moves in staright direction
    else
    {
        m_pos_x += half_plus_factor * cos_theta * m_time_step;
        m_pos_y += half_plus_factor * sin_theta * m_time_step;
    }

    m_window->set_robot_pos(m_pos_x, m_pos_y, m_pos_th);
}

////////////////////////////////////////////////////////////////////////////////
// SET
////////////////////////////////////////////////////////////////////////////////

void CAzolla::az_set_location(double x, double y, double th)
{
    m_pos_th = th;
    m_pos_x = x;
    m_pos_y = y;
    m_window->set_robot_pos(m_pos_x, m_pos_y, m_pos_th);
}

void CAzolla::az_set_lspeed(double s)
{
    m_speed_l = s;
}

void CAzolla::az_set_rspeed(double s)
{
    m_speed_r = s;
}

void CAzolla::az_set_time_step(double t)
{
    m_time_step = t;
}

void CAzolla::az_set_simulation_fn(FUN fun)
{
    m_fun = fun;
    init_local_map(127);
    init_global_map(127);
    init_odom_data();

    m_window->set_addr_grid_data(m_global_map);
    m_window->set_addr_sensor_data(m_sensor_data);
    m_window->set_addr_odom_data(m_odom_x, m_odom_y);

    boost::thread sim(simulation_thread, (void*) this);
}

////////////////////////////////////////////////////////////////////////////////
// GET
////////////////////////////////////////////////////////////////////////////////

double CAzolla::az_get_lspeed()
{
    return m_speed_l;
}

double CAzolla::az_get_rspeed()
{
    return m_speed_r;
}

double CAzolla::az_get_pos_x()
{
    return m_pos_x;
}

double CAzolla::az_get_pos_y()
{
    return m_pos_y;
}

double CAzolla::az_get_angle()
{
    return m_pos_th;
}

double CAzolla::az_get_time_step()
{
    return m_time_step;
}

unsigned char * CAzolla::az_get_local_map()
{
    calc_occ();
    return m_local_map;
}

unsigned char * CAzolla::az_get_global_map1()
{
    m_map_update_type = 1;
    calc_occ();
    return m_global_map;
}

unsigned char * CAzolla::az_get_global_map2()
{
    m_map_update_type = 2;
    calc_occ();
    return m_global_map;
}

void CAzolla::az_get_odometer_data(double x[], double y[], double th[])
{
    x = m_odom_x;
    y = m_odom_y;
    th = m_odom_th;
}

////////////////////////////////////////////////////////////////////////////////
// Calculation
////////////////////////////////////////////////////////////////////////////////

int CAzolla::az_calc_index(int x, int y)
{
    return x + y * GRID_MAP_W;
}

void CAzolla::az_update_odometer()
{
    boost::thread odom(odometry_thread, (void*) this);
}

////////////////////////////////////////////////////////////////////////////////
// PRIVATE MEMBERS
////////////////////////////////////////////////////////////////////////////////

void CAzolla::simulation_thread(void *param)
{

    CAzolla *o = (CAzolla*)param;
    o->simulation_thread_worker();

}

void CAzolla::simulation_thread_worker()
{
    while (m_global_flag) {
        if (m_window->get_simulation_flag()) {
            // At first iteration only, tell the sensor binary map it has to scan.
            if (m_first_time_flag) {
                for (int i = 0; i < LIDAR_RAYS; i++)
                    m_sensor_data[i].bit_map_to_scan(m_window->w(),
                                                     m_window->h(),
                                                     m_window->get_binary_map());

                m_first_time_flag = false;
            } // end i

            m_fun(this);
        } // end if
        else
            m_first_time_flag = true;

        usleep(m_window->get_simulation_speed());
    } // end while
}

void CAzolla::odometry_thread(void *param)
{
    CAzolla *o = (CAzolla*)param;
    o->odometry_thread_worker();
}

void CAzolla::odometry_thread_worker()
{
    // We should store the data first to local vaiables since this function is a
    // a separated thread.
    double x = m_pos_x;
    double y = m_pos_y;
    double th = m_pos_th;

    // Random number generator
    boost::mt19937 rng(static_cast<unsigned int>(std::time(0)));

    // How far the robot moved at last 1 time sampling
    double dx = x - m_pos_x0;
    double dy = y - m_pos_y0;
    double dth = th - m_pos_th0;
    double dp  = sqrt(dx * dx + dy * dy);

    // Normal distribution, zero mean
    boost::normal_distribution<double> Etrs(MT * dp, KT * dp);
    boost::normal_distribution<double> Edrft(MD * dp, KD * dp);
    boost::normal_distribution<double> Erot(MR * dp, KR * std::abs(dth));

    boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > vgtrs(rng, Etrs);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > vgdrft(rng, Edrft);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > vgrot(rng, Edrft);

    for (int i = 0; i < ODOM_SAMPLES; i++) {
        double trs = vgtrs();
        double drft = vgdrft();
        double rot = vgrot();

        m_odom_th[i] = m_odom_th[i] + dth + rot;
        m_odom_th[i] = m_odom_th[i] + drft;
        m_odom_x[i] = m_odom_x[i] + (dp + trs) * cos(m_odom_th[i]);
        m_odom_y[i] = m_odom_y[i] + (dp + trs) * sin(m_odom_th[i]);
    } // end for i

    // Store previous robot pose
    m_pos_x0 = x;
    m_pos_y0 = y;
    m_pos_th0 = th;
}

inline void CAzolla::init_local_map(unsigned char v)
{
    for (int y = 0; y < GRID_MAP_H; ++y)
        for (int x = 0; x < GRID_MAP_W; ++x)
            m_local_map[x + y * GRID_MAP_W] = v;
}

inline void CAzolla::init_global_map(unsigned char v)
{
    for (int y = 0; y < GRID_MAP_H; ++y)
        for (int x = 0; x < GRID_MAP_W; ++x)
            m_global_map[x + y * GRID_MAP_W] = v;
}

inline void CAzolla::init_odom_data()
{
    for (int i = 0; i < ODOM_SAMPLES; i++) {
        m_odom_x[i] = m_pos_x;
        m_odom_y[i] = m_pos_y;
        m_odom_th[i] = m_pos_th;
    }
}

inline void CAzolla::calc_occ()
{
    // Compute occupancy probability of all bresenham points of each ray.
    // If sensor actual value is s, then r is distance form a bresenham point
    // to s, and OCC = exp(-0.5 * r ^2 / var^2).

    // Initialize with probabilty of 127, which means 0.5
    init_local_map(127);

    // Fore every ray
    for (int i = 0; i < LIDAR_RAYS; i++) {
        const std::vector<CPoint2D> br_vect = m_sensor_data[i].get_br_pt();
        const double dist = m_sensor_data[i].get_value();

        // Compute probability for every bresenham point
        for (int j = 0; j < br_vect.size(); j++) {
            // TODO : Check again this part!!!
            if  (m_sensor_data[i].get_br_dist_to_start_pt(j) > dist + LIDAR_VAR_2)
                break;

            double r = m_sensor_data[i].get_br_dist_to_hit_pt(j);
            unsigned char occ = (unsigned char) (exp(-0.5*r*r/LIDAR_VAR_2) * 255.0);

            int x = br_vect.at(j).x();
            int y = br_vect.at(j).y();
            int index = x + y * GRID_MAP_W;

            if (index >= 0 && index < m_window->get_area()) {
                m_local_map[x + y * GRID_MAP_W] = occ;
                update_global_map(x, y, occ);
            } //end if
        } // end for j
    } // end for i

    m_map_update_type = 0;
}

inline void CAzolla::update_global_map(int x, int y, unsigned char occ)
{
    if (m_map_update_type == 1)
        m_global_map[x + y * GRID_MAP_W] = occ;

    else if (m_map_update_type == 2)
        m_global_map[x + y * GRID_MAP_W] *= occ;
}
