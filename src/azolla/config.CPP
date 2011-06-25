#include "config.H"

CConfig::CConfig(void )
{
	/* Default value */
	m_scale_factor = 192.0;
	m_time_out = 0.10;
	m_robot_diameter = 0.1;
	m_grid_map_w = 500;
	m_grid_map_h = 500;
	m_lidar_stdev = 0.1;
	m_lidar_start_angle = -90;
	m_lidar_sweep_angle = 180;
	m_lidar_ray_num = 100;
	m_lidar_max_distance = 1.0;
	m_chosen_sample = 0;
	m_odom_samples = 1000;
	m_KT = 0.10;
	m_KD = 0.10;
	m_KR = 0.08;
	m_MT = 0.0;
	m_MD = 0.0;
	m_MR = 0.0;
	m_robot_diameter = m_robot_diameter * m_scale_factor;
	m_robot_radius = 0.5 * m_robot_diameter;
	m_lidar_stdev = m_lidar_stdev * m_scale_factor;
	m_lidar_max_distance = m_lidar_max_distance * m_scale_factor;
	m_KT = m_KT / m_scale_factor;
	m_KD = m_KD / m_scale_factor;
	m_KR = m_KR / m_scale_factor;
}

CConfig::~CConfig(void )
{

}

int CConfig::read_config_file(const char * fn)
{
	ifstream f(fn);

    if (!f) {
        return -1;
    }

	set<string> items;
	items.insert("*");

	// Parser
	for (pod::config_file_iterator i(f, items), e ; i != e; ++i)
	{
		if (i->string_key.compare("SCALE_FACTOR") == 0)
			m_scale_factor = atof(i->value[0].c_str());

		else if (i->string_key.compare("SCREEN_TIMEOUT") == 0)
			m_time_out = atof(i->value[0].c_str());

		else if (i->string_key.compare("ROBOT_SIZE") == 0)
			m_robot_diameter = atof(i->value[0].c_str());

		else if (i->string_key.compare("GRID_MAP_H") == 0)
			m_grid_map_w = atoi(i->value[0].c_str());

		else if (i->string_key.compare("GRID_MAP_W") == 0)
			m_grid_map_h =atoi(i->value[0].c_str());

		else if (i->string_key.compare("LIDAR_STDEV") == 0)
			m_lidar_stdev = atof(i->value[0].c_str());

		else if (i->string_key.compare("LIDAR_START_ANGLE") == 0)
			m_lidar_start_angle = atof(i->value[0].c_str());

		else if (i->string_key.compare("LIDAR_SWEEP_ANGLE") == 0)
			m_lidar_sweep_angle = atof(i->value[0].c_str());

		else if (i->string_key.compare("LIDAR_RAYS") == 0)
			m_lidar_ray_num = atoi(i->value[0].c_str());

		else if (i->string_key.compare("LIDAR_MAX") == 0)
			m_lidar_max_distance = atof(i->value[0].c_str());

		else if (i->string_key.compare("CHOSEN_SAMPLE") == 0)
			m_chosen_sample = atoi(i->value[0].c_str());

		else if (i->string_key.compare("ODOM_SAMPLES") == 0)
			m_odom_samples = atoi(i->value[0].c_str());

		else if (i->string_key.compare("KT") == 0)
			m_KT = atof(i->value[0].c_str());

		else if (i->string_key.compare("KD") == 0)
			m_KD = atof(i->value[0].c_str());

		else if (i->string_key.compare("KR") == 0)
			m_KR = atof(i->value[0].c_str());

		else if (i->string_key.compare("MT") == 0)
			m_MT = atof(i->value[0].c_str());

		else if (i->string_key.compare("MD") == 0)
			m_MD = atof(i->value[0].c_str());

		else if (i->string_key.compare("MR") == 0)
			m_MR = atof(i->value[0].c_str());
	}

	// Scaling
	m_robot_diameter = m_robot_diameter * m_scale_factor;
	m_robot_radius = 0.5 * m_robot_diameter;
	m_lidar_stdev = m_lidar_stdev * m_scale_factor;
	m_lidar_max_distance = m_lidar_max_distance * m_scale_factor;
	m_KT = m_KT / m_scale_factor;
	m_KD = m_KD / m_scale_factor;
	m_KR = m_KR / m_scale_factor;

	return 0;
}

double CConfig::get_scale_factor()
{
	return m_scale_factor;
}

double CConfig::get_time_out()
{
	return m_time_out;
}

double CConfig::get_robot_radius()
{
	return m_robot_radius;
}

double CConfig::get_robot_diameter()
{
	return m_robot_diameter;
}

int CConfig::get_grid_map_w()
{
	return m_grid_map_w;
}

int CConfig::get_grid_map_h()
{
	return m_grid_map_h;
}

double CConfig::get_lidar_start_angle()
{
	return m_lidar_start_angle;
}

double CConfig::get_lidar_sweep_angle()
{
	return m_lidar_sweep_angle;
}

int CConfig::get_lidar_ray_num()
{
	return m_lidar_ray_num;
}

double CConfig::get_lidar_max_dist()
{
	return m_lidar_max_distance;
}

double CConfig::get_lidar_variance()
{
}

double CConfig::get_lidar_stdev()
{
	return m_lidar_stdev;
}

int CConfig::get_chosen_sample()
{
	return m_chosen_sample;
}

int CConfig::get_odom_samples()
{
	return m_odom_samples;
}

double CConfig::KT()
{
	return m_KT;
}

double CConfig::KD()
{
	return m_KD;
}

double CConfig::KR()
{
	return m_KR;
}

double CConfig::MD()
{
	return m_MD;
}

double CConfig::MT()
{
	return m_MT;
}

double CConfig::MR()
{
	return m_MR;
}

