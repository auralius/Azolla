/**
 *  @file   config.H
 *  @brief  Contains class for reading configuration file
 *  @author Auralius Manurung <auralius@gmx.com>
 *  @date   3/24/2011
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <fstream>
#include <set>

#include <boost/config.hpp>
#include <boost/program_options/detail/config_file.hpp>
#include <boost/program_options/parsers.hpp>

namespace pod = boost::program_options::detail;
using namespace std;

class CConfig
{
public:
    CConfig(void );
    ~CConfig(void );
    int read_config_file(const char * fn);
    double get_scale_factor();
    double get_time_out();
    double get_robot_radius();
    double get_robot_diameter();
    int get_grid_map_w();
    int get_grid_map_h();
    double get_lidar_start_angle();
    double get_lidar_sweep_angle();
    int get_lidar_ray_num();
    double get_lidar_max_dist();
    double get_lidar_variance();
    double get_lidar_stdev();
    int get_chosen_sample();
    int get_odom_samples();
    double KT();
    double KD();
    double KR();
    double MD();
    double MT();
    double MR();

private:
    double m_scale_factor;
    double m_time_out;
    double m_robot_diameter;
    double m_robot_radius;
    int m_grid_map_w;
    int m_grid_map_h;
    double m_lidar_start_angle;
    double m_lidar_sweep_angle;
    int m_lidar_ray_num;
    double m_lidar_max_distance;
    double m_lidar_stdev;
    int m_chosen_sample;
    int m_odom_samples;
    double m_KT;
    double m_KD;
    double m_KR;
    double m_MT;
    double m_MD;
    double m_MR;
};

#endif // CONFIG_H_
