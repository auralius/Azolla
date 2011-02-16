//
// auralius (auralius@gmx.com)
//

#pragma once

#include "point2d.H"


class CSensor
{
public:
    /// Constructor
    CSensor(void);

    /// Set
    void update_value();
    void set_start_point(CPoint2D pt);
    void set_end_point(CPoint2D pt);
    void bit_map_to_scan(int w, int h, const bool *binary_map);
    void enable_noise(bool status);

    /// Get
    double get_value();
    double get_br_dist_to_start_pt(int i);
    double get_br_dist_to_end_pt(int i);
    double get_br_dist_to_hit_pt(int i);
    CPoint2D get_start_point();
    CPoint2D get_end_point();
    CPoint2D get_hit_point();
    const std::vector<CPoint2D> &get_br_pt();

private:
    void bresenham_line();

    CPoint2D m_start_pt;
    CPoint2D m_end_pt;
    CPoint2D m_hit_pt;
    std::vector<CPoint2D> m_bresenham_pt;

    const bool *m_binary_map;
    int m_w;
    int m_h;
    int m_area;

    double m_raw_value;
    double m_noise;
    bool m_noise_flag;
};

// TODO : Add noise feature in sensor readings
// TODO : Sensor geometry: sensor position respect to robot center and its angle.
