/**
 *  @file   sensor.H
 *  @brief  Contains class for the distance sensor
 *  @author Auralius Manurung <auralius@gmx.com>
 *  @date   2/19/2011
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "point2d.H"


class CSensor
{
public:
	/**
	 * Constructor
	 */
    CSensor(void);

    /**
     * Compute bresenham points which make a line segment defined by
     * m_start_pt and m_end_pt.
     */
    void bresenham_line();

    /**
     * Update sensor value, this function must be called after m_start_pt and
     * m_end_pt values have been set. This function will update the value of
     * m_hit_pt and m_raw_value.
     */
    void update_value();

    // Set

    /**
     * Set the position coordinate of the sensor.
     * @param pt Sensor start position
     */
    void set_start_point(CPoint2D pt);

    /**
     * Set the farthest position that the sensor can reach.
     * @param pt Sensor end position
     */
    void set_end_point(CPoint2D pt);

    /**
     * Set the hit point where an object lies.
     * This function should not be called explicitly because
     * hit point should be calculated by update_value() function,
     * unless it is necessary to modify m_hit_pt manually.
     * @param pt Sensor hit position
     */
    void set_hit_point(CPoint2D pt);

    /**
     * Set the binary map of the environment.
     * @param w Width of the binary map
     * @param h Height of the binary map
     * @param binary_map Pointer to the binary map
     */
    void binary_map_to_scan(int w, int h, const bool *binary_map);

    /**
     * Gaussian noise for sensor readings.
     * @param status Enable / disable
     */
    void enable_noise(bool status);

    // Get

    /**
     * Get measured distance.
     * @return Measured distance
     */
    double get_value();

    /**
     * Get distance of a certain bresenham point to m_start_pt.
     * @param i Index of the breseham point
     * @return Calculated distance
     */
    double get_br_dist_to_start_pt(int i);

    /**
     * Get distance of a certain bresenham point to m_end_pt.
     * @param i Index of the breseham point to measure
     * @return Calculated distance
     */
    double get_br_dist_to_end_pt(int i);

    /**
     * Get distance of a certain bresenham point to m_hit_pt.
     * @param i Index of the breseham point to measure
     * @return Calculated distance
     */
    double get_br_dist_to_hit_pt(int i);

    /**
     * Get the start point.
     * @return m_start_pt
     */
    CPoint2D get_start_point();

    /**
     * Get the end point.
     * @return m_end_pt
     */
    CPoint2D get_end_point();

    /**
     * Get the hit point.
     * @return m_hit_pt
     */
    CPoint2D get_hit_point();

    /**
     * Get vector that contains all the bresenham points.
     * @return m_bresenham_pt
     */
    const std::vector<CPoint2D> &get_br_pt();

private:
    /// Sensor start point.
    CPoint2D m_start_pt;

    /// Sensor end point.
    CPoint2D m_end_pt;

    /// A location where the sensor line segment crossed an object.
    CPoint2D m_hit_pt;

    /// Collection of Bresenham points along sensor line segment.
    std::vector<CPoint2D> m_bresenham_pt;

    /// Binary map of the environment.
    const bool *m_binary_map;

    /// Map width
    int m_w;

    /// Map height
    int m_h;

    /// Map area size
    int m_area;

    /// Measured distance
    double m_raw_value;

    /// Noise value
    double m_noise;

    /// Add with noise or not?
    bool m_noise_flag;
};

#endif // SENSOR_H_

// TODO : Add noise feature in sensor readings
