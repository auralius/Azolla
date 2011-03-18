#include "sensor.H"


CSensor::CSensor(void)
{
    m_raw_value = 0.0;
    m_noise = 0.0;
    m_noise_flag = false;
}

void CSensor::bresenham_line()
{
    m_bresenham_pt.clear();

    // Bresenham's line algorithm
    signed char ix;
    signed char iy;

    int x1 = m_start_pt.x();
    int y1 = m_start_pt.y();
    int x2 = m_end_pt.x();
    int y2 = m_end_pt.y();

    // if x1 == x2 or y1 == y2, then it does not matter what we set here
    int delta_x = (x2 > x1?(ix = 1, x2 - x1):(ix = -1, x1 - x2)) << 1;
    int delta_y = (y2 > y1?(iy = 1, y2 - y1):(iy = -1, y1 - y2)) << 1;

    m_bresenham_pt.push_back(CPoint2D(x1, y1));

    if (delta_x >= delta_y) {
        // error may go below zero
        int error = delta_y - (delta_x >> 1);

        while (x1 != x2) {
            if (error >= 0) {
                if (error || (ix > 0)) {
                    y1 += iy;
                    error -= delta_x;
                }
            }

            x1 += ix;
            error += delta_y;

            m_bresenham_pt.push_back(CPoint2D(x1, y1));
        } // end while
    } // end if

    else {
        // error may go below zero
        int error = delta_x - (delta_y >> 1);

        while (y1 != y2) {
            if (error >= 0) {
                if (error || (iy > 0)) {
                    x1 += ix;
                    error -= delta_y;
                }
            }

            y1 += iy;
            error += delta_x;

            m_bresenham_pt.push_back(CPoint2D(x1, y1));
        } // end while
    } // end else
}

void CSensor::update_value()
{
    if (m_binary_map == NULL)
        return;

    bresenham_line();

    for (int i = 0; i < m_bresenham_pt.size(); i++) {
        int index = (int) m_bresenham_pt.at(i).x() +
            (int) m_bresenham_pt.at(i).y() *
            m_w;

        if (index >= m_area || index < 0) {
            m_hit_pt = m_bresenham_pt.at(i);
            break;
        }

        if (m_binary_map[index] == true) {
            m_hit_pt = m_bresenham_pt.at(i);
            break;
        }
    }
    m_raw_value = m_hit_pt.measure_from(m_start_pt);
}

void CSensor::binary_map_to_scan(int w, int h, const bool *binary_map)
{
    m_binary_map = binary_map;
    m_w = w;
    m_h = h;
    m_area = m_h * m_w;
}

void CSensor::enable_noise(bool status)
{
    m_noise_flag = status;
}

void CSensor::set_start_point(CPoint2D pt)
{
    m_start_pt = pt;
}

void CSensor::set_end_point(CPoint2D pt)
{
    m_end_pt = pt;
    m_hit_pt = pt;
}

void CSensor::set_hit_point(CPoint2D pt)
{
    m_hit_pt = pt;
}

///////////////////////////////////////////////////////////////////////////////
// GET
///////////////////////////////////////////////////////////////////////////////

double CSensor::get_value()
{
    return (m_raw_value + m_noise);
}

CPoint2D CSensor::get_start_point()
{
    return m_start_pt;
}

CPoint2D CSensor::get_end_point()
{
    return m_end_pt;
}

CPoint2D CSensor::get_hit_point()
{
    return m_hit_pt;
}

const std::vector<CPoint2D> &CSensor::get_br_pt()
{
    return m_bresenham_pt;
}

double CSensor::get_br_dist_to_start_pt(int i)
{
    return m_bresenham_pt.at(i).measure_from(m_start_pt);
}

double CSensor::get_br_dist_to_end_pt(int i)
{
    return m_bresenham_pt.at(i).measure_from(m_end_pt);
}

double CSensor::get_br_dist_to_hit_pt(int i)
{
    return m_bresenham_pt.at(i).measure_from(m_hit_pt);
}


