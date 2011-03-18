/**
*  @file   point2d.H
*  @brief  Simple class for 2D point
*  @author Auralius Manurung <auralius@gmx.com>
*  @date   2/19/2011
*/

#ifndef POINT2D_H_
#define POINT2D_H_

#include <math.h>
#include <vector>

class CPoint2D
{
public:
    CPoint2D(double X = 0, double Y = 0);
    CPoint2D(const CPoint2D &p);
    ~CPoint2D();
    void set_x(double x);
    void set_y(double y);
    CPoint2D rotate_about(CPoint2D& o, double rad);
    CPoint2D translate(double s, double rad);
    double measure_from(CPoint2D& o);
    double x() const;
    double y() const;

public: // Overload operators
    const CPoint2D  operator-(const CPoint2D& other) const;
    const CPoint2D  operator+(const CPoint2D& other) const;
    CPoint2D&       operator=(const CPoint2D& other);

protected:
    double m_x;
    double m_y;
};

#endif // POINT2D_H_
