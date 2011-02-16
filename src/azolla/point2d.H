//
// Simple class for 2D point
// auralius (auralius@gmx.com)
//

#pragma once

#include <math.h>
#include <vector>

class CPoint2D
{
public:
    CPoint2D(double X = 0, double Y = 0);
    CPoint2D(const CPoint2D &p);
    ~CPoint2D();
    CPoint2D rotate_about(CPoint2D& o, double rad);
    CPoint2D translate(double s, double rad);
    double measure_from(CPoint2D& o);
    double x() const;
    double y() const;

public: // Overload operators
    const CPoint2D  operator-(const CPoint2D& other) const;
    const CPoint2D  operator+(const CPoint2D& other) const;
    CPoint2D&       operator=(const CPoint2D& other);

private:
    double m_x;
    double m_y;
};


