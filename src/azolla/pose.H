/**
*  @file   pose.H
*  @brief  Simple calss for 3D point: x y, and th
*  @author Auralius Manurung <auralius@gmx.com>
*  @date   2/19/2011
*/

#ifndef POSE_H_
#define POSE_H_

#include "point2d.H"

class CPose : public CPoint2D
{
public:
    CPose(double x = 0, double y = 0, double th = 0);
    void                set_th(double th);
    double              th() const;

public: // Overload operators
    const CPose         operator-(const CPose& other) const;
    const CPose         operator+(const CPose& other) const;
    CPose&              operator=(const CPose& other);

private:
    double m_th;
};

#endif // POSE_H_
