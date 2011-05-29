/**
 *  @file   point2d.H
 *  @brief  Contains class for 2D coordinate (X,Y)
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
    /**
     * Constructor.
     * @param x X coordinate
     * @param y Y coordinate
     */
    CPoint2D(double x = 0, double y = 0);

    /**
     * Constructor.
     * @param p Coordinate in (x,y)
     */
    CPoint2D(const CPoint2D &p);

    /**
     * Destructor.
     */
    ~CPoint2D();

    /**
     * Set X coordinate.
     * @param x X coordinate
     */
    void set_x(double x);

    /**
     * Set Y coordinate.
     * @param y Y coordinate
     */
    void set_y(double y);

    /**
     * Do rotation to a certain pivot point.
     * @param rad Angle of rotation (radian)
     * @param o Pivot point for rotation
     */
    CPoint2D rotate_about(CPoint2D& o, double rad);

    /**
     * Move a 2D coordinate with a certain translational and rotational displacement.
     * @param s Translational displacement
     * @param rad Rotational displacement
     */
    CPoint2D translate(double s, double rad);

    /**
     * Measure distance of one point to another point.
     * @param o The another point
     * @return Measure distance
     */
    double measure_from(CPoint2D& o);

    /**
     * Get X coordinate.
     * @return X coordinate
     */
    double x() const;

    /**
     * Get Y coordinate.
     * @return Y coordinate
     */
    double y() const;

public: // Overload operators
    /**
     * Overloading for minus operator.
     */
    const CPoint2D  operator -(const CPoint2D& other) const;

    /**
     * Overloading for plus operator.
     */
    const CPoint2D  operator +(const CPoint2D& other) const;

    /**
     * Overloading for assignment operator.
     */
    CPoint2D& operator =(const CPoint2D& other);

protected:
    /// X coordinate.
    double m_x;

    /// Y Coordinate.
    double m_y;
};

#endif // POINT2D_H_
