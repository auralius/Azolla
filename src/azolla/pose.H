/**
 *  @file   pose.H
 *  @brief  Contains class for 3D coordinate (X,Y,Theta)
 *  @author Auralius Manurung <auralius@gmx.com>
 *  @date   2/19/2011
 */

#ifndef POSE_H_
#define POSE_H_


#include "point2d.H"

class CPose : public CPoint2D
{
public:
    /**
     * Constructor.
     * @param x X coordinate
     * @param y Y coordinate
     * @param th Theta coordinate
     */
    CPose(double x = 0, double y = 0, double th = 0);

    /**
     * Set theta coordinate.
     * @param th Desired theta coordinate
     */
    void set_th(double th);

    /**
     * Get theta coordinate.
     * @return Theta coordinate
     */
    double th() const;

public: // Overload operators
    /**
     * Overloading for minus operator.
     */
    const CPose operator -(const CPose & other) const;

    /**
     * Overloading for plus operator.
     */
    const CPose operator +(const CPose & other) const;

    /**
     * Overloading for assignment operator.
     */
    CPose & operator =(const CPose & other);

private:
    /// Theta coordinate.
    double m_th;

};

#endif // POSE_H_
