#include "point2d.H"

CPoint2D::CPoint2D(double X, double Y)
{
    m_x = X;
    m_y = Y;
}

CPoint2D::CPoint2D(const CPoint2D &p)
{
    m_x = p.x();
    m_y = p.y();
}

CPoint2D::~CPoint2D(void)
{
}

void CPoint2D::set_x(double x)
{
    m_x = x;
}

void CPoint2D::set_y(double y)
{
    m_y = y;
}

const CPoint2D CPoint2D::operator+(const CPoint2D &other) const
{
    return CPoint2D(m_x + other.x(), m_y + other.y());
}

const CPoint2D CPoint2D::operator-(const CPoint2D &other) const
{
    return CPoint2D(m_x - other.x(), m_y - other.y());
}

CPoint2D& CPoint2D::operator=(const CPoint2D& other)
{
    if (this == &other) // Check for self assignment!
        return *this;

    m_x = other.x();
    m_y = other.y();
    return *this;
}

CPoint2D CPoint2D::rotate_about(CPoint2D& o, double rad)
{
    return CPoint2D((m_x-o.x()) * cos(rad) - (m_y - o.y()) * sin(rad) + o.x(),
                    (m_y-o.y()) * cos(rad) + (m_x - o.x()) * sin(rad) + o.y());
}

CPoint2D CPoint2D::translate(double s, double rad)
{
    CPoint2D p(m_x, m_y);
    CPoint2D p_(m_x + s, m_y);
    return p_.rotate_about(p, rad);

}

double CPoint2D::measure_from(CPoint2D& o)
{
    return sqrt(pow((m_x - o.x()), 2) +
                pow((m_y - o.y()), 2));
}

double CPoint2D::x() const
{
    return m_x;
}

double CPoint2D::y() const
{
    return m_y;
}
