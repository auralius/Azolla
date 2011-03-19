#include "pose.H"


CPose::CPose(double x, double y, double th)
{
    m_x = x;
    m_y = y;
    m_th = th;
}

void CPose::set_th(double th)
{
    m_th = th;
}

const CPose CPose::operator+(const CPose &other) const
{
    return CPose(m_x + other.x(), m_y + other.y(), m_th + other.th());
}

const CPose CPose::operator-(const CPose &other) const
{
    return CPose(m_x - other.x(), m_y - other.y(), m_th - other.th());
}

CPose& CPose::operator=(const CPose& other)
{
    if (this == &other) // Check for self assignment!
        return *this;

    m_x = other.x();
    m_y = other.y();
    m_th = other.th();
    return *this;
}

double CPose::th() const
{
    return m_th;
}
