#include "world.H"
#include "stdio.h"

CWorld::CWorld()
{

}

CWorld::CWorld(const char * f)
{
    Fl_PNG_Image map(f);

    m_width = map.w();
    m_height = map.h();

    m_gs_map = new unsigned char[m_width * m_height];
    m_binary_map = new bool [m_width * m_height];

    const unsigned char *datap = (const unsigned char *) map.data()[0];
    unsigned char r,g,b;

    // Convert to grayscale
    for ( int y = 0; y < map.h(); y++ ) {
        for ( int x = 0; x < map.w(); x++ ) {
            long index = (y * map.w() * map.d()) + (x * map.d());
            r = *(datap+index+0);
            g = *(datap+index+1);
            b = *(datap+index+2);
            m_gs_map[x + y * m_width] = 0.30 * r + 0.59 * g + 0.11 * b;

            // Thin border on the 4 edges of the window
            if (y < 23 )
                m_gs_map[x + y * m_width] = 0;
            if (y > m_height - 8 )
                m_gs_map[x + y * m_width] = 0;
            if (x < 2 )
                m_gs_map[x + y * m_width] = 0;
            if (x > m_width - 3 )
                m_gs_map[x + y * m_width] = 0;
        }
    }

    // 1 bit map
    for ( int y = 0; y < map.h(); y++ ) {
        for ( int x = 0; x < map.w(); x++ ) {
            if (m_gs_map[x + y * m_width] >= 127)
                m_binary_map[x + y * m_width] = false;  // No objects
            else
                m_binary_map[x + y * m_width] = true;;  // Yes objects
        }
    }

    // Add light gray as background color to representate unknown area
    for ( int y = 0; y < map.h(); y++ ) {
        for ( int x = 0; x < map.w(); x++ ) {
            if (m_gs_map[x + y * m_width] >= 254)
                m_gs_map[x + y * m_width] = 200; // Light gray
        }
    }
}

CWorld::~CWorld(void)
{
    delete [] m_gs_map;
    delete [] m_binary_map;
}


unsigned char* CWorld::get_map()
{
    return m_gs_map;
}

bool* CWorld::get_binary_map()
{
    return m_binary_map;
}

int CWorld::get_height()
{
    return m_height;
}

int CWorld::get_width()
{
    return m_width;
}

