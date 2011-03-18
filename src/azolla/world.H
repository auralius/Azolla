/**
*  @file   world.H
*  @brief  Load and convert PNG map file into grayscale and binay map
*  @author Auralius Manurung <auralius@gmx.com>
*  @date   2/19/2011
*/

#ifndef WORLD_H_
#define WORLD_H_

#include "configure.H"
#include <FL/Fl_Shared_Image.H>
#include <FL/Fl_PNG_Image.H>

class CWorld
{
public:
    CWorld ();
    CWorld (const char * f);
    ~CWorld(void);
    unsigned char *get_map();
    bool *get_binary_map();
    int get_height();
    int get_width();

private:
    int m_width;
    int m_height;

    unsigned char *m_gs_map; // Greyscaled map for display
    bool *m_binary_map;  // 1 bit map for object detection
};

#endif // WORLD_H_
