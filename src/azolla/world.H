/**
*  @file   world.H
*  @brief  Contains class to process map file
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
	/**
	 * Constructor
	 * @param f Map file name (PNG file)
	 * @param cfg Robot configuration
	 */
    CWorld (const char * f, const AZ_CONFIG *cfg);

    /**
     * Destructor.
     */
    ~CWorld(void);

    /**
     * Get loaded map in C-style array of char.
     * @return Map as an array of char
     */
    unsigned char *get_map();

    /**
	 * Get binary version of loaded map.
	 * @return Map as an array of boolean
	 */
    bool *get_binary_map();

    /**
     * Get map height.
     * @return Map height (still in pixels)
     */
    int get_height();

    /**
     * Get map 	width.
     * @return Map width (still in pixels)
     */
    int get_width();

private:
	/// World width.
    int m_width;

	/// World height.
    int m_height;

	/// Greyscaled map for display.
    unsigned char *m_gs_map;

	/// 1 bit binary map for object detection.
    bool *m_binary_map;

	/// Pointer to m_cfg in CAZolla which contains all robot parameters.
	const AZ_CONFIG *m_cfg;

};

#endif // WORLD_H_
