/**
 *  @file   configure.H
 *  @brief  Contains various constant definitions for simulation
 *  @author Auralius Manurung <auralius@gmx.com>
 *  @date   2/19/2011
 */

#ifndef CONFIGURE_H_
#define CONFIGURE_H_

#include <FL/math.h>

#include "config_struct.H"
#include "config.H"


/// Drawing mode
enum DrawingMode
{
	AZ_DRAW_SENSOR_STRIKE = 1,
	AZ_DRAW_GRID_MAP
};


/**
 * To activate 3D view feature, OpenGL must used.
 * Disable this definition to use OpenGL
 */
//#define NO_OPENGL


/**
 * Time delay for simulation,
 * accessed from menu bar: Simulation >> Speed >> Slow/Medium/Fast
 */
#ifdef WIN32
	#define SLOW     20
	#define MEDIUM   10
	#define FAST     0
#else
	#define SLOW     200
	#define MEDIUM   50
	#define FAST     20
#endif // WIN32

#endif // CONFIGURE_H_
