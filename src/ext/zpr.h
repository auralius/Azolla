/**
 * Mouse Manipulation API
 * Modified from:
 * Zoom-pan-rotate mouse manipulation module for GLUT
 * Version 0.4, October 2003
 *
 * Nigel Stewart
 * School of Computer Science and Information Technology
 * RMIT University
 * nigels@cs.rmit.edu.au
 */

#ifndef ZPR_H
#define ZPR_H

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/glu.h>

#ifdef __cplusplus
extern "C"
{
#endif


enum MouseButton
{
	BUTTON_DOWN,
	BUTTON_UP,
	BUTTON_LEFT,
	BUTTON_RIGHT,
	BUTTON_MIDDLE
};

extern void zprInit();
extern void zprMouse(int button, int state, int x, int y);
extern void zprReshape(int w,int h);
extern void zprMotion(int x, int y);
extern GLfloat zprReferencePoint[4];


#ifdef __cplusplus
}
#endif

#endif
