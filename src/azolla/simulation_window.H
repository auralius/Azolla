/**
 *  @file   simulation_window.H
 *  @brief  Contains class for simulation window
 *  @author Auralius Manurung <auralius@gmx.com>
 *  @date   2/19/2011
 */

#ifndef SIMULATION_WINDOW_H_
#define SIMULATION_WINDOW_H_

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/fl_ask.H>
#include <FL/Fl_Bitmap.H>

#include "configure.H"
#include "config.H"
#include "canvas.H"
#include "world.H"


class CSimulationWindow : public Fl_Double_Window
{
public:
    /**
     * Constructor.
     * @param w Window width
     * @param h Window height
     * @param l String label
     */
    CSimulationWindow(int w,int h,const char *l);

    /**
     * Destructor
     */
    ~CSimulationWindow(void);

    /**
     * Get simulation flag, wether it is running or not.
     * @return Simulation flag
     */
    bool get_simulation_flag();

    /**
     * Get simulation speed.
     * @return Simulation speed: Fast, Medium, Slow
     */
    int get_simulation_speed();

    /**
     * Get binary version of loaded map.
	 * @return Map as an array of boolean
     */
    const bool *get_binary_map();

    /**
     * Get map area size (length x width)
     * @return Area size
     */
    int get_area();

    /**
     * Get pointer to canvas.
     * @return Pointer to canvas
     */
    CCanvas *get_canvas();

    /**
     * Set configuration
     * @param cfg Configuration to set
     */
	void set_cfg_data(const AZ_CONFIG *cfg);

private:
    /// Build menu.
    void build_menu();

    /// Call back when one of the menu items clicked
    static void menu_cb(Fl_Widget*w, void*data);

    /// Menu call back worker.
    void menu_cb_worker();

    /// Flag for simulation status.
    bool m_simulation_running;

    /// Simulation speed.
    int m_simulation_speed;

    /// Map area size
    int m_area;

    /// CWorld is needed to process map file.
    CWorld *m_world;

    /// CCanvas is needed for drawing.
    CCanvas *m_canvas;

    /// Menu bar
    Fl_Menu_Bar *m_menu;

    /// Image map file
    Fl_RGB_Image *m_image;

    /// Configuraton
	const AZ_CONFIG *m_cfg;
};

#endif // SIMULATION_WINDOW_H_
