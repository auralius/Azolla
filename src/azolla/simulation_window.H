//
// auralius (auralius@gmx.com)
//

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/fl_ask.H>
#include <FL/Fl_Bitmap.H>
#include <stdlib.h>
#include <string.h>
#include "canvas.H"
#include "world.H"


class CSimulationWindow : public Fl_Double_Window
{
public:
    CSimulationWindow(int w,int h,const char *l);
    ~CSimulationWindow(void);
    bool get_simulation_flag();
    int get_simulation_speed();
    const bool *get_binary_map();
    int get_area();

    void set_robot_pos(double x, double y, double th);
    void set_addr_sensor_data(CSensor *p);
    void set_addr_grid_data(unsigned char *p);
    void set_addr_odom_data(double *x, double *y);


private:
    void window_configuration();
    static void menu_cb(Fl_Widget*w, void*data);
    void menu_cb_worker();

    bool m_simulation_running;
    int m_simulation_speed;
    int m_area;
    CWorld *m_world;
    CCanvas *m_canvas;
    Fl_Menu_Bar *m_menu;
    Fl_RGB_Image *m_image;
};
