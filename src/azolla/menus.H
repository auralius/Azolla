#include <FL/Fl_Menu_Bar.H>


void open_cb(Fl_Widget*, void*)
{

}

void quit_cb(Fl_Widget*, void*)
{

}

void cut_cb(Fl_Widget*, void*)
{

}

Fl_Menu_Item menuitems[] = {
  { "&File",              0, 0, 0, FL_SUBMENU },
    { "&Load Image File...",    FL_CTRL + 'o', (Fl_Callback *)open_cb },
    { "E&xit", FL_CTRL + 'q', (Fl_Callback *)quit_cb, 0 },
    { 0 },

  { "&Simulation", 0, 0, 0, FL_SUBMENU },
    { "R&un",        FL_CTRL + 'x', (Fl_Callback *)cut_cb },
    { 0 },

  { 0 }
};
