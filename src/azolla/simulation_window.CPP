#include "simulation_window.H"


CSimulationWindow::CSimulationWindow(int w,int h,const char *l)
    :Fl_Double_Window(0 , 0, w, h, l)
{
    // Initialization
    m_simulation_running = false;
    m_simulation_speed = FAST;
    m_world = NULL;
    m_canvas = new CCanvas(0, 25, w, h-25, ""); // Width of menu bar = 25
    m_menu = new Fl_Menu_Bar(0, 0, m_canvas->w(), 25);
    m_image = NULL;

    window_configuration();
}

CSimulationWindow::~CSimulationWindow(void)
{
    // Clean up memory
    Fl::delete_widget(m_menu);
    Fl::delete_widget(m_canvas);

    if (m_world)
    {
        delete m_image;
        delete m_world;
    }
}


/**
*
* GET
*
*/

bool CSimulationWindow::get_simulation_flag()
{
    return m_simulation_running;
}

int CSimulationWindow::get_simulation_speed()
{
    return m_simulation_speed;
}

const bool *CSimulationWindow::get_binary_map()
{
    if (m_world == NULL)
        return NULL;

    return m_world->get_binary_map();
}

int CSimulationWindow::get_area()
{
    return m_area;
}

/**
*
* SET
*
*/

void CSimulationWindow::set_robot_pos(CPose &p)
{
    m_canvas->set_robot_pose(p);
}

void CSimulationWindow::set_addr_sensor_data(CSensor *p)
{
    m_canvas->set_addr_sensor_data(p);
}

void CSimulationWindow::set_addr_grid_data(unsigned char *p)
{
    m_canvas->set_addr_grid_data(p);
}

void CSimulationWindow::set_addr_odom_data(CPose *p)
{
    m_canvas->set_addr_odom_data(p);
}

/**
*
* PRIVATE MEMBERS
*
*/

void CSimulationWindow::window_configuration()
{
    // Menu bar
    m_menu->add("File/Open map...", FL_CTRL + 'o', menu_cb, (void*)this);
    m_menu->add("File/Quit", FL_CTRL + 'q', menu_cb, (void*)this);
    m_menu->add("Simulation/Run", FL_CTRL + 'r', menu_cb, (void*)this);
    m_menu->add("Simulation/Stop", FL_CTRL + 'r', menu_cb, (void*)this);
    m_menu->add("Simulation/Speed/Slow", FL_CTRL + '1', menu_cb, (void*)this);
    m_menu->add("Simulation/Speed/Medium", FL_CTRL + '2', menu_cb, (void*)this);
    m_menu->add("Simulation/Speed/Fast", FL_CTRL + '3', menu_cb, (void*)this);
    m_menu->add("Simulation/Debug/Sensor Strike", FL_CTRL + 'k', menu_cb, (void*)this);
    m_menu->add("Simulation/Debug/Grid Memory", FL_CTRL + 'm', menu_cb, (void*)this);
    m_menu->add("Help/About...", 0, menu_cb, (void*)this);

    // Radio menu bar
    Fl_Menu_Item *m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Speed/Slow");
    m->setonly();
    m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Speed/Medium");
    m->setonly();
    m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Speed/Fast");
    m->setonly();
    m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Debug/Grid Memory");
    m->setonly();
    m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Debug/Sensor Strike");
    m->setonly();
    // Deactivate "Simulation/Stop" menu
    m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Stop");
    m->deactivate();
}

void CSimulationWindow::menu_cb(Fl_Widget*w, void*data)
{
    CSimulationWindow *o = (CSimulationWindow*)data;
    o->menu_cb_worker();
}

void CSimulationWindow::menu_cb_worker()
{
    char picked[80];
    m_menu->item_pathname(picked, sizeof(picked)-1);

    // CALL BACK HANDLER!!
    if ( strcmp(picked, "File/Open map...") == 0 )
    {
        char start_path[256] = "/home/auralius/Pictures/";
        const char *file_name = NULL;
        file_name = fl_file_chooser("Open PNG File", "*.png", start_path);
        if (file_name == NULL)
            return;

        if (m_world)
        {
            delete m_image;
            delete m_world;
        }

        // Load the map image file to memory
        m_world = new CWorld(file_name);
        m_image = new Fl_RGB_Image(m_world->get_map(), m_world->get_width(), m_world->get_height(), 1);

        // Resize the window to follow the size of the map
        this->resize(0, 0, m_world->get_width(), m_world->get_height());
        m_canvas->resize(0, 25, m_world->get_width(), m_world->get_height()-25);
        m_menu->resize(0, 0, m_canvas->w(), 25);

        // Display the map on canvas
        m_canvas->image(m_image);

        // Update the area
        m_area = w() * h();
    }

    else if ( strcmp(picked, "File/Quit") == 0 )
    {
        exit(0);
    }

    else if ( strcmp(picked, "Simulation/Run") == 0 )
    {
        m_simulation_running = true;

        Fl_Menu_Item *m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Run");
        m->deactivate();
        m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Stop");
        m->activate();
    }

    else if ( strcmp(picked, "Simulation/Stop") == 0 )
    {
        m_simulation_running = false;

        Fl_Menu_Item *m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Run");
        m->activate();
        m = (Fl_Menu_Item *)m_menu->find_item("Simulation/Stop");
        m->deactivate();
    }

    else if ( strcmp(picked, "Simulation/Speed/Slow") == 0 )
    {
        m_simulation_speed= SLOW;
    }

    else if ( strcmp(picked, "Simulation/Speed/Medium") == 0 )
    {
        m_simulation_speed= MEDIUM;
    }

    else if ( strcmp(picked, "Simulation/Speed/Fast") == 0 )
    {
        m_simulation_speed= FAST;
    }

    else if ( strcmp(picked, "Simulation/Debug/Sensor Strike") == 0 )
    {
        m_canvas->set_drawing_mode(1);
    }

    else if ( strcmp(picked, "Simulation/Debug/Grid Memory") == 0 )
    {
        m_canvas->set_drawing_mode(2);
    }

    else if ( strcmp(picked, "Help/About...") == 0 )
    {
        fl_message("Azolla 0.0.1\nMobile Robot Simulator\nAuralius Manurung\n(auralius at gmx.com)");
    }
}