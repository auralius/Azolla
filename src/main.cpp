//
// auralius (auralius@gmx.com)
//

#include "azolla/azolla.H"

class CMyRobot : public CAzolla
{
public:
    CMyRobot(CSimulationWindow *w)
    :CAzolla(w)
    {

    }

    virtual void az_sim_fn()
    {
        az_step();

        double front = az_get_sensor_data(0);
        if (front > 0.1) {
            az_set_lspeed(0.1);
            az_set_rspeed(0.1);
        }
        else
            az_set_stop();

    }
};

///////////////////////////////////////////////////////////////////////////////
// MAIN
///////////////////////////////////////////////////////////////////////////////

int main()
{
    CSimulationWindow win(600, 600,"azolla1");
    win.color(FL_GRAY);
    win.end();
    win.show();

    CMyRobot robot(&win);

    return(Fl::run());
}
