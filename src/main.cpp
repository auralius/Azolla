//
// auralius (auralius@gmx.com)
//

#include "azolla/azolla.H"

int i = 0;

void my_sim(CAzolla *p)
{
    p->az_set_lspeed(4.7);
    p->az_set_rspeed(5);
    p->az_step();

    if (i == 100)
    {
        p->az_get_global_map1();
        p->az_update_odometer();
        p->az_log_sensor("test.txt");
        i = 0;
    }
    i++;
}

// MAIN
int main()
{
    CSimulationWindow win(600, 600,"azolla1");
    win.color(FL_GRAY);
    win.end();
    win.show();

    CAzolla robot(&win);
    robot.az_set_simulation_fn(my_sim);
    //robot.az_set_location(200,100,0);

    return(Fl::run());
}
