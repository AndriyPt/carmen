#include "robot_hardware.h"

int main()
{

    RobotHardware rh;
    rh.init();
    ControllerManager cm;

    Rate rate = 10;

    while (ros::ok)
    {
        rh.read();
        cm.update();
        rh.write();
        rate.sleep();
    }
    return 0;
}
