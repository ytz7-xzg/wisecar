#include "common/config.hpp"
#include "control/lane_geometry.hpp"
#include <iostream>
#include <exception>

int main(int argc, char** argv) {
    try {
        const our_car::ControlConfig config = argc > 1 ?
            our_car::load_control_config(argv[1]) : our_car::ControlConfig();
        our_car::SteeringPD controller(config.pd);
        const our_car::GroundLine left(0,-50), right(0,50);
        const double targets[] = {0.5,0.4,0.6};
        std::cout << "Calculation only; no GPIO initialized.\n";
        for (double target : targets) {
            controller.reset();
            const our_car::HeadingError e = our_car::heading_error(left,right,target,100);
            int pwm = config.pd.center;
            if (!e.valid || !controller.compute(e.degrees,pwm)) return 1;
            std::cout << "target=" << target << " error_deg=" << e.degrees << " servo_pwm=" << pwm << '\n';
        }
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
