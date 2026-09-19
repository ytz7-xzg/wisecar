#pragma once

namespace our_car {
// Ground coordinates: x=m*y+b, x right-positive, y forward-positive; lengths in cm.
// NOT the y=m*x+b image line returned by LaneDetector.
struct GroundLine {
    double m = 0, b = 0;
    bool valid = false;
    GroundLine() = default;
    GroundLine(double slope, double intercept) : m(slope), b(intercept), valid(true) {}
};
struct HeadingError { bool valid = false; double degrees = 0; };
HeadingError heading_error(const GroundLine& left, const GroundLine& right,
                           double target_fraction, double lookahead_cm);
// Legacy PWM-based heuristic. This does not estimate measured vehicle speed.
double lookahead_from_pwm(int pwm, int neutral = 10000,
                          double pwm_span = 3200.0, double max_cm = 122.0);
}  // namespace our_car
