#include "control/lane_geometry.hpp"
#include <algorithm>
#include <cmath>

namespace our_car {
HeadingError heading_error(const GroundLine& left, const GroundLine& right,
                           double target, double lookahead) {
    HeadingError result;
    if (!left.valid || !right.valid || !std::isfinite(left.m) ||
        !std::isfinite(left.b) || !std::isfinite(right.m) || !std::isfinite(right.b) ||
        !std::isfinite(target) || target < 0 || target > 1 ||
        !std::isfinite(lookahead) || lookahead <= 0 || right.b <= left.b ||
        std::abs(left.m-right.m) > 1e-6) return result;
    const double b = (1.0-target)*left.b + target*right.b;
    const double k = right.m;
    const double norm = std::hypot(k,1.0);
    const double d = std::abs(b)/norm;
    const double y = std::hypot(d,lookahead)/norm;
    const double x = k*y+b;
    if (!std::isfinite(x) || !std::isfinite(y) || y <= 0) return result;
    result.degrees = std::atan2(x,y)*180.0/std::acos(-1.0);
    result.valid = std::isfinite(result.degrees);
    return result;
}
double lookahead_from_pwm(int pwm, int neutral, double span, double max_cm) {
    if (!std::isfinite(span) || span <= 0 || !std::isfinite(max_cm) || max_cm <= 0)
        return 0;
    const double delta = double(pwm)-double(neutral);
    return std::max(0.0,std::min(span,delta))/span*max_cm;
}
}  // namespace our_car
