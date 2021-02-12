#include "gcem_units.h"
#include "state_space/vector.h"
#include "units.h"

#include <chrono>
#include <iostream>

namespace {

using namespace units::literals;
using namespace std::literals::chrono_literals;

constexpr auto lf = 1.105_m;
constexpr auto lr = 1.738_m;

constexpr auto beta = dyn::math::atan(lr / (lf + lr) * dyn::math::tan(0.2_rad));

using state = dyn::state_space::vector<struct x,
                                       units::length::meter_t,
                                       struct y,
                                       units::length::meter_t,
                                       struct yaw,
                                       units::angle::radian_t,
                                       struct v,
                                       units::velocity::meters_per_second_t>;

using deriv = state::derivative<1>;

struct f {
    constexpr auto operator()(const state& sx, deriv& dxdt, typename state::duration_type) -> void
    {
        dxdt.get<x>() = sx.get<v>() * dyn::math::cos(sx.get<yaw>() + beta);
        dxdt.get<y>() = sx.get<v>() * dyn::math::sin(sx.get<yaw>() + beta);
        dxdt.get<yaw>() = sx.get<v>() / lr * dyn::math::sin(beta) * 1_rad;
        dxdt.get<v>() = 0_mps_sq;
    }
};

constexpr auto calc_deriv(const state& x) -> deriv
{
    auto dxdt = deriv{};

    f{}(x, dxdt, 0s);

    return dxdt;
}

}  // namespace

int main()
{
    constexpr auto dxdt = calc_deriv({0_m, 0_m, 0_rad, 10_mps});

    std::cout << "deriv: " << std::endl;
    std::cout << "x: " << dxdt.get<x>() << std::endl;
    std::cout << "y: " << dxdt.get<y>() << std::endl;
    std::cout << "yaw: " << dxdt.get<yaw>() << std::endl;
    std::cout << "v: " << dxdt.get<v>() << std::endl;

    return 0;
}
