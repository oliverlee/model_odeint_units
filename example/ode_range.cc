#include "ode/iterator.h"
#include "ode/state_space/system.h"
#include "ode/state_space/vector.h"
#include "ode/stepper.h"
#include "units.h"

#include <chrono>
#include <iostream>

namespace {

using namespace units::literals;
using namespace std::literals::chrono_literals;

using state = ode::state_space::vector<struct x,
                                       units::length::meter_t,
                                       struct y,
                                       units::length::meter_t,
                                       struct yaw,
                                       units::angle::radian_t,
                                       struct v,
                                       units::velocity::meters_per_second_t>;

using input = ode::state_space::vector<struct a,
                                       units::acceleration::meters_per_second_squared_t,
                                       struct deltaf,
                                       units::angle::radian_t>;
using deriv = state::derivative<>;

const auto kinematic_bicycle = ode::state_space::make_system<state, input>(
    [](const state& sx, const input& u, units::time::second_t t) -> deriv {
        (void)t;

        constexpr auto lf = 1.105_m;
        constexpr auto lr = 1.738_m;

        const auto beta =
            units::math::atan(lr / (lf + lr) * units::math::tan(u.template get<deltaf>()));

        return {sx.template get<v>() * units::math::cos(sx.template get<yaw>() + beta),
                sx.template get<v>() * units::math::sin(sx.template get<yaw>() + beta),
                sx.template get<v>() / lr * units::math::sin(beta) * 1_rad,
                u.template get<a>()};
    }

);

}  // namespace

int main()
{
    for (const auto result : kinematic_bicycle.integrate_range<ode::stepper::runge_kutta4>(
             {0_m, 0_m, 0_rad, 10_mps}, {0_mps_sq, 0.2_rad}, 3s, 100ms)) {
        std::cout << units::time::second_t{result.first} << ": " << result.second << std::endl;
    }

    return 0;
}
