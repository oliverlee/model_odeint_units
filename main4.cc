#include "gcem_units.h"
#include "iterator.h"
#include "state_space/system.h"
#include "state_space/vector.h"
#include "stepper.h"
#include "units.h"

#include <chrono>
#include <iostream>

namespace {

using namespace units::literals;
using namespace std::literals::chrono_literals;

using state = dyn::state_space::vector<struct x,
                                       units::length::meter_t,
                                       struct y,
                                       units::length::meter_t,
                                       struct yaw,
                                       units::angle::radian_t,
                                       struct v,
                                       units::velocity::meters_per_second_t>;

using input = dyn::state_space::vector<struct a,
                                       units::acceleration::meters_per_second_squared_t,
                                       struct deltaf,
                                       units::angle::radian_t>;
using deriv = state::derivative<1>;

struct f {
    constexpr auto operator()(const state& sx, const input& u, state::duration_type t) const
        -> deriv
    {
        (void)t;

        constexpr auto lf = 1.105_m;
        constexpr auto lr = 1.738_m;

        const auto beta =
            dyn::math::atan(lr / (lf + lr) * dyn::math::tan(u.template get<deltaf>()));

        return {sx.template get<v>() * dyn::math::cos(sx.template get<yaw>() + beta),
                sx.template get<v>() * dyn::math::sin(sx.template get<yaw>() + beta),
                sx.template get<v>() / lr * dyn::math::sin(beta) * 1_rad,
                u.template get<a>()};
    }
};

constexpr auto kinematic_bicycle = dyn::state_space::make_system<state, input>(f{});

constexpr auto trajectory =
    kinematic_bicycle.integrate_trajectory<dyn::stepper::runge_kutta4,
                                           std::chrono::seconds,
                                           3,
                                           std::chrono::milliseconds,
                                           100>({0_m, 0_m, 0_rad, 10_mps}, {0_mps_sq, 0.2_rad});

}  // namespace

int main()
{
    for (const auto& x : trajectory) {
        std::cout << units::time::second_t{x.first} << ": " << x.second << std::endl;
    }

    return 0;
}
