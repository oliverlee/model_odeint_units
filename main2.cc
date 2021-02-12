#include "boost/numeric/odeint.hpp"
#include "gcem_units.h"
#include "iterator.h"
#include "state_space/system.h"
#include "state_space/vector.h"
#include "units.h"

#include <chrono>
#include <iostream>

namespace {

using namespace units::literals;
using namespace std::literals::chrono_literals;

constexpr auto lf = 1.105_m;
constexpr auto lr = 1.738_m;

using state = dyn::state_space::vector<struct x,
                                       units::length::meter_t,
                                       struct y,
                                       units::length::meter_t,
                                       struct yaw,
                                       units::angle::radian_t,
                                       struct v,
                                       units::velocity::meters_per_second_t>;

using deriv = state::derivative<1>;

using input = dyn::state_space::vector<struct a,
                                       units::acceleration::meters_per_second_squared_t,
                                       struct deltaf,
                                       units::angle::radian_t>;

struct f {
    constexpr auto operator()(const state& sx, deriv& dxdt, typename state::duration_type) -> void
    {
        const auto beta = dyn::math::atan(lr / (lf + lr) * dyn::math::tan(u.get<deltaf>()));

        dxdt.get<x>() = sx.get<v>() * dyn::math::cos(sx.get<yaw>() + beta);
        dxdt.get<y>() = sx.get<v>() * dyn::math::sin(sx.get<yaw>() + beta);
        dxdt.get<yaw>() = sx.get<v>() / lr * dyn::math::sin(beta) * 1_rad;
        dxdt.get<v>() = u.get<a>();
    }

    input u;
};

struct state_transition {
    constexpr auto operator()(const input& u) -> f { return {u}; }
};

using KinematicBicycle = dyn::state_space::system<state, input, state_transition>;

}  // namespace

int main()
{
    for (auto result :
         dyn::make_owning_step_range<KinematicBicycle, boost::numeric::odeint::runge_kutta4>(
             {0_m, 0_m, 0_rad, 10_mps}, {0_mps_sq, 0.2_rad}, 3s, 100ms)) {
        // std::cout << units::time::second_t{result.first} << ": " << result.second << std::endl;
        std::cout << units::time::second_t{result.first} << std::endl;
    }

    return 0;
}
