#include "gcem_units.h"
#include "iterator.h"
#include "state_space/system.h"
#include "state_space/vector.h"
#include "stepper.h"
#include "units.h"

#include <array>
#include <chrono>
#include <iostream>
#include <utility>

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

template <class System, class Duration, std::size_t N>
struct invoker {
    template <std::size_t I = N, std::enable_if_t<I == 0, bool> = true>
    constexpr auto operator()() const
    {
        return x;
    }

    template <std::size_t I = N, std::enable_if_t<I != 0, bool> = true>
    constexpr auto operator()() const
    {
        return invoker<System, Duration, I - 1>{
            system, system.template integrate<dyn::stepper::runge_kutta4>(x, u, dt), u, dt}();
    }

    const System& system;
    state x;
    input u;
    Duration dt;
};

template <class System, class Duration, std::size_t... Is>
constexpr auto make_trajectory_impl(
    const System& sys, const state& x0, const input& u, Duration dt, std::index_sequence<Is...>)
    -> std::array<std::pair<Duration, state>, sizeof...(Is)>
{
    return {std::make_pair(Is * dt, invoker<System, Duration, Is>{sys, x0, u, dt}())...};
}

template <class SpanType,
          std::size_t SpanValue,
          class StepType,
          std::size_t StepValue,
          class System>
constexpr auto make_trajectory(const System& sys, const state& x0, const input& u)
{
    static_assert(dyn::tmp::is_specialization_of<SpanType, std::chrono::duration>::value, "");
    static_assert(dyn::tmp::is_specialization_of<StepType, std::chrono::duration>::value, "");

    constexpr auto dt = StepType{StepValue};
    constexpr auto steps = SpanType{SpanValue} / dt;
    static_assert(steps >= 0, "");

    return make_trajectory_impl(sys, x0, u, dt, std::make_index_sequence<steps>{});
}

constexpr auto trajectory =
    make_trajectory<std::chrono::seconds, 3, std::chrono::milliseconds, 100>(
        kinematic_bicycle, {0_m, 0_m, 0_rad, 10_mps}, {0_mps_sq, 0.2_rad});

}  // namespace

int main()
{
    for (const auto& x : trajectory) {
        std::cout << units::time::second_t{x.first} << ": " << x.second << std::endl;
    }

    return 0;
}
