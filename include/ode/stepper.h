#pragma once

#include "ode/tmp/type_traits.h"

namespace ode {
namespace stepper {

template <class Function, class Time, class State, class = void>
struct is_function : std::false_type {};

template <class Function, class Time, class State>
struct is_function<Function,
                   Time,
                   State,
                   tmp::void_t<decltype(std::declval<Function>()(
                       std::declval<Time>(), std::declval<const State&>()))>> : std::true_type {};

template <class, class = void>
struct is_state_space_stepper : std::false_type {};

template <class T>
struct is_state_space_stepper<T, tmp::void_t<tmp::bool_constant<T::is_state_space_stepper>>>
    : tmp::bool_constant<T::is_state_space_stepper> {};

struct odeint_tag {};
struct state_space_tag {};

template <class T>
using stepper_tag =
    std::conditional_t<is_state_space_stepper<T>::value, state_space_tag, odeint_tag>;

template <class State, class Scalar, class Deriv, class StepDuration, class Unused = void>
struct runge_kutta4 {
    using state_type = State;
    using scalar_type = Scalar;
    using deriv_type = Deriv;
    using step_type = StepDuration;
    using timepoint_type = StepDuration;

    static constexpr bool is_state_space_stepper = true;

    template <class Function>
    static constexpr auto step(Function f, const state_type& x, timepoint_type t, step_type dt)
        -> std::enable_if_t<is_function<Function, timepoint_type, state_type>::value, state_type>
    {
        // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods

        const auto half_dt = dt / scalar_type{2};

        const auto k1 = f(t, x);
        const auto k2 = f(t + half_dt, x + half_dt * k1);
        const auto k3 = f(t + half_dt, x + half_dt * k2);
        const auto k4 = f(t + dt, x + dt * k3);

        return x + dt / scalar_type{6} * (k1 + scalar_type{2} * (k2 + k3) + k4);
    }
};

}  // namespace stepper
}  // namespace ode
