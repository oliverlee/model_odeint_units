#pragma once

#include "boost/numeric/odeint.hpp"
#include "state_space/vector.h"
#include "type_traits.h"
#include "units.h"

namespace dyn {
namespace state_space {

template <class State, class Input, class TransitionFunction>
struct system {
    static_assert(tmp::is_specialization_of<Input, vector>::value,
                  "`State` must be a specialization of `state_space::vector`.");
    static_assert(tmp::is_specialization_of<State, vector>::value,
                  "`State` must be a specialization of `state_space::vector`.");

    using input = Input;
    using state = State;
    using deriv = typename State::template derivative<1>;
    using real_type = typename State::real_type;
    using duration_type = typename State::duration_type;

    static_assert(
        std::is_void<decltype(std::declval<TransitionFunction>()(std::declval<input>())(
            std::declval<state>(), std::declval<deriv&>(), std::declval<duration_type>()))>::value,
        "The output of a `TransitionFunction` must have a signature similar to f(const "
        "state_type&, deriv_type&, duration_type`.");

    using transition_function_type = TransitionFunction;

    template <template <class...> class Stepper>
    using specialize_stepper = Stepper<state,
                                       real_type,
                                       deriv,
                                       duration_type,
                                       boost::numeric::odeint::vector_space_algebra>;

    static constexpr auto state_transition(const input& u) { return transition_function_type{}(u); }
};

}  // namespace state_space
}  // namespace dyn
