#pragma once

#include "boost/numeric/odeint.hpp"
#include "iterator.h"
#include "state_space/vector.h"
#include "type_traits.h"
#include "units.h"

namespace dyn {
namespace state_space {

template <class State, class Input, class TransitionFunction>
class system {
  public:
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
        "The output of a `TransitionFunction` must have a signature similar to f(const state&, "
        "deriv&, duration_type`.");

    using transition_function_type = TransitionFunction;

    template <template <class...> class Stepper>
    using specialize_stepper = Stepper<state,
                                       real_type,
                                       deriv,
                                       duration_type,
                                       boost::numeric::odeint::vector_space_algebra>;

    template <class T,
              class = std::enable_if_t<std::is_convertible<T, transition_function_type>::value>>
    constexpr system(T&& t) : tf_{std::forward<T>(t)}
    {}

    constexpr auto state_transition(const input& u) const { return tf_(u); }

    template <template <class...> class Stepper, class IntegrationStep>
    auto integrate_range(const state& x0,
                         const input& u,
                         tmp::type_identity_t<IntegrationStep> span,
                         IntegrationStep step) const
    {

        return make_owning_step_range<Stepper>(*this, x0, u, span, step);
    }

  private:
    transition_function_type tf_;
};

namespace detail {

template <class State, class Input, class TransitionFunction>
constexpr auto make_system(State, Input, TransitionFunction&& tf)
    -> system<std::decay_t<State>, std::decay_t<Input>, std::decay_t<TransitionFunction>>
{
    return system<std::decay_t<State>, std::decay_t<Input>, std::decay_t<TransitionFunction>>{
        std::forward<TransitionFunction>(tf)};
}

}  // namespace detail

template <class State, class Input, class TransitionFunction>
constexpr auto make_system(TransitionFunction&& tf)
    -> system<std::decay_t<State>, std::decay_t<Input>, std::decay_t<TransitionFunction>>
{
    return detail::make_system(State{}, Input{}, std::forward<TransitionFunction>(tf));
}

}  // namespace state_space
}  // namespace dyn
