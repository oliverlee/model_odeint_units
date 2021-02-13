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
                  "`Input` must be a specialization of `state_space::vector`.");
    static_assert(tmp::is_specialization_of<State, vector>::value,
                  "`State` must be a specialization of `state_space::vector`.");

    using input = Input;
    using state = State;
    using deriv = typename State::template derivative<>;
    using scalar_type = typename State::real_type;
    using duration_type = typename State::duration_type;

  private:
    template <class, class = void>
    struct is_odeint_form : std::false_type {};

    template <class T>
    struct is_odeint_form<T,
                          tmp::void_t<decltype(std::declval<T>()(input())(
                              state(), std::declval<deriv&>(), duration_type()))>>
        : std::true_type {};

    template <class, class = void>
    struct is_state_space_form : std::false_type {};

    template <class T>
    struct is_state_space_form<
        T,
        tmp::void_t<decltype(std::declval<T>()(state(), input(), duration_type()))>>
        : std::true_type {};

    static constexpr bool tf_is_odeint_form = is_odeint_form<TransitionFunction>::value;
    static constexpr bool tf_is_state_space_form = is_state_space_form<TransitionFunction>::value;

    struct odeint_tag {};
    struct state_space_tag {};

    using integrate_range_tag = std::conditional_t<tf_is_odeint_form, odeint_tag, state_space_tag>;

  public:
    static_assert(
        tf_is_odeint_form || tf_is_state_space_form,
        "The output of a `TransitionFunction` must have a signature similar to f(const state&, "
        "deriv&, duration_type` or must be callable with the signature f(const state&, const "
        "input&, duration_type) -> deriv.");

    using transition_function_type = TransitionFunction;

    template <template <class...> class Stepper>
    using specialize_stepper = Stepper<state,
                                       scalar_type,
                                       deriv,
                                       duration_type,
                                       boost::numeric::odeint::vector_space_algebra>;

    template <class T,
              class = std::enable_if_t<std::is_convertible<T, transition_function_type>::value>>
    constexpr system(T&& t) : tf_{std::forward<T>(t)}
    {}

    template <template <class...> class Stepper, class IntegrationStep>
    auto integrate_range(const state& x0,
                         const input& u,
                         tmp::type_identity_t<IntegrationStep> span,
                         IntegrationStep step) const
    {
        return integrate_range<Stepper, IntegrationStep>(x0, u, span, step, integrate_range_tag{});
    }

  private:
    template <template <class...> class Stepper, class IntegrationStep>
    auto integrate_range(const state& x0,
                         const input& u,
                         tmp::type_identity_t<IntegrationStep> span,
                         IntegrationStep step,
                         odeint_tag) const
    {
        return make_owning_step_range<specialize_stepper<Stepper>>(tf_(u), x0, span, step);
    }

    template <template <class...> class Stepper, class IntegrationStep>
    auto integrate_range(const state& x0,
                         const input& u,
                         tmp::type_identity_t<IntegrationStep> span,
                         IntegrationStep step,
                         state_space_tag) const
    {
        const auto f = [this, u](const auto& x, auto& dxdt, auto t) { dxdt = tf_(x, u, t); };
        return make_owning_step_range<specialize_stepper<Stepper>>(f, x0, span, step);
    }

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
