#pragma once

#include "iterator.h"
#include "state_space/vector.h"
#include "stepper.h"
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
    struct is_tf_odeint_form : std::false_type {};

    template <class T>
    struct is_tf_odeint_form<T,
                             tmp::void_t<decltype(std::declval<T>()(input())(
                                 state(), std::declval<deriv&>(), duration_type()))>>
        : std::true_type {};

    template <class, class = void>
    struct is_tf_state_space_form : std::false_type {};

    template <class T>
    struct is_tf_state_space_form<
        T,
        tmp::void_t<decltype(std::declval<T>()(state(), input(), duration_type()))>>
        : std::true_type {};

    static constexpr bool tf_is_odeint_form = is_tf_odeint_form<TransitionFunction>::value;
    static constexpr bool tf_is_state_space_form =
        is_tf_state_space_form<TransitionFunction>::value;

    struct odeint_tf_tag {};
    struct state_space_tf_tag {};

    using transfer_function_form_tag =
        std::conditional_t<tf_is_odeint_form, odeint_tf_tag, state_space_tf_tag>;

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
                                       duration_type
#ifdef BOOST_NUMERIC_ODEINT_HPP_INCLUDED
                                       ,
                                       boost::numeric::odeint::vector_space_algebra
#endif  // BOOST_NUMERIC_ODEINT_HPP_INCLUDED
                                       >;

    template <class T,
              class = std::enable_if_t<std::is_convertible<T, transition_function_type>::value>>
    constexpr system(T&& t) : tf_{std::forward<T>(t)}
    {}

    template <template <class...> class Stepper, class IntegrationStep>
    constexpr auto integrate_range(const state& x0,
                                   const input& u,
                                   tmp::type_identity_t<IntegrationStep> span,
                                   IntegrationStep step) const
    {
        using SpecializedStepper = specialize_stepper<Stepper>;

        return make_owning_step_range<SpecializedStepper>(
            adapt_transfer_function(u, stepper::stepper_tag<SpecializedStepper>{}), x0, span, step);
    }

    template <template <class...> class Stepper, class IntegrationStep>
    constexpr auto integrate(const state& x0, const input& u, IntegrationStep dt) const -> state
    {
        using SpecializedStepper = specialize_stepper<Stepper>;

        return do_step<SpecializedStepper>(x0, u, dt, stepper::stepper_tag<SpecializedStepper>{});
    }

  private:
    template <class Stepper, class IntegrationStep>
    auto do_step(state x, const input& u, IntegrationStep dt, stepper::odeint_tag) const -> state
    {
        Stepper{}.do_step(
            adapt_transfer_function(u, transfer_function_form_tag{}), x, IntegrationStep{}, dt);

        return x;
    }

    template <class Stepper, class IntegrationStep>
    constexpr auto
    do_step(const state& x0, const input& u, IntegrationStep dt, stepper::state_space_tag tag) const
        -> state
    {
        return Stepper{}.step(adapt_transfer_function(u, tag), x0, IntegrationStep{}, dt);
    }

    auto adapt_transfer_function(const input& u, odeint_tf_tag) const { return tf_(u); }

    auto adapt_transfer_function(const input& u, state_space_tf_tag) const
    {
        return [this, u](const auto& x, auto& dxdt, auto t) { dxdt = tf_(x, u, t); };
    }

    auto adapt_transfer_function(const input& u, stepper::odeint_tag) const
    {
        return adapt_transfer_function(u, odeint_tf_tag{});
    }

    constexpr auto adapt_transfer_function(const input& u, stepper::state_space_tag) const
    {
        struct standard_form {
            constexpr auto operator()(duration_type t, const state& x) const -> deriv
            {
                return tf(x, u, t);
            }

            const transition_function_type& tf;
            input u;
        };

        return standard_form{tf_, u};
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
