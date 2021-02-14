#pragma once

#include "stepper.h"
#include "type_traits.h"

#include <chrono>
#include <functional>
#include <iterator>
#include <utility>

namespace ode {

template <class Iterator>
constexpr auto adapt_rangepair(std::pair<Iterator, Iterator> rp)
{
    struct range {
        Iterator begin_;
        Iterator end_;

        constexpr auto begin() { return begin_; }
        constexpr auto end() { return end_; }
    };

    return range{rp.first, rp.second};
}

template <class Stepper, class System, class State, class StepDuration>
class owning_step_iterator {
    static_assert(tmp::is_specialization_of<StepDuration, std::chrono::duration>::value, "");

    using stepper_type = Stepper;
    using system_type = System;
    using state_type = State;
    using iterator_step_type = StepDuration;

  public:
    using iterator = owning_step_iterator;

    using difference_type = std::ptrdiff_t;
    using value_type = std::pair<iterator_step_type, state_type>;
    using pointer = std::add_pointer_t<value_type>;
    using reference = std::pair<std::add_lvalue_reference_t<iterator_step_type>,
                                std::add_lvalue_reference_t<state_type>>;
    using iterator_category = std::input_iterator_tag;

    constexpr owning_step_iterator(system_type sys,
                                   state_type x0,
                                   iterator_step_type span,
                                   iterator_step_type step)
        : system_{std::move(sys)}, state_{std::move(x0)}, span_{span}, step_{step}
    {}

    constexpr owning_step_iterator(system_type sys) : system_{std::move(sys)} {}

    auto operator++() noexcept -> iterator&
    {
        increment(stepper::stepper_tag<Stepper>{});
        return *this;
    }

    auto operator++(int) noexcept -> iterator
    {
        auto self = *this;
        increment(stepper::stepper_tag<Stepper>{});
        return self;
    }

    constexpr auto operator==(const owning_step_iterator& other) const noexcept -> bool
    {
        if (other.at_end()) {
            return at_end();
        }

        return (span_ == other.span_) && (step_ == other.step_) && (elapsed_ == other.elapsed_);
    }

    constexpr auto operator!=(const owning_step_iterator& other) const noexcept -> bool
    {
        return !(*this == other);
    }

    constexpr auto operator*() -> reference
    {
        return std::make_pair(std::ref(elapsed_), std::ref(state_));
    }

  private:
    auto increment(stepper::odeint_tag) -> void
    {
        stepper_type{}.do_step(system_, state_, elapsed_, step_);
        elapsed_ += step_;
    }

    auto increment(stepper::state_space_tag) -> void
    {
        state_ = stepper_type{}.step(system_, state_, elapsed_, step_);
        elapsed_ += step_;
    }

    constexpr auto at_end() const noexcept -> bool { return elapsed_ >= span_; }

    system_type system_;
    state_type state_ = {};
    iterator_step_type span_ = {};
    iterator_step_type step_ = {};
    iterator_step_type elapsed_ = {};
};

template <class Stepper, class System, class State, class StepDuration>
constexpr auto make_owning_step_range(const System& sys,
                                      const State& x0,
                                      tmp::type_identity_t<StepDuration> span,
                                      StepDuration step)
{
    return adapt_rangepair(std::make_pair(
        owning_step_iterator<Stepper, System, State, StepDuration>(sys, x0, span, step),
        owning_step_iterator<Stepper, System, State, StepDuration>(sys)));
}

template <class Model,
          template <class...>
          class Stepper,
          class StepDuration,
          class = std::enable_if_t<std::is_void<tmp::void_t<
              typename Model::template specialize_stepper<Stepper>,
              decltype(Model::state_transition(std::declval<typename Model::input>()))>>::value>>
auto make_owning_step_range(const typename Model::state& x0,
                            const typename Model::input& u,
                            tmp::type_identity_t<StepDuration> span,
                            StepDuration step)
{
    return make_owning_step_range<typename Model::template specialize_stepper<Stepper>>(
        Model::state_transition(u), x0, span, step);
}

}  // namespace ode
