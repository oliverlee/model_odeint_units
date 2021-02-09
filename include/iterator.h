#pragma once

#include "type_traits.h"

#include <chrono>
#include <functional>
#include <iterator>
#include <utility>

namespace dyn {

template <class Iterator>
auto adapt_rangepair(std::pair<Iterator, Iterator> rp)
{
    struct range {
        decltype(rp.first) begin_;
        decltype(rp.second) end_;

        auto begin() { return begin_; }
        auto end() { return end_; }
    };

    return range{rp.first, rp.second};
}

template <class Model, template <class...> class Stepper, class Duration>
class owning_step_iterator {
    static_assert(stdx::is_specialization_of<Duration, std::chrono::duration>::value, "");

    using real_type = typename Model::real_type;
    using iterator_step_type = Duration;

    using system_type = decltype(Model::state_transition(std::declval<typename Model::input>()));
    using stepper_type = typename Model::template specialize_stepper<Stepper>;

  public:
    using iterator = owning_step_iterator<Model, Stepper, Duration>;

    using difference_type = std::ptrdiff_t;
    using value_type = std::pair<iterator_step_type, typename Model::state>;
    using pointer = std::add_pointer_t<value_type>;
    using reference = std::pair<std::add_lvalue_reference_t<iterator_step_type>,
                                std::add_lvalue_reference_t<typename Model::state>>;
    using iterator_category = std::input_iterator_tag;

    owning_step_iterator(const typename Model::state& x0,
                         const typename Model::input& u,
                         iterator_step_type span,
                         iterator_step_type step)
        : state_{x0}, system_{Model::state_transition(u)}, span_{span}, step_{step}
    {}

    owning_step_iterator() = default;

    auto operator++() noexcept -> iterator&
    {
        increment();
        return *this;
    }

    auto operator++(int) noexcept -> iterator
    {
        auto self = *this;
        increment();
        return self;
    }

    auto operator==(const owning_step_iterator& other) const noexcept -> bool
    {
        if (other.at_end()) {
            return at_end();
        }

        return (span_ == other.span_) && (step_ == other.step_) && (elapsed_ == other.elapsed_);
    }

    auto operator!=(const owning_step_iterator& other) const noexcept -> bool
    {
        return !(*this == other);
    }

    auto operator*() -> reference { return std::make_pair(std::ref(elapsed_), std::ref(state_)); }

  private:
    auto increment() -> void
    {
        stepper_type{}.do_step(system_, state_, elapsed_, step_);
        elapsed_ += step_;
    }

    auto at_end() const noexcept -> bool { return elapsed_ >= span_; }

    typename Model::state state_ = {};
    system_type system_ = {Model::state_transition({})};
    iterator_step_type span_ = {};
    iterator_step_type step_ = {};
    iterator_step_type elapsed_ = {};
};

template <class Model, template <class...> class Stepper, class Duration>
auto make_owning_step_range(const typename Model::state& x0,
                            const typename Model::input& u,
                            stdx::type_identity_t<Duration> span,
                            Duration step)
{
    return adapt_rangepair(
        std::make_pair(owning_step_iterator<Model, Stepper, Duration>(x0, u, span, step),
                       owning_step_iterator<Model, Stepper, Duration>()));
}

}  // namespace dyn
