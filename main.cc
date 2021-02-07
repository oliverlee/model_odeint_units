#include "boost/numeric/odeint.hpp"
#include "duration_adaptor.h"
#include "model.h"
#include "type_traits.h"
#include "unit_view.h"
#include "units.h"

#include <iostream>
#include <ratio>
#include <utility>

namespace {
namespace odeint = boost::numeric::odeint;

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

template <class Model, template <class...> class Stepper, class SpanDuration, class StepDuration>
auto make_step_range(typename Model::state& x,
                     const typename Model::input& u,
                     SpanDuration span,
                     StepDuration step)
{
    using Real = typename Model::real_type;
    using Duration = std::conditional_t<
        dyn::stdx::is_specialization_of<StepDuration, std::chrono::duration>::value,
        dyn::duration_adaptor<StepDuration, Real>,
        StepDuration>;

    return adapt_rangepair(odeint::make_const_step_time_range(
        Stepper<typename Model::state, Real, typename Model::deriv, Duration>{},
        Model::state_transition(u),
        x,
        Duration{0},
        Duration{span},
        Duration{step}));
}

template <class Model, template <class...> class Stepper, class Duration>
auto make_step_range(typename Model::state& x, const typename Model::input& u, Duration step)
{
    return make_step_range<Model, Stepper>(x, u, step, step);
}

}  // namespace

int main()
{
    using namespace units::literals;
    using namespace std::literals::chrono_literals;

    using Model = dyn::model<double, std::ratio<1105, 1000>, std::ratio<1738, 1000>>;

    std::cout << std::left << std::setprecision(3) << std::fixed;
    std::cout << Model{} << std::endl;

    const auto u = Model::input{0_mps_sq, 0.2_rad};
    auto x = Model::state{0_m, 0_m, 0_rad, 10_mps};

    for (auto s : make_step_range<Model, odeint::runge_kutta4>(x, u, 3s, 100ms)) {
        std::cout << "t=" << s.second << ": " << s.first << '\n';
    }

    // for (auto s : make_step_range<Model, odeint::runge_kutta4>(x, u, 100_ms)) {
    //    std::cout << "t=" << s.second << ": " << s.first << '\n';
    //}
}
