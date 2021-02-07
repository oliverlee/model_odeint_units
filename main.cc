#include "boost/numeric/odeint.hpp"
#include "model.h"
#include "unit_view.h"
#include "units.h"

#include <iostream>
#include <ratio>
#include <utility>
#include <vector>

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

template <class Model, template <class...> class Stepper>
auto make_step_range(typename Model::state& x,
                     const typename Model::input& u,
                     typename Model::duration_type t_span,
                     typename Model::duration_type dt)
{
    using Real = typename Model::real_type;

    return adapt_rangepair(odeint::make_const_step_time_range(
        Stepper<typename Model::state, Real, typename Model::deriv, Real>{},
        Model::state_transition(u),
        x,
        Real{0},
        t_span.template to<Real>(),
        dt.template to<Real>()));
}

}  // namespace

int main()
{
    using namespace units::literals;

    using Model = dyn::model<double, std::ratio<1105, 1000>, std::ratio<1738, 1000>>;

    std::cout << std::left << std::setprecision(3) << std::fixed;
    std::cout << Model{} << std::endl;

    const auto u = Model::input{0_mps_sq, 0.2_rad};
    auto x = Model::state{0_m, 0_m, 0_rad, 10_mps};

    for (auto s : make_step_range<Model, odeint::runge_kutta4>(x, u, 3_s, 100_ms)) {
        std::cout << "t=" << s.second << ": " << s.first << '\n';
    }
}
