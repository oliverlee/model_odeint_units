#include "boost/numeric/odeint.hpp"
#include "include/model.h"
#include "units.h"

#include <iostream>
#include <ratio>
#include <utility>
#include <vector>

namespace {

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

}  // namespace

int main()
{
    namespace odeint = boost::numeric::odeint;
    using namespace units::literals;

    using Model = dyn::model<double, std::ratio<1105, 1000>, std::ratio<1738, 1000>>;

    std::cout << std::left << std::setprecision(3) << std::fixed;
    std::cout << Model{} << std::endl;

    auto stepper = odeint::runge_kutta4<Model::state>{};
    const auto f = Model::state_transition(Model::input{0_mps_sq, 0.2_rad});
    auto x = Model::state{0.0, 0.0, 0.0, 10.0};

    for (auto s :
         adapt_rangepair(odeint::make_const_step_time_range(stepper, f, x, 0.0, 10.0, 0.1))) {
        std::cout << "t=" << s.second << ": " << s.first << '\n';
    }
}
