#include "boost/numeric/odeint.hpp"
#include "include/model.h"

#include <iostream>
#include <ratio>
#include <utility>
#include <vector>

namespace {

using Model = dyn::model<double, std::ratio<1105, 1000>, std::ratio<1738, 1000>>;

auto states = std::vector<std::pair<double, Model::state>>{};

struct Observer {
    void operator()(const Model::state& x, double t) { states.emplace_back(t, x); }
};

}  // namespace

int main()
{
    const auto f = Model::state_transition(Model::input{0.0, 0.2});

    auto x = Model::state{0.0, 0.0, 0.0, 10.0};
    boost::numeric::odeint::integrate(f, x, 0.0, 10.0, 0.1, Observer{});

    std::cout << std::left << std::setprecision(3) << std::fixed;

    std::cout << Model{} << std::endl;
    for (const auto& s : states) {
        std::cout << "t=" << s.first << ": " << s.second << '\n';
    }
}
