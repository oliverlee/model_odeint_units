#include "boost/numeric/odeint.hpp"
#include "ode/iterator.h"
#include "ode/odeint/model.h"
#include "units.h"

#include <iostream>
#include <ratio>

int main()
{
    using namespace units::literals;
    using namespace std::literals::chrono_literals;
    namespace odeint = boost::numeric::odeint;

    using Model = ode::odeint::model<double, std::ratio<1105, 1000>, std::ratio<1738, 1000>>;

    std::cout << std::left << std::setprecision(3) << std::fixed;
    std::cout << Model{} << std::endl;

    for (auto result : ode::make_owning_step_range<Model, odeint::runge_kutta4>(
             {0_m, 0_m, 0_rad, 10_mps}, {0_mps_sq, 0.2_rad}, 3s, 100ms)) {
        std::cout << units::time::second_t{result.first} << ": " << result.second << std::endl;
    }
}
