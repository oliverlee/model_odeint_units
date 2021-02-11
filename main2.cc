#include "state_space/vector.h"
#include "units.h"

#include <iostream>
#include <tuple>

int main()
{
    using namespace units::literals;

    using state = dyn::state_space::
        vector<struct x, units::length::meter_t, struct v, units::velocity::meters_per_second_t>;

    auto s = state{};

    s.get<x>() = 1_m;
    s.get<v>() = 2_mps;

    std::cout << "x: " << s.get<x>() << std::endl;
    std::cout << "v: " << s.get<v>() << std::endl;

    return 0;
}
