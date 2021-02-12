#include "state_space/vector.h"
#include "units.h"

#include <chrono>
#include <iostream>
#include <tuple>

int main()
{
    using namespace units::literals;

    using state = dyn::state_space::
        vector<struct x, units::length::meter_t, struct v, units::velocity::meters_per_second_t>;

    {
        constexpr auto s = state{1_m, 2_mps} * 2;

        std::cout << "state: " << std::endl;
        std::cout << "x: " << s.get<x>() << std::endl;
        std::cout << "v: " << s.get<v>() << std::endl;
    }


    {
        constexpr auto d =
            state::derivative<1>{1_mps, 2_mps_sq} + state::derivative<1>{4_mps, 3_mps_sq};

        std::cout << "derivative: " << std::endl;
        std::cout << "x: " << d.get<x>() << std::endl;
        std::cout << "v: " << d.get<v>() << std::endl;

        constexpr auto i = d * std::chrono::seconds{1};
        std::cout << "integral of derivative: " << std::endl;
        std::cout << "x: " << i.get<x>() << std::endl;
        std::cout << "v: " << i.get<v>() << std::endl;
    }

    return 0;
}
