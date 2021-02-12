#include "state_space/vector.h"
#include "units.h"

#include <iostream>
#include <tuple>

int main()
{
    using namespace units::literals;

    using state = dyn::state_space::
        vector<struct x, units::length::meter_t, struct v, units::velocity::meters_per_second_t>;

    {
        auto s = state{};

        s.get<x>() = 1_m;
        s.get<v>() = 2_mps;

        s *= 2;

        std::cout << "state: " << std::endl;
        std::cout << "x: " << s.get<x>() << std::endl;
        std::cout << "v: " << s.get<v>() << std::endl;
    }


    {
        auto d = state::derivative<1>{};

        d.get<x>() = 1_mps;
        d.get<v>() = 2_mps_sq;

        d += state::derivative<1>{4_mps, 3_mps_sq};

        std::cout << "derivative: " << std::endl;
        std::cout << "x: " << d.get<x>() << std::endl;
        std::cout << "v: " << d.get<v>() << std::endl;
    }

    return 0;
}
