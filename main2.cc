#include "state_space/vector.h"
#include "units.h"

#include <tuple>

int main()
{

    using state = dyn::state_space::
        vector<struct x, units::length::meter_t, struct y, units::length::meter_t>;

    static_assert(
        std::is_same<units::length::meter_t, std::tuple_element_t<0, state::data_type>>::value, "");
    static_assert(
        std::is_same<units::length::meter_t, std::tuple_element_t<1, state::data_type>>::value, "");

    return 0;
}
