#pragma once

#include "type_traits.h"

#include <ostream>
#include <ratio>

namespace dyn {

/// Kinematic Bicycle Model
/// Kong 2015 Kinematic
/// @tparam Real type
/// @tparam Distance from center of mass to front axle, as a specialization of `std::ratio` [m]
/// @tparam Distance from center of mass to rear axle, as a specialization of `std::ratio` [m]
/// @note Floating-point types cannot be non-type template parameters until C++20.
template <class Real, class Lf, class Lr>
struct model {
    static constexpr auto lf = Real{Lf::num} / Real{Lf::den};
    static constexpr auto lr = Real{Lr::num} / Real{Lr::den};
};

template <class Model>
auto operator<<(std::ostream& os, const Model&)
    -> std::enable_if_t<stdx::is_specialization_of<Model, model>::value, std::ostream&>
{
    return os << "model m: (" << Model::lf << ", " << Model::lr << ")";
}

}  // namespace dyn
