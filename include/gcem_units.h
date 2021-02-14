#pragma once

#include "gcem.hpp"
#include "units.h"

namespace ode {
namespace math {

template <class AngleUnit>
constexpr auto sin(const AngleUnit angle) noexcept
{
    static_assert(units::traits::is_angle_unit<AngleUnit>::value, "");
    return units::dimensionless::scalar_t(
        gcem::sin(angle.template convert<units::angle::radian>().value()));
}

template <class AngleUnit>
constexpr auto cos(const AngleUnit angle) noexcept
{
    static_assert(units::traits::is_angle_unit<AngleUnit>::value, "");
    return units::dimensionless::scalar_t(
        gcem::cos(angle.template convert<units::angle::radian>().value()));
}

template <class AngleUnit>
constexpr auto tan(const AngleUnit angle) noexcept
{
    static_assert(units::traits::is_angle_unit<AngleUnit>::value, "");
    return units::dimensionless::scalar_t(
        gcem::tan(angle.template convert<units::angle::radian>().value()));
}

template <class ScalarUnit>
constexpr auto atan(const ScalarUnit x) noexcept
{
    static_assert(units::traits::is_dimensionless_unit<ScalarUnit>::value, "");
    return units::angle::radian_t(gcem::atan(x.value()));
}

}  // namespace math
}  // namespace ode
