#include "gcem.hpp"
#include "state_space/vector.h"
#include "units.h"

#include <chrono>
#include <iostream>

namespace {

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

using namespace units::literals;
using namespace std::literals::chrono_literals;

constexpr auto lf = 1.105_m;
constexpr auto lr = 1.738_m;

constexpr auto beta = math::atan(lr / (lf + lr) * math::tan(0.2_rad));

using state = dyn::state_space::vector<struct x,
                                       units::length::meter_t,
                                       struct y,
                                       units::length::meter_t,
                                       struct yaw,
                                       units::angle::radian_t,
                                       struct v,
                                       units::velocity::meters_per_second_t>;

using deriv = state::derivative<1>;

struct f {
    constexpr auto operator()(const state& sx, deriv& dxdt, typename state::duration_type) -> void
    {
        dxdt.get<x>() = sx.get<v>() * math::cos(sx.get<yaw>() + beta);
        dxdt.get<y>() = sx.get<v>() * math::sin(sx.get<yaw>() + beta);
        dxdt.get<yaw>() = sx.get<v>() / lr * math::sin(beta) * 1_rad;
        dxdt.get<v>() = 0_mps_sq;
    }
};

constexpr auto calc_deriv(const state& x) -> deriv
{
    auto dxdt = deriv{};

    f{}(x, dxdt, 0s);

    return dxdt;
}

}  // namespace

int main()
{
    constexpr auto dxdt = calc_deriv({0_m, 0_m, 0_rad, 10_mps});

    std::cout << "deriv: " << std::endl;
    std::cout << "x: " << dxdt.get<x>() << std::endl;
    std::cout << "y: " << dxdt.get<y>() << std::endl;
    std::cout << "yaw: " << dxdt.get<yaw>() << std::endl;
    std::cout << "v: " << dxdt.get<v>() << std::endl;

    return 0;
}
