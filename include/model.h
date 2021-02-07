#pragma once

#include "type_traits.h"
#include "unit_view.h"
#include "units.h"

#include <array>
#include <cmath>
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
    using length_type = units::unit_t<units::length::meter, Real>;
    using angle_type = units::unit_t<units::angle::radian, Real>;
    using velocity_type = units::unit_t<units::velocity::meters_per_second, Real>;
    using acceleration_type = units::unit_t<units::acceleration::meters_per_second_squared, Real>;

    /// Distance from center of mass to front axle [m]
    static constexpr auto lf = Real{Lf::num} / Real{Lf::den};
    /// Distance from center of mass to rear axle [m]
    static constexpr auto lr = Real{Lr::num} / Real{Lr::den};

    struct state : std::array<Real, 4> {
        using model_type = model<Real, Lf, Lr>;

        constexpr state() = default;
        constexpr state(length_type x, length_type y, angle_type yaw, velocity_type v)
            : std::array<Real, 4>{x.value(), y.value(), yaw.value(), v.value()}
        {}

        /// X-coordinate of center of mass w.r.t inertia frame
        constexpr auto x() noexcept -> unit_view<length_type> { return this->operator[](0); }
        constexpr auto x() const noexcept -> length_type
        {
            return length_type{this->operator[](0)};
        }

        /// Y-coordinate of center of mass w.r.t inertia frame
        constexpr auto y() noexcept -> unit_view<length_type> { return this->operator[](1); }
        constexpr auto y() const noexcept -> length_type
        {
            return length_type{this->operator[](1)};
        }

        /// Yaw angle (inertial heading)
        constexpr auto yaw() noexcept -> unit_view<angle_type> { return this->operator[](2); }
        constexpr auto yaw() const noexcept -> angle_type
        {
            return angle_type{this->operator[](2)};
        }

        /// Velocity of center of mass
        constexpr auto v() noexcept -> unit_view<velocity_type> { return this->operator[](3); }
        constexpr auto v() const noexcept -> velocity_type
        {
            return velocity_type{this->operator[](3)};
        }
    };

    struct input {
        /// Acceleration of center of mass in the same direction as velocity [m/s^2]
        acceleration_type a;

        /// Front steering angle [rad]
        angle_type deltaf;
    };

    /// Vehicle course, relative to yaw
    static auto course(angle_type deltaf) -> angle_type
    {
        return units::math::atan(lr / (lf + lr) * units::math::tan(deltaf));
    }

    static auto state_transition(input u)
    {
        return [u](const state& x, state& dxdt, Real /* t */) {
            const auto beta = course(u.deltaf);

            dxdt[0] = (x.v() * units::math::cos(x.yaw() + beta)).value();
            dxdt[1] = (x.v() * units::math::sin(x.yaw() + beta)).value();
            dxdt[2] = (x.v() / lr * units::math::sin(beta)).value();
            dxdt[3] = u.a.value();
        };
    }
};

template <class Model>
auto operator<<(std::ostream& os, const Model&)
    -> std::enable_if_t<stdx::is_specialization_of<Model, model>::value, std::ostream&>
{
    return os << "model (" << Model::lf << ", " << Model::lr << ")";
}

template <class State>
auto operator<<(std::ostream& os, const State& s)
    -> std::enable_if_t<stdx::is_specialization_of<typename State::model_type, model>::value,
                        std::ostream&>
{
    return os << "state {" << s.x() << ", " << s.y() << ", " << s.yaw() << ", " << s.v() << "}";
}

}  // namespace dyn
