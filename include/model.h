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
    using real_type = Real;

    using length_type = units::unit_t<units::length::meter, Real>;
    using velocity_type = units::unit_t<units::velocity::meters_per_second, Real>;
    using acceleration_type = units::unit_t<units::acceleration::meters_per_second_squared, Real>;
    using angle_type = units::unit_t<units::angle::radian, Real>;
    using angular_rate_type = units::unit_t<units::angular_velocity::radians_per_second, Real>;

    /// Distance from center of mass to front axle
    static constexpr length_type lf{Real{Lf::num} / Real{Lf::den}};
    /// Distance from center of mass to rear axle
    static constexpr length_type lr{Real{Lr::num} / Real{Lr::den}};

    struct state : std::array<Real, 4> {
        using model_type = model<Real, Lf, Lr>;

        constexpr state() = default;
        constexpr state(length_type x, length_type y, angle_type yaw, velocity_type v)
            : std::array<Real, 4>{x.value(), y.value(), yaw.value(), v.value()}
        {}

        /// X-coordinate of center of mass w.r.t inertia frame
        constexpr auto x() noexcept { return unit_view<length_type>{this->at(0)}; }
        constexpr auto x() const noexcept { return length_type{this->at(0)}; }

        /// Y-coordinate of center of mass w.r.t inertia frame
        constexpr auto y() noexcept { return unit_view<length_type>{this->at(1)}; }
        constexpr auto y() const noexcept { return length_type{this->at(1)}; }

        /// Yaw angle (inertial heading)
        constexpr auto yaw() noexcept { return unit_view<angle_type>{this->at(2)}; }
        constexpr auto yaw() const noexcept { return angle_type{this->at(2)}; }

        /// Velocity of center of mass
        constexpr auto v() noexcept { return unit_view<velocity_type>{this->at(3)}; }
        constexpr auto v() const noexcept { return velocity_type{this->at(3)}; }
    };

    struct deriv : std::array<Real, 4> {
        using model_type = model<Real, Lf, Lr>;

        constexpr deriv() = default;
        constexpr deriv(velocity_type x,
                        velocity_type y,
                        angular_rate_type yaw,
                        acceleration_type v)
            : std::array<Real, 4>{x.value(), y.value(), yaw.value(), v.value()}
        {}

        /// Time derivative of x-coordinate of center of mass w.r.t inertia frame
        constexpr auto x() noexcept { return unit_view<velocity_type>{this->at(0)}; }
        constexpr auto x() const noexcept { return velocity_type{this->at(0)}; }

        /// Time derivative of y-coordinate of center of mass w.r.t inertia frame
        constexpr auto y() noexcept { return unit_view<velocity_type>{this->at(1)}; }
        constexpr auto y() const noexcept { return velocity_type{this->at(1)}; }

        /// Time derivative of yaw angle (inertial heading)
        constexpr auto yaw() noexcept { return unit_view<angular_rate_type>{this->at(2)}; }
        constexpr auto yaw() const noexcept { return angular_rate_type{this->at(2)}; }

        /// Time derivative of velocity of center of mass
        constexpr auto v() noexcept { return unit_view<acceleration_type>{this->at(3)}; }
        constexpr auto v() const noexcept { return acceleration_type{this->at(3)}; }
    };

    struct input {
        using model_type = model<Real, Lf, Lr>;

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
        return [u](const state& x, deriv& dxdt, Real /* t */) {
            const auto beta = course(u.deltaf);

            dxdt.x() = x.v() * units::math::cos(x.yaw() + beta);
            dxdt.y() = x.v() * units::math::sin(x.yaw() + beta);
            dxdt.yaw() = x.v() / lr * units::math::sin(beta) * angle_type{1};
            dxdt.v() = u.a;
        };
    }
};

template <class Real, class Lf, class Lr>
constexpr typename model<Real, Lf, Lr>::length_type model<Real, Lf, Lr>::lf;
template <class Real, class Lf, class Lr>
constexpr typename model<Real, Lf, Lr>::length_type model<Real, Lf, Lr>::lr;

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
