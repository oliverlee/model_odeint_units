#pragma once

#include "type_traits.h"
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
    /// Distance from center of mass to front axle [m]
    static constexpr auto lf = Real{Lf::num} / Real{Lf::den};
    /// Distance from center of mass to rear axle [m]
    static constexpr auto lr = Real{Lr::num} / Real{Lr::den};

    struct state : std::array<Real, 4> {
        using model_type = model<Real, Lf, Lr>;

        constexpr state() = default;
        constexpr state(Real x, Real y, Real yaw, Real v) : std::array<Real, 4>{x, y, yaw, v} {}

        /// X-coordinate of center of mass w.r.t inertia frame [m]
        constexpr auto x() noexcept -> Real& { return x_impl(this); }
        constexpr auto x() const noexcept -> const Real& { return x_impl(this); }

        /// Y-coordinate of center of mass w.r.t inertia frame [m]
        constexpr auto y() noexcept -> Real& { return y_impl(this); }
        constexpr auto y() const noexcept -> const Real& { return y_impl(this); }

        /// Yaw angle (inertial heading) [rad]
        constexpr auto yaw() noexcept -> Real& { return yaw_impl(this); }
        constexpr auto yaw() const noexcept -> const Real& { return yaw_impl(this); }

        /// Velocity of center of mass [m/s]
        constexpr auto v() noexcept -> Real& { return v_impl(this); }
        constexpr auto v() const noexcept -> const Real& { return v_impl(this); }

      private:
        template <class Self>
        static constexpr auto x_impl(Self self) noexcept -> auto&
        {
            return self->at(0);
        }
        template <class Self>
        static constexpr auto y_impl(Self self) noexcept -> auto&
        {
            return self->at(1);
        }
        template <class Self>
        static constexpr auto yaw_impl(Self self) noexcept -> auto&
        {
            return self->at(2);
        }
        template <class Self>
        static constexpr auto v_impl(Self self) noexcept -> auto&
        {
            return self->at(3);
        }
    };

    struct input {
        /// Acceleration of center of mass in the same direction as velocity [m/s^2]
        units::unit_t<units::acceleration::meters_per_second_squared, Real> a;

        /// Front steering angle [rad]
        units::unit_t<units::angle::radian, Real> deltaf;
    };

    /// Vehicle course, relative to yaw
    static auto course(Real deltaf) -> Real { return std::atan(lr / (lf + lr) * std::tan(deltaf)); }

    static auto state_transition(input u)
    {
        return [u](const state& x, state& dxdt, Real /* t */) {
            const auto beta = course(u.deltaf.template to<Real>());

            dxdt.x() = x.v() * cos(x.yaw() + beta);
            dxdt.y() = x.v() * sin(x.yaw() + beta);
            dxdt.yaw() = x.v() / lr * sin(beta);
            dxdt.v() = u.a.template to<Real>();
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
