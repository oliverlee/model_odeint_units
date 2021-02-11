#pragma once

#include "boost/numeric/odeint.hpp"
#include "type_traits.h"
#include "unit_proxy.h"
#include "units.h"

#include <array>
#include <cmath>
#include <ostream>
#include <ratio>

namespace dyn {

namespace detail {

template <bool InverseTime>
using time_base =
    std::conditional_t<InverseTime, units::inverse<units::time::second>, units::time::second>;

template <class BaseUnit, int DerivOrder, class Real>
using unit_with_deriv_order = units::unit_t<
    tmp::rebind_outer<
        tmp::push_front<BaseUnit, tmp::repeat<tmp::abs(DerivOrder), time_base<(DerivOrder > 0)>>>,
        tmp::list,
        units::compound_unit>,
    Real>;

}  // namespace detail

/// Kinematic Bicycle Model
/// Kong 2015 Kinematic
/// @tparam Real type
/// @tparam Distance from center of mass to front axle, as a specialization of `std::ratio` [m]
/// @tparam Distance from center of mass to rear axle, as a specialization of `std::ratio` [m]
/// @note Floating-point types cannot be non-type template parameters until C++20.
template <class Real, class Lf, class Lr>
struct model {
  private:
    template <class... Ts>
    using unit_t = units::unit_t<Ts...>;

    using second = units::time::second;
    using meter = units::length::meter;
    using meters_per_second = units::velocity::meters_per_second;
    using meters_per_second_squared = units::acceleration::meters_per_second_squared;
    using radian = units::angle::radian;
    using radians_per_second = units::angular_velocity::radians_per_second;

  public:
    using real_type = Real;

    using duration_type = unit_t<second, real_type>;
    using length_type = unit_t<meter, real_type>;
    using velocity_type = unit_t<meters_per_second, real_type>;
    using acceleration_type = unit_t<meters_per_second_squared, real_type>;
    using angle_type = unit_t<radian, real_type>;
    using angular_rate_type = unit_t<radians_per_second, real_type>;

    /// Distance from center of mass to front axle
    static constexpr length_type lf{real_type{Lf::num} / real_type{Lf::den}};
    /// Distance from center of mass to rear axle
    static constexpr length_type lr{real_type{Lr::num} / real_type{Lr::den}};

    template <int DerivOrder>
    struct state_with_deriv_order {
        using model_type = model<Real, Lf, Lr>;
        using real_type = model_type::real_type;
        using duration_type = model_type::duration_type;

        static constexpr auto deriv_order = DerivOrder;

        template <int NewOrder>
        using rebind = state_with_deriv_order<NewOrder>;

        using x_type =
            detail::unit_with_deriv_order<typename length_type::unit_type, deriv_order, real_type>;
        using y_type =
            detail::unit_with_deriv_order<typename length_type::unit_type, deriv_order, real_type>;
        using yaw_type =
            detail::unit_with_deriv_order<typename angle_type::unit_type, deriv_order, real_type>;
        using v_type = detail::
            unit_with_deriv_order<typename velocity_type::unit_type, deriv_order, real_type>;

        /// X-coordinate of center of mass w.r.t inertia frame
        x_type x;

        /// Y-coordinate of center of mass w.r.t inertia frame
        y_type y;

        /// Yaw angle (inertial heading)
        yaw_type yaw;

        /// Velocity of center of mass
        v_type v;

        auto operator+=(const state_with_deriv_order& s) -> state_with_deriv_order&
        {
            x += s.x;
            y += s.y;
            yaw += s.yaw;
            v += s.v;

            return *this;
        }

        auto operator*=(const real_type& a) -> state_with_deriv_order&
        {
            const auto k = units::unit_t<units::dimensionless::scalar, real_type>{a};

            x *= k;
            y *= k;
            yaw *= k;
            v *= k;

            return *this;
        }
    };

    using state = state_with_deriv_order<0>;
    using deriv = state_with_deriv_order<1>;

    struct input {
        using model_type = model<Real, Lf, Lr>;

        /// Acceleration of center of mass in the same direction as velocity [m/s^2]
        acceleration_type a;

        /// Front steering angle [rad]
        angle_type deltaf;
    };

    template <template <class...> class Stepper>
    using specialize_stepper = Stepper<state,
                                       real_type,
                                       deriv,
                                       duration_type,
                                       boost::numeric::odeint::vector_space_algebra>;

    /// Vehicle course, relative to yaw
    static auto course(angle_type deltaf) -> angle_type
    {
        return units::math::atan(lr / (lf + lr) * units::math::tan(deltaf));
    }

    static auto state_transition(input u)
    {
        return [u](const state& x, deriv& dxdt, duration_type /* t */) {
            const auto beta = course(u.deltaf);

            dxdt.x = x.v * units::math::cos(x.yaw + beta);
            dxdt.y = x.v * units::math::sin(x.yaw + beta);
            dxdt.yaw = x.v / lr * units::math::sin(beta) * angle_type{1};
            dxdt.v = u.a;
        };
    }
};

template <class Real, class Lf, class Lr>
constexpr typename model<Real, Lf, Lr>::length_type model<Real, Lf, Lr>::lf;
template <class Real, class Lf, class Lr>
constexpr typename model<Real, Lf, Lr>::length_type model<Real, Lf, Lr>::lr;

template <class State>
auto operator+(const State& s1, const State& s2) -> State
{
    auto s3 = s1;
    return s3 += s2;
}

template <class State>
auto operator*(const typename State::real_type& a, const State& s) -> State
{
    auto s2 = s;
    return s2 *= a;
}

template <class State>
auto operator*(const typename State::duration_type& a, const State& s) ->
    typename State::template rebind<State::deriv_order - 1>
{
    return {a * s.x, a * s.y, a * s.yaw, a * s.v};
}

template <class Model>
auto operator<<(std::ostream& os, const Model&)
    -> std::enable_if_t<tmp::is_specialization_of<Model, model>::value, std::ostream&>
{
    return os << "model (" << Model::lf << ", " << Model::lr << ")";
}

template <class State>
auto operator<<(std::ostream& os, const State& s)
    -> std::enable_if_t<tmp::is_specialization_of<typename State::model_type, model>::value,
                        std::ostream&>
{
    return os << "{" << s.x << ", " << s.y << ", " << s.yaw << ", " << s.v << "}";
}

}  // namespace dyn
