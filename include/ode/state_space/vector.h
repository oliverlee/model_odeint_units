#pragma once

#include "ode/tmp/type_mapping.h"
#include "ode/tmp/type_traits.h"
#include "units.h"

#include <iosfwd>
#include <tuple>

namespace ode {
namespace state_space {

namespace detail {

template <class Scalar>
struct elementwise_scalar_multiply {
    static_assert(units::traits::is_dimensionless_unit<Scalar>::value,
                  "`Scalar` must be a dimensionless unit.");

    template <class T>
    constexpr auto operator()(T& elem)
    {
        elem *= scalar;
    }

    const Scalar scalar;
};

using implicit_duration_type = units::time::second_t;

}  // namespace detail

template <class... Args>
class vector {
  private:
    using keys = tmp::skip<1, tmp::list<Args...>>;
    using values = tmp::skip<1, tmp::drop<1, tmp::list<Args...>>>;

    using key_index_mapping =
        tmp::rebind_outer<tmp::map<std::add_pointer, keys>, tmp::mapping::index_map>;

    template <class T>
    using enable_if_key = std::enable_if_t<key_index_mapping::template contains_key<T*>::value>;

    template <bool InverseTime>
    using time_base =
        std::conditional_t<InverseTime, units::inverse<units::time::second>, units::time::second>;

    template <class Unit, int DerivOrder>
    struct unit_with_deriv {
        using type = units::unit_t<
            tmp::rebind_outer<
                tmp::push_front<typename Unit::unit_type,
                                tmp::repeat<tmp::abs(DerivOrder), time_base<(DerivOrder > 0)>>>,
                tmp::list,
                units::compound_unit>,
            typename Unit::underlying_type>;
    };

    template <class UnitDerivArgPair>
    struct unit_deriv {
        using type = typename unit_with_deriv<typename UnitDerivArgPair::first_type,
                                              UnitDerivArgPair::second_type::value>::type;
    };

  public:
    static_assert((sizeof...(Args) % 2) == 0,
                  "A vector requires an even number of template types.");
    static_assert((sizeof...(Args) > 0), "A vector requires more than zero template types.");
    static_assert(!tmp::rebind_outer<tmp::map<std::is_pointer, keys>, tmp::disjunction>::value,
                  "Vector key types cannot be pointers.");
    static_assert(
        tmp::rebind_outer<tmp::map<units::traits::is_unit_t, values>, tmp::conjunction>::value,
        "Vector value types must be a unit container.");

    using data_type = tmp::rebind_outer<values, std::tuple>;

    static constexpr std::size_t size = sizeof...(Args) / 2;

    template <int N = 1>
    using derivative = tmp::rebind_outer<
        tmp::interleave<
            keys,
            tmp::map<unit_deriv,
                     tmp::zip<values, tmp::repeat<size, std::integral_constant<int, N>>>>>,
        vector>;

    constexpr vector() = default;

    template <class... Utypes,
              class = std::enable_if_t<std::is_constructible<data_type, Utypes...>::value>>
    constexpr vector(Utypes&&... args) : data_{std::forward<Utypes>(args)...}
    {}

    template <class T, class = enable_if_key<T>>
    constexpr decltype(auto) get()
    {
        using Index = typename key_index_mapping::template at_key<T*>;
        return std::get<Index::value>(data_);
    }

    template <class T, class = enable_if_key<T>>
    constexpr decltype(auto) get() const
    {
        using Index = typename key_index_mapping::template at_key<T*>;
        return std::get<Index::value>(data_);
    }

    constexpr auto operator+=(const vector& other) -> vector&
    {
        add_to_impl(other, std::make_index_sequence<size>{});

        return *this;
    }

    template <class Scalar>
    constexpr auto operator*=(Scalar a)
        -> std::enable_if_t<units::traits::is_dimensionless_unit<Scalar>::value, vector&>
    {
        for_each(detail::elementwise_scalar_multiply<Scalar>{a});

        return *this;
    }

    template <class Duration>
    constexpr auto operator*(Duration dt) const
        -> std::enable_if_t<std::is_convertible<Duration, detail::implicit_duration_type>::value,
                            derivative<-1>>
    {
        return multiply_by_time_impl(dt, std::make_index_sequence<size>{});
    }

    template <class Visitor>
    constexpr auto for_each(Visitor v) -> void
    {
        for_each_impl(this, v, std::make_index_sequence<size>{});
    }

    template <class Visitor>
    constexpr auto for_each(Visitor v) const -> void
    {
        for_each_impl(this, v, std::make_index_sequence<size>{});
    }

  private:
    template <class Self, class Visitor, std::size_t... Is>
    static constexpr auto for_each_impl(Self self, Visitor v, std::index_sequence<Is...>) -> void
    {
        const auto unused = {(v(std::get<Is>(self->data_)), 0)...};
        (void)unused;
    }

    template <std::size_t... Is>
    constexpr auto add_to_impl(const vector& other, std::index_sequence<Is...>) -> void
    {
        const auto unused = {(std::get<Is>(data_) += std::get<Is>(other.data_), 0)...};
        (void)unused;
    }

    template <std::size_t... Is>
    constexpr auto multiply_by_time_impl(detail::implicit_duration_type dt,
                                         std::index_sequence<Is...>) const -> derivative<-1>
    {
        auto integral = derivative<-1>{};

        const auto unused = {(std::get<Is>(integral.data_) += std::get<Is>(data_) * dt, 0)...};
        (void)unused;

        return integral;
    }

    data_type data_;

    template <class...>
    friend class vector;
};

template <class Vector>
constexpr auto operator+(const Vector& x, const Vector& y)
    -> std::enable_if_t<tmp::is_specialization_of<Vector, vector>::value, Vector>
{
    auto z = x;
    return z += y;
}

template <class Scalar, class Vector>
constexpr auto operator*(Scalar a, const Vector& x)
    -> std::enable_if_t<tmp::is_specialization_of<Vector, vector>::value &&
                            units::traits::is_dimensionless_unit<Scalar>::value,
                        Vector>
{
    auto y = x;
    return y *= a;
}

template <class Duration, class Vector>
constexpr auto operator*(Duration t, const Vector& x)
    -> std::enable_if_t<tmp::is_specialization_of<Vector, vector>::value &&
                            !units::traits::is_dimensionless_unit<Duration>::value &&
                            std::is_convertible<Duration, detail::implicit_duration_type>::value,
                        typename Vector::template derivative<-1>>
{
    return x * t;
}

template <class Vector>
auto operator<<(std::ostream& os, const Vector& v)
    -> std::enable_if_t<tmp::is_specialization_of<Vector, vector>::value, std::ostream&>
{
    bool first = true;

    os << "{ ";
    v.for_each([&os, &first](const auto& elem) {
        if (first) {
            first = false;
        } else {
            os << ", ";
        }

        os << elem;
    });

    return os << "}";
}

}  // namespace state_space
}  // namespace ode
