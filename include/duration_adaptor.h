#pragma once

#include "boost/numeric/odeint.hpp"
#include "type_traits.h"

#include <chrono>

namespace dyn {

template <class Duration, class Implicit>
class duration_adaptor : Duration {
  public:
    static_assert(stdx::is_specialization_of<Duration, std::chrono::duration>::value, "");

    using type = duration_adaptor<Duration, Implicit>;
    using underlying_type = Duration;
    using implicit_type = Implicit;

    using Duration::Duration;

    template <class T, class = std::enable_if_t<!std::is_same<std::decay_t<T>, type>::value>>
    explicit constexpr duration_adaptor(T t) : Duration{t}
    {}

    constexpr duration_adaptor(Implicit i)
        : Duration{typename underlying_type::rep{i * underlying_type::period::den /
                                                 underlying_type::period::num}}
    {}

    constexpr operator Implicit() const
    {
        return std::chrono::duration_cast<std::chrono::duration<Implicit>>(*this).count();
    }

    // constexpr operator double() const {
    //    return std::chrono::duration_cast<std::chrono::duration<double>>(*this).count();
    //}
};

template <class T>
using is_duration_adaptor = stdx::is_specialization_of<T, duration_adaptor>;

template <class D1,
          class D2,
          class Implicit = std::enable_if_t<is_duration_adaptor<std::decay_t<D1>>::value &&
                                                is_duration_adaptor<std::decay_t<D2>>::value,
                                            typename D1::implicit_type>>
auto operator+(const D1& d1, const D2& d2)
{
    using U1 = typename D1::underlying_type;
    using U2 = typename D2::underlying_type;

    return duration_adaptor<std::common_type_t<U1, U2>, Implicit>{static_cast<U1>(d1) +
                                                                  static_cast<U2>(d2)};
}

template <class D1,
          class D2,
          class Implicit = std::enable_if_t<is_duration_adaptor<std::decay_t<D1>>::value &&
                                                is_duration_adaptor<std::decay_t<D2>>::value,
                                            typename D1::implicit_type>>
auto operator-(const D1& d1, const D2& d2)
{
    using U1 = typename D1::underlying_type;
    using U2 = typename D2::underlying_type;

    return duration_adaptor<std::common_type_t<U1, U2>, Implicit>{static_cast<U1>(d1) -
                                                                  static_cast<U2>(d2)};
}

template <class D1,
          class D2,
          class = std::enable_if_t<is_duration_adaptor<std::decay_t<D1>>::value &&
                                   is_duration_adaptor<std::decay_t<D2>>::value>>
auto operator<(const D1& d1, const D2& d2)
{
    using U1 = typename D1::underlying_type;
    using U2 = typename D2::underlying_type;

    return static_cast<U1>(d1) < static_cast<U2>(d2);
}

template <class D1,
          class D2,
          class = std::enable_if_t<is_duration_adaptor<std::decay_t<D1>>::value &&
                                   is_duration_adaptor<std::decay_t<D2>>::value>>
auto operator<=(const D1& d1, const D2& d2)
{
    using U1 = typename D1::underlying_type;
    using U2 = typename D2::underlying_type;

    return static_cast<U1>(d1) <= static_cast<U2>(d2);
}

template <class Implicit, class D>
auto operator+(const Implicit& i, const D& d) -> std::enable_if_t<
    std::is_same<std::decay_t<Implicit>, typename std::decay_t<D>::implicit_type>::value,
    Implicit>
{
    return i + Implicit(d);
}

template <class Implicit, class D>
auto operator*(const Implicit& i, const D& d) -> std::enable_if_t<
    std::is_same<std::decay_t<Implicit>, typename std::decay_t<D>::implicit_type>::value,
    Implicit>
{
    return i * Implicit(d);
}

}  // namespace dyn

namespace boost {
namespace numeric {
namespace odeint {
namespace detail {

template <class T, class = std::enable_if_t<dyn::is_duration_adaptor<std::decay_t<T>>::value>>
bool less_with_sign(T t1, T t2, T)
{
    return t1 < t2;
}

template <class T, class = std::enable_if_t<dyn::is_duration_adaptor<std::decay_t<T>>::value>>
bool less_eq_with_sign(T t1, T t2, T)
{
    return t1 <= t2;
}

}  // namespace detail
}  // namespace odeint
}  // namespace numeric
}  // namespace boost
