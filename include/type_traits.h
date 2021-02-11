#pragma once

#include <type_traits>

namespace dyn {
namespace tmp {

/// @brief Checks if a type is a template specialization
/// @see http://open-std.org/JTC1/SC22/WG21/docs/papers/2020/p2098r1.pdf
template <class T, template <class...> class Primary>
struct is_specialization_of : std::false_type {};

template <template <class...> class Primary, class... Args>
struct is_specialization_of<Primary<Args...>, Primary> : std::true_type {};

/// @brief Provides the member typedef type that names T (i.e., the identity transformation).
/// @see https://en.cppreference.com/w/cpp/types/type_identity
template <class T>
struct type_identity {
    using type = T;
};

template <class T>
using type_identity_t = typename type_identity<T>::type;


/// @brief A metatype that contains a parameter pack
template <class...>
struct list {
    using type = list;
};

/// @brief Stores `Count` number of repetitions of type `T` in a `list`
namespace detail {

template <int, class...>
struct repeat_impl;

template <int N, class T, class... Ts>
struct repeat_impl<N, T, list<Ts...>> : std::conditional_t<(N <= 0),
                                                           repeat_impl<N, list<Ts...>>,
                                                           repeat_impl<N - 1, T, list<T, Ts...>>> {
};

template <int N, class... Ts>
struct repeat_impl<N, list<Ts...>> : list<Ts...> {};

}  // namespace detail

template <int Count, class T>
using repeat = typename detail::repeat_impl<Count, T, list<>>::type;

/// @brief Pushes `T` to the front of list `L`
namespace detail {

template <class T, class L>
struct push_front_impl;

template <class T, class... Ts>
struct push_front_impl<T, list<Ts...>> : list<T, Ts...> {};

}  // namespace detail

// Given L<T, Ts...> return L<Ts...>.
template <class T, class L>
using push_front = typename detail::push_front_impl<T, L>::type;

/// @brief Replace the outer container type
namespace detail {

template <class T, template <class...> class From, template <class...> class To>
struct rebind_outer_impl;

template <class... Ts, template <class...> class From, template <class...> class To>
struct rebind_outer_impl<From<Ts...>, From, To> {
    using type = To<Ts...>;
};

}  // namespace detail

template <class T, template <class...> class From, template <class...> class To>
using rebind_outer = typename detail::rebind_outer_impl<T, From, To>::type;

/// @brief Calculate the absolute value of an integer type
template <class Int>
constexpr auto abs(Int n) noexcept -> std::enable_if_t<std::is_signed<Int>::value, Int>
{
    return (n < 0) ? -n : n;
}

/// @brief Drop the first n elements in a list
namespace detail {

template <int N, class L>
struct drop_impl;

template <int N, class T, class... Ts>
struct drop_impl<N, list<T, Ts...>>
    : std::conditional_t<(N <= 0), list<T, Ts...>, drop_impl<N - 1, list<Ts...>>> {};

}  // namespace detail

template <int N, class L>
using drop = typename detail::drop_impl<N, L>::type;

/// @brief Skip every n-th element in a list
namespace detail {

template <int N, int I, class R, class T>
struct skip_impl;

template <int N, int I, class... Rs, class T, class... Ts>
struct skip_impl<N, I, list<Rs...>, list<T, Ts...>>
    : std::conditional_t<(I == 0),
                         skip_impl<N, N, list<Rs..., T>, list<Ts...>>,
                         skip_impl<N, I - 1, list<Rs...>, list<Ts...>>> {
    static_assert(N > 0, "");
};

template <int N, int I, class... Rs>
struct skip_impl<N, I, list<Rs...>, list<>> : list<Rs...> {};

template <class... Ts>
struct skip_impl<0, 0, list<>, list<Ts...>> : list<Ts...> {};

}  // namespace detail

template <int N, class L>
using skip = typename detail::skip_impl<N, 0, list<>, L>::type;

}  // namespace tmp

}  // namespace dyn
