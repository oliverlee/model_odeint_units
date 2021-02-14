#pragma once

#include <cstdint>
#include <type_traits>
#include <utility>

namespace ode {
namespace tmp {

/// @brief Utility metafunction that maps a sequence of any types to the type void
/// @see https://en.cppreference.com/w/cpp/types/void_t
template <class...>
using void_t = void;

/// @brief A helper alias template for the common case where T is bool
/// @see https://en.cppreference.com/w/cpp/types/integral_constant
template <bool B>
using bool_constant = std::integral_constant<bool, B>;

/// @brief Checks if a type is a template specialization
/// @see http://open-std.org/JTC1/SC22/WG21/docs/papers/2020/p2098r1.pdf
template <class T, template <class...> class Primary>
struct is_specialization_of : std::false_type {};

template <template <class...> class Primary, class... Args>
struct is_specialization_of<Primary<Args...>, Primary> : std::true_type {};

/// @brief A type alias for an integral constant where T is `std::size_t`
template <std::size_t I>
using index_constant = std::integral_constant<std::size_t, I>;

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

/// @brief A metatype that inherits from all types in a parameter pack
template <class... Ts>
struct inheritor : Ts... {
    using type = inheritor;
};

/// @brief Variadic logical AND metafunction
/// @see https://en.cppreference.com/w/cpp/experimental/conjunction#Possible_implementation
template <class...>
struct conjunction : std::true_type {};
template <class B1>
struct conjunction<B1> : B1 {};
template <class B1, class... Bn>
struct conjunction<B1, Bn...> : std::conditional_t<bool(B1::value), conjunction<Bn...>, B1> {};

/// @brief Variadic logical OR metafunction
/// @see https://en.cppreference.com/w/cpp/experimental/disjunction#Possible_implementation
template <class...>
struct disjunction : std::false_type {};
template <class B1>
struct disjunction<B1> : B1 {};
template <class B1, class... Bn>
struct disjunction<B1, Bn...> : std::conditional_t<bool(B1::value), B1, disjunction<Bn...>> {};

/// @brief Logical NOT metafunction
/// @see https://en.cppreference.com/w/cpp/types/negation#Possible_implementation
template <class B>
struct negation : std::integral_constant<bool, !bool(B::value)> {};

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
/// @note If the last template type is skipped, the outer `list` is rebound.
namespace detail {

template <class...>
struct rebind_outer_tag;

template <class T, template <class...> class From, template <class...> class To>
struct rebind_outer_impl;

template <class... Ts, template <class...> class From, template <class...> class To>
struct rebind_outer_impl<From<Ts...>, From, To> {
    using type = To<Ts...>;
};

template <class... Ts, template <class...> class To>
struct rebind_outer_impl<list<Ts...>, To, rebind_outer_tag> {
    using type = To<Ts...>;
};

}  // namespace detail

template <class T,
          template <class...>
          class From,
          template <class...> class To = detail::rebind_outer_tag>
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

/// @brief Map a metafunction `Func` to a list of types
namespace detail {

template <template <class> class Func, class R, class T>
struct map_impl;

template <template <class> class Func, class... Rs, class T, class... Ts>
struct map_impl<Func, list<Rs...>, list<T, Ts...>>
    : map_impl<Func, list<Rs..., typename Func<T>::type>, list<Ts...>> {};

template <template <class> class Func, class... Rs>
struct map_impl<Func, list<Rs...>, list<>> : list<Rs...> {};

}  // namespace detail

template <template <class> class Func, class L>
using map = typename detail::map_impl<Func, list<>, L>::type;

/// @brief Check if type K is contained within a list of types
template <class K, class L>
using contains = std::is_base_of<K, rebind_outer<L, list, inheritor>>;

/// @brief Return a list of unique types
namespace detail {

template <class R, class T>
struct make_unique_impl;

template <class... Rs, class T, class... Ts>
struct make_unique_impl<list<Rs...>, list<T, Ts...>>
    : std::conditional_t<contains<T, list<Rs...>>::value,
                         make_unique_impl<list<Rs...>, list<Ts...>>,
                         make_unique_impl<list<Rs..., T>, list<Ts...>>> {};

template <class... Rs>
struct make_unique_impl<list<Rs...>, list<>> : list<Rs...> {};

}  // namespace detail

template <class L>
using make_unique = typename detail::make_unique_impl<list<>, L>::type;

/// @brief Zip together two lists
namespace detail {

template <class R, class L1, class L2>
struct zip_impl;

template <class... Rs, class T1, class... T1s, class T2, class... T2s>
struct zip_impl<list<Rs...>, list<T1, T1s...>, list<T2, T2s...>>
    : zip_impl<list<Rs..., std::pair<T1, T2>>, list<T1s...>, list<T2s...>> {};

template <class... Rs>
struct zip_impl<list<Rs...>, list<>, list<>> : list<Rs...> {};

}  // namespace detail

template <class L1, class L2>
using zip = typename detail::zip_impl<list<>, L1, L2>::type;

/// @brief Interleave together two lists
namespace detail {

template <class R, class L1, class L2>
struct interleave_impl;


template <class... Rs, class T1, class... T1s, class T2, class... T2s>
struct interleave_impl<list<Rs...>, list<T1, T1s...>, list<T2, T2s...>>
    : interleave_impl<list<Rs..., T1, T2>, list<T1s...>, list<T2s...>> {};

template <class... Rs>
struct interleave_impl<list<Rs...>, list<>, list<>> : list<Rs...> {};

}  // namespace detail

template <class L1, class L2>
using interleave = typename detail::interleave_impl<list<>, L1, L2>::type;

/// @brief Obtain the front item in a list
namespace detail {

template <class L>
struct front_impl;

template <class T, class... Ts>
struct front_impl<list<T, Ts...>> {
    using type = T;
};

}  // namespace detail

template <class L>
using front = typename detail::front_impl<L>::type;

}  // namespace tmp
}  // namespace ode
