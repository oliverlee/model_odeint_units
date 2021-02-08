#pragma once

#include <type_traits>

namespace dyn {
namespace stdx {

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

}  // namespace stdx

namespace tmp {

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

}  // namespace tmp

}  // namespace dyn
