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
}  // namespace dyn
