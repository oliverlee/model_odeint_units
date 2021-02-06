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

}  // namespace stdx
}  // namespace dyn
