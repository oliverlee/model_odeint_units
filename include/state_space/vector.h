#pragma once

#include "type_traits.h"

#include <tuple>

namespace dyn {
namespace state_space {

template <class... Args>
struct vector {
    static_assert((sizeof...(Args) % 2) == 0, "");

    using data_type =
        tmp::rebind_outer<tmp::skip<1, tmp::drop<1, tmp::list<Args...>>>, tmp::list, std::tuple>;
};

}  // namespace state_space
}  // namespace dyn
