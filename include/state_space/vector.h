#pragma once

#include "type_mapping.h"
#include "type_traits.h"
#include "units.h"

#include <tuple>

namespace dyn {
namespace state_space {

template <class... Args>
struct vector {
  private:
    using keys = tmp::skip<1, tmp::list<Args...>>;
    using values = tmp::skip<1, tmp::drop<1, tmp::list<Args...>>>;

    using key_index_mapping =
        tmp::rebind_outer<tmp::map<std::add_pointer, keys>, tmp::mapping::index_map>;

    template <class T>
    using enable_if_key = std::enable_if_t<key_index_mapping::template contains_key<T*>::value>;

  public:
    static_assert((sizeof...(Args) % 2) == 0,
                  "A vector requires an even number of template types.");
    static_assert(
        !tmp::rebind_outer<tmp::map<std::is_pointer, keys>, tmp::disjunction>::value,
        "Vector key types cannot be pointers.");
    static_assert(tmp::rebind_outer<tmp::map<units::traits::is_unit_t, values>,
                                    tmp::conjunction>::value,
                  "Vector value types must be a unit container.");

    using data_type = tmp::rebind_outer<values, std::tuple>;

    template <class T, class = enable_if_key<T>>
    decltype(auto) get()
    {
        using Index = typename key_index_mapping::template at_key<T*>;
        return std::get<Index::value>(data);
    }

    template <class T, class = enable_if_key<T>>
    decltype(auto) get() const
    {
        using Index = typename key_index_mapping::template at_key<T*>;
        return std::get<Index::value>(data);
    }

    data_type data;
};

}  // namespace state_space
}  // namespace dyn
