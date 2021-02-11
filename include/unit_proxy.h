#pragma once

#include "units.h"

namespace dyn {

template <class Unit>
class unit_proxy {
  public:
    using unit_type = Unit;
    using underlying_type = typename Unit::underlying_type;

    explicit constexpr unit_proxy(underlying_type& value) noexcept : value_{value} {}

    constexpr auto operator=(const Unit& other) noexcept -> void { value_ = other.value(); }

    operator const Unit&() const noexcept { return unit_type{value_}; }

  private:
    underlying_type& value_;
};

template <class UnitProxy>
auto operator<<(std::ostream& os, const UnitProxy& p)
    -> std::enable_if_t<tmp::is_specialization_of<UnitProxy, unit_proxy>::value, std::ostream&>
{
    return os << typename UnitProxy::unit_type(p);
}

}  // namespace dyn
