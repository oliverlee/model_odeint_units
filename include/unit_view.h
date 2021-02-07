#pragma once

namespace dyn {

template <class Unit>
class unit_view {
  public:
    using unit_type = Unit;
    using underlying_type = typename Unit::underlying_type;

    explicit constexpr unit_view(underlying_type& value) noexcept : value_{value} {}

    constexpr unit_view(unit_view&&) noexcept = default;
    constexpr auto operator=(unit_view&&) noexcept -> unit_view& = default;

    unit_view(const unit_view&) = delete;
    auto operator=(const unit_view&) -> unit_view& = delete;

    constexpr auto operator=(const Unit& other) noexcept { value_ = other.value(); }

    constexpr auto as_unit() const noexcept { return unit_type{value_}; };

  private:
    underlying_type& value_;
};


template <class UnitView>
auto operator<<(std::ostream& os, const UnitView& uv)
    -> std::enable_if_t<stdx::is_specialization_of<UnitView, unit_view>::value, std::ostream&>
{
    return os << uv.as_unit();
}

}  // namespace dyn
