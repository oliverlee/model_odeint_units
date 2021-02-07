#pragma once

namespace dyn {

template <class T>
class unit_view {
  public:
    using non_linear_scale_type = typename T::non_linear_scale_type;
    using underlying_type = typename T::underlying_type;
    using value_type = typename T::value_type;
    using unit_type = typename T::unit_type;

    explicit constexpr unit_view(underlying_type& value) noexcept : value_{value} {}

    unit_view(unit_view&&) = delete;
    auto operator=(unit_view&&) -> unit_view& = delete;
    unit_view(const unit_view&) = delete;
    auto operator=(const unit_view&) -> unit_view& = delete;

    constexpr auto operator=(const T& other) noexcept { value_ = other.value(); }

    constexpr operator T() const noexcept { return unit_type{value_}; }

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
