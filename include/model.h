#pragma once

#include <iostream>

namespace dyn {

struct Model {};

auto operator<<(std::ostream& os, const Model& m) -> std::ostream&
{
    return os << "model m";
}

}  // namespace dyn
