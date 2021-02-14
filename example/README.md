# examples

All examples depend on `units` for unit-safe operations.

* `odeint_model`
Uses a model class with `boost::numeric::odeint::{runge_kutta4,array_algebra}`.

* `odeint_state_space`
Uses `ode::state_space` types with
`boost::numeric::odeint::{runge_kutta4,vector_space_algebra}`.

* `ode_range`
Uses `ode::state_space` types with `ode::stepper`.

* `ode_constexpr`
Uses `ode::state_space` types with `ode::stepper` and `gcem` allowing
integration at compile-time. Builds may fail with Clang as it does not memoize
constexpr operations <span id="a0">[[0]](#0)</span>.

<span id="0">[0]: </span>https://stackoverflow.com/questions/24591466/constexpr-depth-limit-with-clang-fconstexpr-depth-doesnt-seem-to-work<br>


