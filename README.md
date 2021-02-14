# model_odeint_units
Compile-time ODE integration with unit-safe operations

This project uses [Bazel](https://bazel.build) for building and requires a
compiler supporting C++14. The following libraries are pulled in as
dependencies:

* [odeint](https://github.com/headmyshoulder/odeint-v2) Provides explicit
  steppers for solving ODEs.
* [units](https://github.com/nholthaus/units) A compile-time physical units
  library.
* [gcem](https://github.com/kthohr/gcem) A compile-time math library using
  generalized constant expressions.

## building

    $ bazel build //...

## running

    $ bazel run //example:odeint_model
