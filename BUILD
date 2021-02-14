package(default_visibility = ["//example:__pkg__"])

cc_library(
    name = "ode",
    hdrs = [
        "include/ode/iterator.h",
        "include/ode/state_space/system.h",
        "include/ode/state_space/vector.h",
        "include/ode/stepper.h",
        "include/ode/tmp/type_mapping.h",
        "include/ode/tmp/type_traits.h",
    ],
    strip_include_prefix = "include",
    defines = [
        "DISABLE_PREDEFINED_UNITS",
        "ENABLE_PREDEFINED_LENGTH_UNITS",
        "ENABLE_PREDEFINED_TIME_UNITS",
        "ENABLE_PREDEFINED_VELOCITY_UNITS",
        "ENABLE_PREDEFINED_ACCELERATION_UNITS",
        "ENABLE_PREDEFINED_ANGLE_UNITS",
        "ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS",
    ],
    deps = [
        "@units",
    ],
)

cc_library(
    name = "ode_with_boost_odeint",
    hdrs = [
        "include/ode/model.h",
        "include/ode/unit_proxy.h",
    ],
    strip_include_prefix = "include",
    deps = [
        "//:ode",
        "@boost//:numeric_odeint",
    ],
)

cc_library(
    name = "ode_with_gcem",
    hdrs = [
        "include/ode/gcem_units.h",
    ],
    strip_include_prefix = "include",
    deps = [
        "//:ode",
        "@gcem",
    ],
)
