package(default_visibility = ["//example:__pkg__"])

cc_library(
    name = "ode",
    hdrs = [
        "include/iterator.h",
        "include/state_space/system.h",
        "include/state_space/vector.h",
        "include/stepper.h",
        "include/type_mapping.h",
        "include/type_traits.h",
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
        "include/model.h",
        "include/unit_proxy.h",
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
        "include/gcem_units.h",
    ],
    strip_include_prefix = "include",
    deps = [
        "//:ode",
        "@gcem",
    ],
)
