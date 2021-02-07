cc_binary(
    name = "main",
    srcs = [
        "include/model.h",
        "include/type_traits.h",
        "include/unit_view.h",
        "main.cc",
    ],
    includes = ["include"],
    deps = [
        "@boost//:numeric_odeint",
        "@extern//:units",
    ],
    defines = [
        "DISABLE_PREDEFINED_UNITS",
        "ENABLE_PREDEFINED_LENGTH_UNITS",
        "ENABLE_PREDEFINED_TIME_UNITS",
        "ENABLE_PREDEFINED_VELOCITY_UNITS",
        "ENABLE_PREDEFINED_ACCELERATION_UNITS",
        "ENABLE_PREDEFINED_ANGLE_UNITS",
        "ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS",
    ],
)
