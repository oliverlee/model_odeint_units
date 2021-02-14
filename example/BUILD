COPTS = [
    "-std=c++14",
]

cc_binary(
    name = "odeint_model",
    srcs = [
        "odeint_model.cc",
    ],
    deps = [
        "//:ode_with_boost_odeint",
    ],
    copts = COPTS,
)

cc_binary(
    name = "odeint_state_space",
    srcs = [
        "odeint_state_space.cc",
    ],
    deps = [
        "//:ode_with_boost_odeint",
    ],
    copts = COPTS,
)

cc_binary(
    name = "ode_range",
    srcs = [
        "ode_range.cc",
    ],
    deps = [
        "//:ode",
    ],
    copts = COPTS,
)

cc_binary(
    name = "ode_constexpr",
    srcs = [
        "ode_constexpr.cc",
    ],
    deps = [
        "//:ode_with_gcem",
    ],
    copts = COPTS,
)
