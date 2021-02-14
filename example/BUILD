COPTS = [
    "-std=c++14",
]

cc_binary(
    name = "main",
    srcs = [
        "main.cc",
    ],
    deps = [
        "//:ode_with_boost_odeint",
    ],
    copts = COPTS,
)

cc_binary(
    name = "main2",
    srcs = [
        "main2.cc",
    ],
    deps = [
        "//:ode_with_boost_odeint",
        "//:ode_with_gcem",
    ],
    copts = COPTS,
)

cc_binary(
    name = "main3",
    srcs = [
        "main3.cc",
    ],
    deps = [
        "//:ode_with_gcem",
    ],
    copts = COPTS,
)

cc_binary(
    name = "main4",
    srcs = [
        "main4.cc",
    ],
    deps = [
        "//:ode_with_gcem",
    ],
    copts = COPTS,
)
