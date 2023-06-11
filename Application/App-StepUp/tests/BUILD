TEST_DEPS = [
    "@com_google_googletest//:gtest_main",
    "//Application/App-StepUp:app_step_up",
    "//testing-interfaces/pico-interface:pico_test"
]
# TEST_COPTS = [
#     "-I Application/App-StepUp",
#     "-I Interfaces/ControlInterface",
# ]

cc_test(
    name = "hello_test",
    srcs = ["hello_test.cc"],
    deps = TEST_DEPS,
    # copts = TEST_COPTS,
)

cc_test(
    name = "test_tmc_control",
    srcs = ["test_tmc_control.cc"],
    deps = TEST_DEPS,
    # copts = TEST_COPTS,
)
