TEST_DEPS = [
    "@com_google_googletest//:gtest_main",
    "//App-StepUp:app_step_up",
    "//testing_interfaces/pico_interface:pico_test",
    "//Interfaces:control_interface",
    "//Libraries/TMC_API:tmc_api",
]
TEST_COPTS = [
    "-I App-StepUp",
]

cc_test(
    name = "hello_test",
    srcs = ["hello_test.cc"],
    deps = TEST_DEPS,
    copts = TEST_COPTS,
)

cc_test(
    name = "test_tmc_control",
    srcs = ["test_tmc_control.cc"],
    deps = TEST_DEPS,
    copts = TEST_COPTS,
)
