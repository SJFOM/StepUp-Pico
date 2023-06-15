cc_library(
    name = "app_step_up",
    hdrs = glob(["include/*.h*"]),
    srcs = glob(["src/*.c*"]),
    copts = [
        "-I Libraries/TMC_API",
        # "-I testing-interfaces/pico-interface"
        # "-I Interfaces",
    ],
    deps = [
        "//Libraries/TMC_API:tmc_api",
        "//testing_interfaces/pico_interface:pico_test",
        "//Interfaces:control_interface",
    ],
    visibility = [
        "//App-StepUp/tests:__pkg__",
        "//App-StepUp/src:__pkg__",
    ],
)