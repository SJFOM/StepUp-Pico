cc_library(
    name = "app_step_up",
    hdrs = glob(["include/*.h*"]),
    srcs = glob(["src/*.c*"]),
    copts = [
        "-I Libraries/TMC_API",
        # "-I testing-interfaces/pico-interface"
        # "-I Interfaces/ControlInterface",
    ],
    deps = [
        "//Libraries/TMC_API:TMC_API",
        "//testing-interfaces/pico-interface:pico_test"
        # "//Interfaces/ControlInterface:ControlInterface",
    ],
    visibility = [
        "//App-StepUp/tests:__pkg__",
        "//App-StepUp/src:__pkg__",
    ],
)