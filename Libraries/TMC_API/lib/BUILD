cc_library(
    name = "tmc_api_lib",
    hdrs = glob(["*.h"]),
    srcs = glob(["*.c"]),
    copts = [
        "-I Libraries/TMC_API/lib",
    ],
    visibility = [
        "//Libraries/TMC_API:__pkg__",
        # "//Libraries/TMC_API/test:__pkg__"
    ],
)
