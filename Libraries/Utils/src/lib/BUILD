load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "utils_lib",
    hdrs = glob(["*.h"]),
    copts = ["-I Libraries/Utils/src"],
    visibility = [
        "//Libraries/Utils/src:__pkg__",
        "//Libraries/Utils/test:__pkg__",
    ],
)
