cc_library(
    name = "test_lib",
    hdrs = glob(["*.hpp"]),
    copts = ["-I Libraries/TestLibrary/src"],
    visibility = [
        "//Libraries/TestLibrary/src:__pkg__",
        "//Libraries/TestLibrary/test:__pkg__",
        "//Libraries/TestLibrary/test/mocks:__pkg__",
    ],
)
