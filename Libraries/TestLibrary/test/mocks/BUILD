load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "test_library_mocks",
    srcs = glob(["*.hpp"]),
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//Libraries/TestLibrary/src/lib:test_lib",
        "@googletest//:gtest_main",
    ],
)
