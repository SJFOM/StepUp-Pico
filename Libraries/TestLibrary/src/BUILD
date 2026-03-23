load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "TestLibrary",
    hdrs = ["TestLibrary.h"],
    copts = [
        "-I Libraries/TestLibrary/src",
    ],
    visibility = ["//visibility:public"],
    deps = ["//Libraries/TestLibrary/src/lib:test_lib"],
)
