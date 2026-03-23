load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "Utils",
    hdrs = ["Utils.h"],
    copts = [
        "-I Libraries/Utils/src",
    ],
    visibility = ["//visibility:public"],
    deps = ["//Libraries/Utils/src/lib:utils_lib"],
)
