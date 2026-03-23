load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_test(
    name = "test_library_test",
    srcs = ["test.cc"],
    deps = [
        "//Libraries/TestLibrary/test/mocks:test_library_mocks",
        "@googletest//:gtest_main",
    ],
)
