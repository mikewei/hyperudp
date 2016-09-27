package(default_visibility = ["//visibility:public"])

cc_library(
	name = "hyperudp",
	includes = ["include"],
	copts = [
		"-g",
		"-O2",
		"-Wall",
	],
	linkopts = [
		"-lrt",
	],
	nocopts = "-fPIC",
	linkstatic = 1,
	srcs = glob([
		"src/*.cc",
		"src/*.h",
	]),
	deps = [
		"//ccbase",
		"//shm_container",
    "//third_party/gflags-20",
  ],
)

cc_test(
	name = "test",
	copts = [
		"-g",
		"-O2",
		"-Wall",
		"-fno-strict-aliasing",
	],
	nocopts = "-fPIC",
	linkstatic = 1,
	srcs = glob(["test/*_test.cc"]),
	deps = [
		":hyperudp",
		"//gtestx",
	],
	malloc = "//third_party/jemalloc-360"
)

cc_binary(
	name = "echo_server",
	copts = [
		"-g",
		"-O2",
		"-Wall",
	],
	nocopts = "-fPIC",
	linkstatic = 1,
	srcs = ["example/echo_server.cc"],
	deps = [
		":hyperudp",
	],
	malloc = "//third_party/jemalloc-360"
)

cc_binary(
	name = "echo_client",
	copts = [
		"-g",
		"-O2",
		"-Wall",
	],
	nocopts = "-fPIC",
	linkstatic = 1,
	srcs = ["example/echo_client.cc"],
	deps = [
		":hyperudp",
	],
	malloc = "//third_party/jemalloc-360"
)

