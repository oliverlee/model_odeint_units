load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "77dbe5a5262c71a2fe2c033677f50aaa0e31090d",
    remote = "https://github.com/nelhage/rules_boost",
    shallow_since = "1611019749 -0800",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

http_archive(
    name = "gcem",
    url = "https://github.com/kthohr/gcem/archive/v1.13.1.tar.gz",
    sha256 = "69a1973f146a4a5e584193af062359f50bd5b948c4175d58ea2622e1c066b99b",
    strip_prefix = "gcem-1.13.1",
    build_file = "@//:gcem.BUILD"
)

http_archive(
    name = "units",
    url = "https://github.com/nholthaus/units/archive/ea6d126942cb3225a341568ab57ec52513977875.tar.gz",
    sha256 = "38efce0d968438c3602524d9715f25ffded91256c85527492436b0eb4fa80fee",
    strip_prefix = "units-ea6d126942cb3225a341568ab57ec52513977875",
    build_file = "@//:units.BUILD"
)
