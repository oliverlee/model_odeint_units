load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "77dbe5a5262c71a2fe2c033677f50aaa0e31090d",
    remote = "https://github.com/nelhage/rules_boost",
    shallow_since = "1611019749 -0800",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

new_local_repository(
    name = "extern",
    path = "extern",
    build_file = "extern/BUILD",
)
