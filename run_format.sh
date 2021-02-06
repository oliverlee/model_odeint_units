#!/bin/bash

format_bin=${1:-/usr/local/opt/llvm/bin/clang-format}
find . -path ./extern -prune -o \(  -name "*.cc" -o -name "*.h" \) -exec ${format_bin} -i {} \;
