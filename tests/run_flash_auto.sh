#!/bin/sh
# Builds and runs tests/test_flash_auto.c against a copy of pl_flash_7055_auto.c placed next to stub headers (a quote-include
# would otherwise find the real platf.h next to the source).
set -e
here=$(cd "$(dirname "$0")" && pwd)
tmp=$(mktemp -d)
cp "$here/../pl_flash_7055_auto.c" "$here"/stub/*.h "$here/test_flash_auto.c" "$tmp"/
cc -Wall -Wextra -o "$tmp/t" "$tmp/test_flash_auto.c"
"$tmp/t"
rm -rf "$tmp"
