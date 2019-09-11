#!/bin/sh

set -x

"$@" &
pid=$!
sleep 10
kill -9 $pid || true

exit 0
