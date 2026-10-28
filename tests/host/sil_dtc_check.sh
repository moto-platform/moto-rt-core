#!/bin/sh
# SIL check for D-080: the host program with N stored synthetic DTCs must complete the
# key-on run (count, list, then 0x04 and 0x06 per DTC) and print every raw answer.
# Usage: sil_dtc_check.sh <vehicle_rtcore_host> [dtcs]
set -e
exe="$1"
n="${2:-2}"
out=$("$exe" --duration-ms 3000 --min-responses 20 --dtcs "$n" --quiet)
want=$((2 + 2 * n))
got=$(printf '%s\n' "$out" | grep -c '^\[DTC\] run 1 key-on 0x19 ')
if [ "$got" -ne "$want" ]; then
    printf '%s\n' "$out"
    echo "expected $want [DTC] answer lines, got $got"
    exit 1
fi
printf '%s\n' "$out" | grep -q 'DTC run 1 (key-on): done, ' || { printf '%s\n' "$out"; echo "run not done"; exit 1; }
printf '%s\n' "$out" | grep -q '^\[DTC\] run 1 key-on 0x19 0x04 DTC ' || { echo "no 0x04 answer"; exit 1; }
printf '%s\n' "$out" | grep -q '^\[DTC\] run 1 key-on 0x19 0x06 DTC ' || { echo "no 0x06 answer"; exit 1; }
