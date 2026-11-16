#!/usr/bin/env bash
# Builds the firmware, flashes it through the debug probe, runs it, and prints its report lines.
# The firmware runs until the timeout ends it: exit status 124 is the normal end; the verdict is in
# the [PASS]/[FAIL] lines. A crash gives another status. The full log is kept under logs/.
#
# Usage: [FEATURES=<cargo features>] tools/run.sh [timeout_seconds, default 120]
#   FEATURES=no-fbeat tools/run.sh 30    # the negative control (see README)
# Before flashing, tools/check-ram-code.py (python3, arm-none-eabi-binutils) checks that nothing the
# flight core runs lies in flash (decision-ram-code-20261115, R1): it must pass, or with the feature
# ram-negative (its negative control) it must fail; otherwise nothing is flashed (exit 3).
set -u
tmo="${1:-120}"
bin="marv-v2-firmware"
cd "$(dirname "$0")/.." || exit 1

cargo build --release --bin "$bin" ${FEATURES:+--features "$FEATURES"} || exit $?

mkdir -p logs
log="logs/${bin}-$(date +%Y%m%d-%H%M%S).log"
# The build folder is wherever cargo puts it: CARGO_TARGET_DIR or a cargo config can move it.
target_dir="$(cargo metadata --format-version 1 --no-deps | sed -n 's/.*"target_directory":"\([^"]*\)".*/\1/p')"
elf="${target_dir:?cargo metadata gave no target directory}/thumbv8m.main-none-eabihf/release/${bin}"

python3 tools/check-ram-code.py "$elf" | tee "$log"
check=${PIPESTATUS[0]}
want=0
[[ " ${FEATURES//,/ } " == *" ram-negative "* ]] && want=1
if [ "$check" -ne "$want" ]; then
    echo "check-ram-code: exit $check, $want required: not flashed" | tee -a "$log"
    exit 3
fi

# One probe, one user at a time, across every checkout on this machine.
lock="${XDG_RUNTIME_DIR:-/tmp}/marv-v2-probe.lock"
flock "$lock" timeout "$tmo" probe-rs run --chip RP235x "$elf" >>"$log" 2>&1
status=$?

grep -E '\[(PASS|FAIL|SKIP|INFO|STAT)\]|RESULT|panicked|ERROR|Error' "$log" | tail -n 80
echo "log: $log"
exit "$status"
