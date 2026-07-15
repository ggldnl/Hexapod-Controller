#!/usr/bin/env bash
# Run the Python controller tests. Pure Python, no hardware, no pyserial needed.
#   ./tests/run.sh
cd "$(dirname "$0")/.." || exit 2
status=0
for t in tests/test_*.py; do
    echo "=== $t ==="
    python3 "$t" || status=1
    echo
done
[ $status -eq 0 ] && echo "ALL PYTHON TESTS PASSED" || echo "PYTHON TESTS FAILED"
exit $status
