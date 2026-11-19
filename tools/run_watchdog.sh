#!/usr/bin/env bash
# Run a long CAD/mesh job with a hard time limit and stall detection.
# usage: run_watchdog.sh <max_s> <stall_s> <logfile> -- <command...>
#   max_s   : kill after this many seconds in total
#   stall_s : kill if <logfile> got no new output for this many seconds
# The job runs in its own process group (survives the calling shell); all of it is killed on limit.
# Status goes to stdout and <logfile>.status; exit code: job's code, 124 = time limit, 125 = stall.
set -u
max_s=$1 stall_s=$2 log=$3
shift 3
[ "${1:-}" = "--" ] && shift
status="$log.status"
: > "$log"
start=$(date +%s)
setsid "$@" >> "$log" 2>&1 < /dev/null &
pid=$!
echo "started pid $pid, limit ${max_s}s, stall ${stall_s}s" | tee "$status"

stop() {
    echo "$1 -> killing process group $pid" | tee -a "$status"
    kill -TERM -- -"$pid" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
    kill -KILL -- -"$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
    exit "$2"
}

while kill -0 "$pid" 2>/dev/null; do
    sleep 5
    now=$(date +%s)
    idle=$(( now - $(stat -c %Y "$log") ))
    (( now - start > max_s )) && stop "time limit ${max_s}s reached" 124
    (( idle > stall_s )) && stop "no output for ${idle}s (last: $(tail -1 "$log"))" 125
done
wait "$pid"
rc=$?
echo "finished rc=$rc after $(( $(date +%s) - start ))s" | tee -a "$status"
exit $rc
