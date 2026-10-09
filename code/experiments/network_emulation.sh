#!/usr/bin/env bash
set -euo pipefail
action="${1:-show}"; shift || true
interface=""
delay_ms=0
jitter_ms=0
loss_percent=0
rate_mbit=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --interface) interface="$2"; shift 2;;
    --delay-ms) delay_ms="$2"; shift 2;;
    --jitter-ms) jitter_ms="$2"; shift 2;;
    --loss-percent) loss_percent="$2"; shift 2;;
    --rate-mbit) rate_mbit="$2"; shift 2;;
    *) echo "unknown argument: $1" >&2; exit 2;;
  esac
done
[[ -n "$interface" ]] || { echo "--interface is required" >&2; exit 2; }
case "$action" in
  show) tc qdisc show dev "$interface";;
  clear) sudo tc qdisc del dev "$interface" root 2>/dev/null || true;;
  apply)
    args=(netem)
    [[ "$delay_ms" != "0" ]] && args+=(delay "${delay_ms}ms" "${jitter_ms}ms")
    [[ "$loss_percent" != "0" ]] && args+=(loss "${loss_percent}%")
    [[ "$rate_mbit" != "0" ]] && args+=(rate "${rate_mbit}mbit")
    sudo tc qdisc replace dev "$interface" root "${args[@]}"
    tc qdisc show dev "$interface"
    ;;
  *) echo "usage: $0 {show|apply|clear} --interface IFACE [...]" >&2; exit 2;;
esac
