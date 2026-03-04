#!/bin/bash
HOST="${1:-192.168.1.248}"
for msg in MANUAL_CONTROL SET_MODE COMMAND_LONG SET_POSITION_TARGET_GLOBAL_INT SET_POSITION_TARGET_LOCAL_NED SET_ATTITUDE_TARGET; do
  echo "=== $msg ==="
  curl -s "http://$HOST/mavlink2rest/v1/helper/mavlink?name=$msg"
  echo ""
done
