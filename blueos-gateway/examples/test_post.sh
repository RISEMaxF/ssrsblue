#!/bin/bash
# Test MAVLink2REST POST endpoint
HOST="${1:-192.168.1.248}"

echo "=== Testing POST paths ==="

for path in \
  "/mavlink2rest/v1/mavlink" \
  "/mavlink2rest/mavlink" \
  "/mavlink2rest/v1/mavlink/" \
  "/mavlink2rest/mavlink/" \
; do
  CODE=$(curl -s -o /dev/null -w "%{http_code}" \
    -X POST "http://$HOST$path" \
    -H "Content-Type: application/json" \
    -d '{"header":{"system_id":255,"component_id":0,"sequence":0},"message":{"type":"HEARTBEAT","custom_mode":0,"mavtype":{"type":"MAV_TYPE_GCS"},"autopilot":{"type":"MAV_AUTOPILOT_INVALID"},"base_mode":{"bits":0},"system_status":{"type":"MAV_STATE_ACTIVE"},"mavlink_version":3}}')
  echo "POST $path -> $CODE"
done

echo ""
echo "=== Testing POST to internal ports ==="
for port in 6040 8081 9000 9090; do
  CODE=$(curl -s -o /dev/null -w "%{http_code}" --connect-timeout 1 \
    -X POST "http://$HOST:$port/v1/mavlink" \
    -H "Content-Type: application/json" \
    -d '{"header":{"system_id":255,"component_id":0,"sequence":0},"message":{"type":"HEARTBEAT","custom_mode":0,"mavtype":{"type":"MAV_TYPE_GCS"},"autopilot":{"type":"MAV_AUTOPILOT_INVALID"},"base_mode":{"bits":0},"system_status":{"type":"MAV_STATE_ACTIVE"},"mavlink_version":3}}')
  echo "POST :$port/v1/mavlink -> $CODE"
done
