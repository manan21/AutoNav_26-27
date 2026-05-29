#!/usr/bin/env bash
# Summarize a bag and flag missing required topics.

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: verify_bag.sh /path/to/bag [--required-topic-file file] [--strict]" >&2
  exit 2
fi

BAG_PATH=$1
shift
REQUIRED_TOPIC_FILE=""
STRICT=0

while [ $# -gt 0 ]; do
  case "$1" in
    --required-topic-file)
      REQUIRED_TOPIC_FILE=${2:?missing --required-topic-file value}
      shift 2
      ;;
    --strict)
      STRICT=1
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

INFO_FILE="$(mktemp)"
trap 'rm -f "$INFO_FILE"' EXIT

echo
echo "ros2 bag info: $BAG_PATH"
if ! ros2 bag info "$BAG_PATH" | tee "$INFO_FILE"; then
  echo "ros2 bag info failed for $BAG_PATH" >&2
  exit 1
fi

if [ -z "$REQUIRED_TOPIC_FILE" ] || [ ! -f "$REQUIRED_TOPIC_FILE" ]; then
  exit 0
fi

missing=0
echo
echo "Required topic check:"
while IFS= read -r topic; do
  topic="${topic%%#*}"
  topic="$(echo "$topic" | xargs)"
  [ -z "$topic" ] && continue
  if grep -F "Topic: $topic " "$INFO_FILE" >/dev/null 2>&1 || grep -F "Topic: $topic |" "$INFO_FILE" >/dev/null 2>&1; then
    echo "  OK      $topic"
  else
    echo "  MISSING $topic"
    missing=$((missing + 1))
  fi
done <"$REQUIRED_TOPIC_FILE"

if [ "$missing" -gt 0 ]; then
  echo
  echo "Missing $missing required topic(s)."
  if [ "$STRICT" -eq 1 ]; then
    exit 1
  fi
fi
