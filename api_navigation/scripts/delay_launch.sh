#!/bin/bash
# Delay launch helper script
sleep $1
shift
exec "$@"
