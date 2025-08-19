#!/bin/bash
# kill_gazebo.sh - Script to kill all Gazebo related processes

echo "🔍 Checking for Gazebo processes..."

# Kill gzserver, gzclient, ignition-gazebo if running
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 ignition-gazebo 2>/dev/null
pkill -9 gazebo 2>/dev/null

# Verify
if pgrep -a gzserver >/dev/null || pgrep -a gzclient >/dev/null || pgrep -a ignition-gazebo >/dev/null; then
    echo "⚠️ Some Gazebo processes are still running:"
    pgrep -a gz
else
    echo "✅ All Gazebo processes killed successfully."
fi
