#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/netbridge/setup.bash

exec "$@"
