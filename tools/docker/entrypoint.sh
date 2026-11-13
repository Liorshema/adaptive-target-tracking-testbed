#!/usr/bin/env bash
set -eo pipefail

if [[ "$(id -u)" == 0 ]]; then
    echo 'The testbed must run as an unprivileged user (default UID/GID 10001).' >&2
    exit 1
fi
if [[ ! -d /data || ! -w /data ]]; then
    echo '/data must be a writable persistent mount for UID/GID 10001.' >&2
    exit 1
fi

# Retain build/dependency evidence alongside data, once per distinct image manifest.
manifest_id=$(sha256sum /opt/testbed-image/* | sha256sum | cut -d ' ' -f 1)
destination="/data/deployments/$manifest_id"
mkdir -p "$destination"
cp -n /opt/testbed-image/* "$destination/"

source /opt/ros/jazzy/setup.bash
source /opt/testbed/install/setup.bash
cd /opt/testbed
exec "$@"
