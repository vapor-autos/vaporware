#!/usr/bin/env bash

LAUNCH_ENV_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -f "$LAUNCH_ENV_DIR/.env" ]; then
  set -a
  source "$LAUNCH_ENV_DIR/.env"
  set +a
fi

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="18.4"
fi

export STAGING_ROOT="/data/safe_staging"
