#!/usr/bin/env bash

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

SP_PROFILE_VALUE="${SUNNYPILOT_HARDWARE_PROFILE:-}"
SP_PROFILE_FILE="${SUNNYPILOT_HARDWARE_PROFILE_FILE:-/data/hardware_profile}"
SP_MODEL_FILE="${SUNNYPILOT_HARDWARE_MODEL_FILE:-/sys/firmware/devicetree/base/model}"
if [ -z "$SP_PROFILE_VALUE" ] && [ -f "$SP_PROFILE_FILE" ]; then
  SP_PROFILE_VALUE="$(tr -d '\000\r\n ' < "$SP_PROFILE_FILE")"
fi
if [ -z "$SP_PROFILE_VALUE" ]; then
  SP_MODEL_VALUE=""
  if [ -f "$SP_MODEL_FILE" ]; then
    SP_MODEL_VALUE="$(tr -d '\000\r\n' < "$SP_MODEL_FILE")"
  fi
  [ "$SP_MODEL_VALUE" = "comma tici" ] && SP_PROFILE_VALUE="c3xl" || SP_PROFILE_VALUE="standard"
fi

if [ "$SP_PROFILE_VALUE" = "c3xl" ]; then
  [ -z "$AGNOS_VERSION" ] && export AGNOS_VERSION="19.7"
  [ -z "$AGNOS_MANIFEST_FILE" ] && export AGNOS_MANIFEST_FILE="openpilot/common/hardware/comma/agnos-c3xl.json"
else
  [ -z "$AGNOS_VERSION" ] && export AGNOS_VERSION="19.7"
  [ -z "$AGNOS_MANIFEST_FILE" ] && export AGNOS_MANIFEST_FILE="openpilot/common/hardware/comma/agnos.json"
fi
unset SP_PROFILE_VALUE SP_PROFILE_FILE SP_MODEL_FILE SP_MODEL_VALUE

export STAGING_ROOT="/data/safe_staging"
