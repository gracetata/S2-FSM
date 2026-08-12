#!/usr/bin/env bash
# Source this file before ROS setup and launch. It performs no robot I/O.

_locomotion_config_dir="$(
  cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P
)" || {
  echo "[S2-FSM] cannot resolve the config directory" >&2
  return 1
}
_locomotion_env_file="${_locomotion_config_dir}/nuc.env"

if [[ ! -f "${_locomotion_env_file}" ]]; then
  echo "[S2-FSM] missing ${_locomotion_env_file}" >&2
  echo "[S2-FSM] copy nuc.env.example to nuc.env and edit this NUC's values" >&2
  return 1
fi

_locomotion_allexport_was_set=0
case "$-" in
  *a*) _locomotion_allexport_was_set=1 ;;
esac
set -a
# shellcheck disable=SC1090
source "${_locomotion_env_file}" || {
  if [[ "${_locomotion_allexport_was_set}" -eq 0 ]]; then
    set +a
  fi
  echo "[S2-FSM] failed to load ${_locomotion_env_file}" >&2
  return 1
}
if [[ "${_locomotion_allexport_was_set}" -eq 0 ]]; then
  set +a
fi

for _locomotion_variable in \
  LOCOMOTION_RUNTIME_PYTHON \
  LOCOMOTION_RUNTIME_HOME \
  LOCOMOTION_NETWORK_INTERFACE \
  LOCOMOTION_ROBOT_IP \
  LOCOMOTION_LOG_ROOT \
  LOCOMOTION_SOCKET_PATH; do
  if [[ -z "${!_locomotion_variable:-}" ]]; then
    echo "[S2-FSM] ${_locomotion_variable} is missing or empty" >&2
    return 1
  fi
done

if [[ ! -x "${LOCOMOTION_RUNTIME_PYTHON}" ]]; then
  echo "[S2-FSM] runtime Python is not executable: ${LOCOMOTION_RUNTIME_PYTHON}" >&2
  return 1
fi
if [[ ! -d "${LOCOMOTION_RUNTIME_HOME}" ]]; then
  echo "[S2-FSM] runtime environment is not a directory: ${LOCOMOTION_RUNTIME_HOME}" >&2
  return 1
fi
if [[ "${LOCOMOTION_SOCKET_PATH}" != /* ]]; then
  echo "[S2-FSM] LOCOMOTION_SOCKET_PATH must be absolute" >&2
  return 1
fi

mkdir -p "${LOCOMOTION_LOG_ROOT}" || {
  echo "[S2-FSM] cannot create log directory: ${LOCOMOTION_LOG_ROOT}" >&2
  return 1
}

PYTHONNOUSERSITE=1 "${LOCOMOTION_RUNTIME_PYTHON}" -c \
  "import numpy, onnxruntime, cyclonedds, unitree_sdk2py" || {
  echo "[S2-FSM] runtime Python is missing a required control dependency" >&2
  return 1
}

echo "[S2-FSM] NUC environment ready"
echo "[S2-FSM] runtime Python: ${LOCOMOTION_RUNTIME_PYTHON}"
echo "[S2-FSM] robot network: ${LOCOMOTION_NETWORK_INTERFACE} -> ${LOCOMOTION_ROBOT_IP}"

unset _locomotion_allexport_was_set _locomotion_config_dir \
  _locomotion_env_file _locomotion_variable
