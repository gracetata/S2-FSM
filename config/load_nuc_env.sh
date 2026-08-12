#!/usr/bin/env bash
# Source this file from the repository root before building or running.

if [[ -z "${FSM_ROOT:-}" ]]; then
  echo "ERROR: FSM_ROOT is not set. Run: export FSM_ROOT=\"\$(pwd -P)\"" >&2
  return 1
fi

_nuc_env_file="${FSM_ROOT}/config/nuc.env"
if [[ ! -f "${_nuc_env_file}" ]]; then
  cat >&2 <<EOF
ERROR: missing ${_nuc_env_file}
Create and edit this NUC's local configuration before building or launching:
  cp "${FSM_ROOT}/config/nuc.env.example" "${_nuc_env_file}"
EOF
  unset _nuc_env_file
  return 1
fi

_nuc_env_allexport_was_set=0
case "$-" in
  *a*) _nuc_env_allexport_was_set=1 ;;
esac
set -a
# shellcheck source=/dev/null
source "${_nuc_env_file}"
if [[ "${_nuc_env_allexport_was_set}" -eq 0 ]]; then
  set +a
fi
unset _nuc_env_allexport_was_set

_nuc_env_required=(
  LOCOMOTION_RUNTIME_PYTHON
  LOCOMOTION_RUNTIME_HOME
  LOCOMOTION_NETWORK_INTERFACE
  LOCOMOTION_ROBOT_IP
  LOCOMOTION_LOG_ROOT
)
for _nuc_env_name in "${_nuc_env_required[@]}"; do
  if [[ -z "${!_nuc_env_name:-}" ]]; then
    echo "ERROR: ${_nuc_env_name} is not set by ${_nuc_env_file}" >&2
    unset _nuc_env_file _nuc_env_required _nuc_env_name
    return 1
  fi
done
unset _nuc_env_required _nuc_env_name

if [[ ! -x "${LOCOMOTION_RUNTIME_PYTHON}" ]]; then
  echo "ERROR: runtime Python is not executable: ${LOCOMOTION_RUNTIME_PYTHON}" >&2
  unset _nuc_env_file
  return 1
fi
if [[ ! -d "${LOCOMOTION_RUNTIME_HOME}" ]]; then
  echo "ERROR: runtime environment is not a directory: ${LOCOMOTION_RUNTIME_HOME}" >&2
  unset _nuc_env_file
  return 1
fi

unset _nuc_env_file
