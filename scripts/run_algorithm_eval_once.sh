#!/usr/bin/env bash
# Analyze one recorded ROS/Gazebo algorithm run and put all derived outputs in results/.
#
# Debug workflow:
#   EVAL_OUTPUT_MODE=debug OVERWRITE=1 ./run_algorithm_eval_once.sh <run_dir>
#
# Paper workflow:
#   ./run_algorithm_eval_once.sh <run_dir> results/algorithm_eval_v1/<versioned_name>

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

RUN_DIR="${1:?usage: ./run_algorithm_eval_once.sh <run_dir> [result_dir]}"
RUN_NAME="$(basename "${RUN_DIR}")"
EVAL_OUTPUT_MODE="${EVAL_OUTPUT_MODE:-paper}"
if [[ $# -ge 2 ]]; then
  RESULT_DIR="${2}"
elif [[ "${EVAL_OUTPUT_MODE}" == "debug" ]]; then
  RESULT_DIR="results/algorithm_eval_tmp/${RUN_NAME}"
else
  RESULT_DIR="results/algorithm_eval_v1/${RUN_NAME}"
fi

OVERWRITE="${OVERWRITE:-0}"
ALLOW_OVERWRITE_VERSIONED="${ALLOW_OVERWRITE_VERSIONED:-0}"

INFLUENCE_RADIUS="${INFLUENCE_RADIUS:-3.5}"
DRONE_RADIUS="${DRONE_RADIUS:-0.35}"
SAFE_MARGIN="${SAFE_MARGIN:-0.6}"
NEAR_MISS_MARGIN="${NEAR_MISS_MARGIN:-0.5}"
TARGET_X="${TARGET_X:-}"
TARGET_Y="${TARGET_Y:-}"
TARGET_Z="${TARGET_Z:-}"
TARGET_RADIUS="${TARGET_RADIUS:-1.5}"

if [[ "${OVERWRITE}" == "1" ]]; then
  case "${RESULT_DIR}" in
    results/algorithm_eval_tmp/*|./results/algorithm_eval_tmp/*)
      echo "[Algorithm Eval] overwrite debug result: ${RESULT_DIR}"
      rm -rf "${RESULT_DIR}"
      ;;
    results/algorithm_eval_v1/*|./results/algorithm_eval_v1/*)
      if [[ "${ALLOW_OVERWRITE_VERSIONED}" == "1" ]]; then
        echo "[Algorithm Eval] overwrite versioned result: ${RESULT_DIR}"
        rm -rf "${RESULT_DIR}"
      else
        echo "[Algorithm Eval] refusing to overwrite versioned result: ${RESULT_DIR}" >&2
        echo "[Algorithm Eval] use results/algorithm_eval_tmp/... for debugging, or set ALLOW_OVERWRITE_VERSIONED=1 explicitly." >&2
        exit 2
      fi
      ;;
    *)
      echo "[Algorithm Eval] refusing to overwrite unexpected result dir: ${RESULT_DIR}" >&2
      echo "[Algorithm Eval] use results/algorithm_eval_tmp/<name> for debug overwrite." >&2
      exit 2
      ;;
  esac
fi

mkdir -p "${RESULT_DIR}/plots"

ANALYZE_CMD=(
  python3 ros_analysis/analyze_ros_run.py
  --run-dir "${RUN_DIR}"
  --output-json "${RESULT_DIR}/analysis_summary.json"
  --output-csv "${RESULT_DIR}/algorithm_metrics.csv"
  --drone-radius "${DRONE_RADIUS}"
  --safe-margin "${SAFE_MARGIN}"
  --near-miss-margin "${NEAR_MISS_MARGIN}"
  --influence-radius "${INFLUENCE_RADIUS}"
)

if [[ -n "${TARGET_X}" && -n "${TARGET_Y}" && -n "${TARGET_Z}" ]]; then
  ANALYZE_CMD+=(
    --target-x "${TARGET_X}"
    --target-y "${TARGET_Y}"
    --target-z "${TARGET_Z}"
    --target-radius "${TARGET_RADIUS}"
  )
fi

"${ANALYZE_CMD[@]}"

python3 ros_analysis/plot_ros_run.py \
  --run-dir "${RUN_DIR}" \
  --output-dir "${RESULT_DIR}/plots" \
  --influence-radius "${INFLUENCE_RADIUS}" \
  --plot-set standard

echo "[Algorithm Eval] result_dir=${RESULT_DIR}"
echo "[Algorithm Eval] metrics=${RESULT_DIR}/algorithm_metrics.csv"
echo "[Algorithm Eval] plots=${RESULT_DIR}/plots"
if [[ "${EVAL_OUTPUT_MODE}" == "debug" || "${RESULT_DIR}" == results/algorithm_eval_tmp/* || "${RESULT_DIR}" == ./results/algorithm_eval_tmp/* ]]; then
  echo "[Algorithm Eval] debug mode: this directory is meant to be overwritten."
else
  echo "[Algorithm Eval] paper mode: keep this directory versioned once the run is valid."
fi
