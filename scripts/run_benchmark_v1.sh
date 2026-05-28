#!/usr/bin/env bash
# Reproduce Benchmark V1 scenario-level Formation Survivability results.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

SUITE_PATH="${1:-scenes/benchmark_v1.yaml}"
OUTPUT_DIR="${2:-results/formation_survivability_v1}"
PLOT_Z="${PLOT_Z:--3.0}"

echo "[Benchmark V1] suite=${SUITE_PATH}"
echo "[Benchmark V1] output=${OUTPUT_DIR}"

python3 difficulty_analysis/formation_survivability_experiment.py \
  --suite "${SUITE_PATH}" \
  --output-dir "${OUTPUT_DIR}"

python3 difficulty_analysis/plot_difficulty_analysis.py \
  --summary-csv "${OUTPUT_DIR}/suite_survivability.csv" \
  --output-dir "${OUTPUT_DIR}/plots" \
  --z "${PLOT_Z}" \
  --plot-scenes-from-summary

echo "[Benchmark V1] done"
echo "[Benchmark V1] summary=${OUTPUT_DIR}/suite_survivability.csv"
echo "[Benchmark V1] plots=${OUTPUT_DIR}/plots"
