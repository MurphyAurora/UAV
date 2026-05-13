#!/bin/bash

# 稳定回退入口：直接复用 start1.sh
# 如需混合功能（状态交换/避障/主控）后续再单独开启

NUM_DRONES=${1:-3}
echo "[INFO] rollback mode: run start1 only, num_drones=${NUM_DRONES}"
exec bash ~/ws_xtd2/scripts/start1.sh "${NUM_DRONES}"
