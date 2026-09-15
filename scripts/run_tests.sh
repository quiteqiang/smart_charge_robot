#!/usr/bin/env bash
# 自动化集成测试：单容器内启动完整系统（带就绪重试，抵御低算力启动竞态），
# 随后运行 pytest 驱动 8 个场景。完整会话日志写入 logs/test_run_<时间戳>.log。
set -uo pipefail
cd "$(dirname "$0")/.."
mkdir -p logs
LOG="logs/test_run_$(date +%Y%m%d_%H%M%S).log"

docker compose run --rm dev bash -lc '
set -eo pipefail   # 不用 -u：ROS setup.bash 含未定义变量
source /opt/ros/jazzy/setup.bash
source install/setup.bash

launch_and_wait() {
  ros2 launch smart_charge_bringup full_demo.launch.py use_rviz:=false &
  BRINGUP_PID=$!
  for i in $(seq 1 75); do
    # 电池节点先上线（2 Hz 常发）
    if [ $i -eq 1 ] || [ $((i % 5)) -eq 0 ]; then
      timeout -k 2 5 ros2 topic echo /battery_state --once >/dev/null 2>&1 && echo "[run_tests] battery up (${i}0s-ish)"
    fi
    # bt_navigator 激活 = 全栈就绪
    if timeout -k 2 5 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -qE "^active \\[3\\]"; then
      echo "[run_tests] bt_navigator active after ~${i}s"
      return 0
    fi
    sleep 2
  done
  return 1
}

OK=0
for attempt in 1 2 3; do
  echo "[run_tests] bringup attempt $attempt"
  if launch_and_wait; then
    OK=1
    break
  fi
  echo "[run_tests] bringup attempt $attempt failed, restarting"
  kill $BRINGUP_PID 2>/dev/null || true
  sleep 5
  pkill -f "full_demo|nav2|amcl|mining_truck|charge_" 2>/dev/null || true
  sleep 5
done

if [ "$OK" != "1" ]; then
  echo "[run_tests] bringup failed after 3 attempts"
  exit 1
fi
sleep 5

python3 -m pytest tests/test_integration.py -v -s --log-cli-level=INFO
kill $BRINGUP_PID 2>/dev/null || true
' > "$LOG" 2>&1
STATUS=$?
echo "full log: $LOG"
tail -40 "$LOG"
exit $STATUS
