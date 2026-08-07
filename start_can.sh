#!/usr/bin/env bash

# 车/Bunker：can0，500 kbit/s
# 臂/Piper ：can1，1 Mbit/s
# 通过 USB-CAN 序列号识别设备，所以更换电脑上的 USB 插口不影响映射。

set -Eeuo pipefail

readonly VEHICLE_SERIAL="${VEHICLE_CAN_SERIAL:-003D003A5746570F20383839}"
readonly ARM_SERIAL="${ARM_CAN_SERIAL:-0036003A5246571120393733}"
readonly CHECK_SECONDS="${CAN_CHECK_SECONDS:-5}"

die() {
  printf '错误：%s\n' "$*" >&2
  exit 1
}

need_command() {
  command -v "$1" >/dev/null 2>&1 || die "缺少命令：$1"
}

[[ "$VEHICLE_SERIAL" != "$ARM_SERIAL" ]] || die "车和臂的 USB-CAN 序列号不能相同"
[[ "$CHECK_SECONDS" =~ ^[1-9][0-9]*$ ]] || die "CAN_CHECK_SECONDS 必须是正整数"

# 直接运行脚本即可；需要时会自动请求 sudo。
if (( EUID != 0 )); then
  need_command sudo
  need_command readlink
  script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
  exec sudo -- /usr/bin/env \
    VEHICLE_CAN_SERIAL="$VEHICLE_SERIAL" \
    ARM_CAN_SERIAL="$ARM_SERIAL" \
    CAN_CHECK_SECONDS="$CHECK_SECONDS" \
    bash "$script_path"
fi

for command_name in ip udevadm candump flock mktemp sed head rm; do
  need_command "$command_name"
done

# 防止两个终端同时执行脚本、同时修改接口名。
exec 9>/run/lock/fastlio-start-can.lock
flock -n 9 || die "另一个 start_can.sh 正在运行"

get_serial() {
  local iface="$1"
  udevadm info --query=property --path="/sys/class/net/$iface" 2>/dev/null |
    sed -n 's/^ID_SERIAL_SHORT=//p' |
    head -n 1
}

find_iface_by_serial() {
  local wanted_serial="$1"
  local path iface serial
  local -a matches=()

  for path in /sys/class/net/*; do
    [[ -e "$path" && -r "$path/type" ]] || continue
    [[ "$(<"$path/type")" == "280" ]] || continue
    iface="${path##*/}"
    serial="$(get_serial "$iface" || true)"
    [[ "$serial" == "$wanted_serial" ]] && matches+=("$iface")
  done

  (( ${#matches[@]} == 1 )) || return 1
  printf '%s\n' "${matches[0]}"
}

link_exists() {
  ip link show dev "$1" >/dev/null 2>&1
}

choose_temp_name() {
  local prefix="$1"
  local i name
  for (( i=0; i<100; i++ )); do
    name="${prefix}${i}"
    if ! link_exists "$name"; then
      printf '%s\n' "$name"
      return 0
    fi
  done
  return 1
}

safe_down_by_serial() {
  local path iface serial
  set +e
  for path in /sys/class/net/*; do
    [[ -e "$path" && -r "$path/type" ]] || continue
    [[ "$(<"$path/type")" == "280" ]] || continue
    iface="${path##*/}"
    serial="$(get_serial "$iface" || true)"
    if [[ "$serial" == "$VEHICLE_SERIAL" || "$serial" == "$ARM_SERIAL" ]]; then
      ip link set dev "$iface" down >/dev/null 2>&1
    fi
  done
}

start_failed() {
  local status="$1"
  local line="$2"
  trap - ERR
  safe_down_by_serial
  printf '启动失败（第 %s 行），两路 CAN 已尽力置为 DOWN。\n' "$line" >&2
  exit "$status"
}

udevadm settle --timeout=5 || die "等待 USB-CAN 枚举完成时超时"

vehicle_iface="$(find_iface_by_serial "$VEHICLE_SERIAL" || true)"
arm_iface="$(find_iface_by_serial "$ARM_SERIAL" || true)"

[[ -n "$vehicle_iface" ]] ||
  die "没有唯一找到车的 USB-CAN（serial=$VEHICLE_SERIAL）"
[[ -n "$arm_iface" ]] ||
  die "没有唯一找到臂的 USB-CAN（serial=$ARM_SERIAL）"
[[ "$vehicle_iface" != "$arm_iface" ]] || die "车和臂映射到了同一个接口"

printf '识别结果：车=%s（%s），臂=%s（%s）\n' \
  "$vehicle_iface" "$VEHICLE_SERIAL" "$arm_iface" "$ARM_SERIAL"

# can0/can1 如果被第三个无关接口占用，则停止，避免误改其他设备。
for target in can0 can1; do
  if link_exists "$target" \
    && [[ "$target" != "$vehicle_iface" ]] \
    && [[ "$target" != "$arm_iface" ]]; then
    die "$target 被无关接口占用"
  fi
done

vehicle_tmp=""
arm_tmp=""
if [[ "$vehicle_iface" != "can0" || "$arm_iface" != "can1" ]]; then
  vehicle_tmp="$(choose_temp_name cvtmp)" || die "无法生成车的临时接口名"
  arm_tmp="$(choose_temp_name catmp)" || die "无法生成臂的临时接口名"
fi

trap 'start_failed "$?" "$LINENO"' ERR

ip link set dev "$vehicle_iface" down
ip link set dev "$arm_iface" down

# 用两个临时名处理 can0/can1 刚好互换的情况。
if [[ -n "$vehicle_tmp" ]]; then
  ip link set dev "$vehicle_iface" name "$vehicle_tmp"
  ip link set dev "$arm_iface" name "$arm_tmp"
  ip link set dev "$vehicle_tmp" name can0
  ip link set dev "$arm_tmp" name can1
fi

ip link set dev can0 type can bitrate 500000 restart-ms 100
ip link set dev can1 type can bitrate 1000000 restart-ms 100
ip link set dev can0 up
ip link set dev can1 up

trap - ERR
printf 'CAN 已启动：can0=车@500k，can1=臂@1M。\n'
printf '同时监听两路 %s 秒，检查是否收到数据...\n' "$CHECK_SECONDS"

# 两路同时监听；每路收到第一帧就结束，超时则输出文件为空。
vehicle_data=""
arm_data=""
vehicle_error=""
arm_error=""

cleanup() {
  local temp_file
  set +e
  for temp_file in "$vehicle_data" "$arm_data" "$vehicle_error" "$arm_error"; do
    [[ -n "$temp_file" ]] && rm -f -- "$temp_file"
  done
}
trap cleanup EXIT

vehicle_data="$(mktemp)"
arm_data="$(mktemp)"
vehicle_error="$(mktemp)"
arm_error="$(mktemp)"

check_milliseconds=$(( 10#$CHECK_SECONDS * 1000 ))
candump -L -n 1 -T "$check_milliseconds" can0 >"$vehicle_data" 2>"$vehicle_error" &
vehicle_pid=$!
candump -L -n 1 -T "$check_milliseconds" can1 >"$arm_data" 2>"$arm_error" &
arm_pid=$!

set +e
wait "$vehicle_pid"
vehicle_status=$?
wait "$arm_pid"
arm_status=$?
set -e

validation_failed=0

if [[ -s "$vehicle_data" ]]; then
  printf '[有数据] 车 can0：%s\n' "$(head -n 1 "$vehicle_data")"
else
  printf '[无数据] 车 can0：%s 秒内未收到 CAN 帧' "$CHECK_SECONDS"
  [[ -s "$vehicle_error" ]] && printf '；%s' "$(head -n 1 "$vehicle_error")"
  printf '\n'
  validation_failed=1
fi

if [[ -s "$arm_data" ]]; then
  printf '[有数据] 臂 can1：%s\n' "$(head -n 1 "$arm_data")"
else
  printf '[无数据] 臂 can1：%s 秒内未收到 CAN 帧' "$CHECK_SECONDS"
  [[ -s "$arm_error" ]] && printf '；%s' "$(head -n 1 "$arm_error")"
  printf '\n'
  validation_failed=1
fi

if (( vehicle_status != 0 || arm_status != 0 )); then
  validation_failed=1
fi

if (( validation_failed != 0 )); then
  printf '验证未通过：请检查对应设备供电、CAN_H/CAN_L、波特率和终端电阻。\n' >&2
  exit 1
fi

printf '验证通过：车和臂两路 CAN 都收到了数据。\n'
