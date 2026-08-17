#!/usr/bin/env bash
# 一键启动 Bunker Mini + Livox MID360 + S-FAST_LIO 建图。
# Ctrl+C 时先停止建图并等待 PCD 落盘，再停止雷达和底盘，最后恢复
# mid360.yaml 中 pcd_save_en=false，避免影响后续重定位。

set -Eeuo pipefail

readonly ROS_SETUP="/opt/ros/noetic/setup.bash"
readonly BUNKER_SETUP="/home/user/bunker_ws/devel/setup.bash"
readonly LIVOX_SETUP="/home/user/livox_ws/devel/setup.bash"
readonly FASTLIO_SETUP="/home/user/fastlio_ws/devel/setup.bash"
readonly MID360_CONFIG="/home/user/fastlio_ws/src/S-FAST_LIO/config/mid360.yaml"
readonly PCD_DIR="/home/user/fastlio_ws/src/S-FAST_LIO/PCD"
readonly RVIZ_ENABLE="${RVIZ_ENABLE:-true}"

BUNKER_PID=""
LIVOX_PID=""
MAPPING_PID=""
CLEANED_UP=0

set_pcd_save() {
    local value="$1"
    local matches
    matches="$(grep -Ec '^[[:space:]]*pcd_save_en:[[:space:]]*(true|false)([[:space:]]*(#.*)?)?$' "${MID360_CONFIG}")"
    if [[ "${matches}" -ne 1 ]]; then
        echo "错误：${MID360_CONFIG} 中应当恰好有一个 pcd_save_en 布尔配置，实际找到 ${matches} 个。" >&2
        return 1
    fi

    sed -i -E \
        "s/^([[:space:]]*pcd_save_en:[[:space:]]*)(true|false)([[:space:]]*(#.*)?)$/\\1${value}\\3/" \
        "${MID360_CONFIG}"
    grep -Eq "^[[:space:]]*pcd_save_en:[[:space:]]*${value}([[:space:]]*(#.*)?)?$" \
        "${MID360_CONFIG}"
}

process_alive() {
    local pid="$1"
    [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

stop_process_group() {
    local name="$1"
    local pid="$2"
    local attempt

    if ! process_alive "${pid}"; then
        return 0
    fi

    echo "停止${name}（PID ${pid}）……"
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true

    # roslaunch 通常会在数秒内完成节点退出；给建图节点时间写完 PCD。
    for attempt in $(seq 1 200); do
        if ! process_alive "${pid}"; then
            wait "${pid}" 2>/dev/null || true
            return 0
        fi
        sleep 0.1
    done

    echo "${name}未在20秒内退出，发送 TERM。" >&2
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
}

cleanup() {
    local exit_code=$?
    if [[ "${CLEANED_UP}" -eq 1 ]]; then
        return
    fi
    CLEANED_UP=1
    trap - EXIT INT TERM

    echo
    echo "正在安全结束建图……"
    # 顺序很重要：先让 S-FAST_LIO 利用仍在线的雷达完成退出和地图保存。
    stop_process_group "S-FAST_LIO建图" "${MAPPING_PID}"
    stop_process_group "Livox MID360驱动" "${LIVOX_PID}"
    stop_process_group "Bunker底盘驱动" "${BUNKER_PID}"

    if set_pcd_save false; then
        echo "已恢复 pcd_save_en: false（可安全用于重定位）。"
    else
        echo "警告：无法自动恢复 pcd_save_en，请手动检查 ${MID360_CONFIG}。" >&2
    fi

    echo "地图目录：${PCD_DIR}"
    for map_file in GlobalMap.pcd GlobalMap_ikdtree.pcd; do
        if [[ -f "${PCD_DIR}/${map_file}" ]]; then
            ls -lh --time-style=long-iso "${PCD_DIR}/${map_file}"
        fi
    done
    echo "建图流程已结束。"
    exit "${exit_code}"
}

require_file() {
    local path="$1"
    if [[ ! -f "${path}" ]]; then
        echo "错误：缺少文件 ${path}" >&2
        exit 1
    fi
}

start_launch() {
    local name="$1"
    local command="$2"
    local wait_seconds="$3"
    local output_variable="$4"
    local pid

    echo "启动${name}……"
    setsid /bin/bash -lc "${command}" &
    pid=$!
    sleep "${wait_seconds}"
    if ! process_alive "${pid}"; then
        wait "${pid}" || true
        echo "错误：${name}启动后立即退出，请查看上方 roslaunch 日志。" >&2
        return 1
    fi
    printf -v "${output_variable}" '%s' "${pid}"
}

main() {
    require_file "${ROS_SETUP}"
    require_file "${BUNKER_SETUP}"
    require_file "${LIVOX_SETUP}"
    require_file "${FASTLIO_SETUP}"
    require_file "${MID360_CONFIG}"
    command -v setsid >/dev/null

    trap cleanup EXIT INT TERM

    echo "============================================================"
    echo "Bunker Mini + MID360 + S-FAST_LIO 一键建图"
    echo "PCD保存目录：${PCD_DIR}"
    echo "RViz：${RVIZ_ENABLE}（可用 RVIZ_ENABLE=false 关闭）"
    if [[ -f "${PCD_DIR}/GlobalMap.pcd" ]]; then
        echo "注意：停止本次建图时，现有 GlobalMap*.pcd 将被新地图覆盖。"
    fi
    echo "============================================================"

    set_pcd_save true
    echo "已设置 pcd_save_en: true。"

    start_launch \
        "Bunker底盘驱动" \
        "source '${ROS_SETUP}'; source '${BUNKER_SETUP}'; exec roslaunch bunker_bringup bunker_robot_base.launch publish_tf:=false" \
        3 BUNKER_PID

    start_launch \
        "Livox MID360驱动" \
        "source '${ROS_SETUP}'; source '${LIVOX_SETUP}'; exec roslaunch livox_ros_driver2 msg_MID360.launch" \
        4 LIVOX_PID

    start_launch \
        "S-FAST_LIO建图" \
        "source '${ROS_SETUP}'; source '${FASTLIO_SETUP}'; exec roslaunch sfast_lio mapping_mid360.launch rviz:='${RVIZ_ENABLE}'" \
        3 MAPPING_PID

    echo
    echo "三个模块均已启动。开始移动小车建图。"
    echo "完成后在本终端按 Ctrl+C，脚本会保存地图并恢复重定位配置。"

    # 任一 roslaunch 意外退出就结束整套流程，避免留下半套节点。
    set +e
    wait -n "${BUNKER_PID}" "${LIVOX_PID}" "${MAPPING_PID}"
    local status=$?
    set -e
    echo "检测到一个启动模块退出，正在关闭其余模块。" >&2
    return "${status}"
}

main "$@"
