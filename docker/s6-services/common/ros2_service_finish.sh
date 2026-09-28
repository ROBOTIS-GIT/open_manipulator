#!/command/with-contenv sh
# Reusable ROS2 service finish script.

set -e

SERVICE_NAME="${SERVICE_NAME}"
if [ -z "${SERVICE_NAME}" ]; then
    echo "Error: SERVICE_NAME environment variable must be set" >&2
    exit 1
fi

EXIT_CODE=${1:-unknown}
echo "[${SERVICE_NAME} finish] Service stopped with exit code $EXIT_CODE"

PGID_FILE=/run/${SERVICE_NAME}.pgid
if [ -f "${PGID_FILE}" ]; then
    PGID=$(cat "${PGID_FILE}" 2>/dev/null || echo "")
    if [ -n "${PGID}" ]; then
        echo "[${SERVICE_NAME} finish] Sending SIGTERM to process group ${PGID}"
        kill -TERM -"${PGID}" 2>/dev/null || echo "[${SERVICE_NAME} finish] Warning: kill -TERM -${PGID} failed or group already gone"

        TIMEOUT=30
        SLEEP_INTERVAL=1
        ELAPSED=0

        pgid_alive() {
            kill -0 -"${PGID}" 2>/dev/null
        }

        if pgid_alive; then
            echo "[${SERVICE_NAME} finish] Waiting for process group ${PGID} to exit (timeout: ${TIMEOUT}s)..."
        fi

        while pgid_alive && [ "${ELAPSED}" -lt "${TIMEOUT}" ]; do
            sleep "${SLEEP_INTERVAL}"
            ELAPSED=$((ELAPSED + SLEEP_INTERVAL))
        done

        if pgid_alive; then
            echo "[${SERVICE_NAME} finish] Timeout waiting for process group ${PGID} to exit; sending SIGKILL"
            kill -KILL -"${PGID}" 2>/dev/null || echo "[${SERVICE_NAME} finish] Warning: kill -KILL -${PGID} failed or group already gone"
        else
            echo "[${SERVICE_NAME} finish] Process group ${PGID} fully exited after ${ELAPSED}s"
        fi
    fi
fi

s6-rc -d change "${SERVICE_NAME}-log" 2>/dev/null && echo "[${SERVICE_NAME} finish] ${SERVICE_NAME}-log stopped" || echo "[${SERVICE_NAME} finish] ${SERVICE_NAME}-log not running or already stopped"

exit 0
