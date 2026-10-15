#!/command/with-contenv sh
# Reusable ROS2 service finish script.

set -e
# A robot service stays stopped after exit, including errors in cleanup.
trap 'exit 125' EXIT
exec 2>&1

SERVICE_NAME="${SERVICE_NAME}"
if [ -z "${SERVICE_NAME}" ]; then
    echo "Error: SERVICE_NAME environment variable must be set" >&2
    exit 1
fi

EXIT_CODE=${1:-0}
EXIT_SIGNAL=${2:-0}
echo "[${SERVICE_NAME} finish] Service stopped with exit code $EXIT_CODE, signal $EXIT_SIGNAL"

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

rm -f "${PGID_FILE}"
# Address the supervisor directly: no nested s6-rc lock or reverse-dependency
# transaction. This also stops the logger when the producer crashes.
/command/s6-svc -d "/run/service/${SERVICE_NAME}-log" || true

exit 125
