#!/usr/bin/env bash
# Start the independent saver and one mapper; sensor drivers are started separately.
set -eo pipefail
task_project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$task_project_root/map_autosave/start_mapping_saved.sh" "$@"
