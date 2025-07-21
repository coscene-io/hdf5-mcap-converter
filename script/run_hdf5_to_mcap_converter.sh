#!/bin/bash

set -o pipefail # Exit with error if any command in a pipe fails

# This script is executed INSIDE the Docker container by an Argo Workflow.
# Its purpose is to take arguments for directories and filenames,
# construct full paths within the container, and then run the Python converter script.

source /opt/ros/humble/setup.bash

DEBUG_FLAG=""
LOG_TO_FILE_FLAG=false

# Initialize variables for paths and filenames
INPUT_DIR=""
OUTPUT_DIR=""
CONFIG_FILE_NAME=""
HDF5_INPUT_SPEC="" # Renamed from HDF5_FILE_NAME, will store raw -f input
URDF_FILE_NAME=""
OUTPUT_MCAP_TEMPLATE="" # Renamed from OUTPUT_MCAP_FILE_NAME, will store raw -m input
CUSTOM_HANDLERS_ARG=""
HDF5_FILES_TO_PROCESS=() # Array to store actual HDF5 filenames

# --- Temporary URDF Download Cleanup (for in-container downloads) ---
TEMP_URDF_DOWNLOAD_DIR="" # Initialize for trap cleanup

cleanup_temp_urdf_dir() {
    if [ -n "$TEMP_URDF_DOWNLOAD_DIR" ] && [ -d "$TEMP_URDF_DOWNLOAD_DIR" ]; then
        echo "Cleaning up temporary URDF download directory (in container): $TEMP_URDF_DOWNLOAD_DIR"
        rm -rf "$TEMP_URDF_DOWNLOAD_DIR"
    fi
}
trap cleanup_temp_urdf_dir EXIT SIGHUP SIGINT SIGQUIT SIGTERM
# --- End Temporary URDF Download Cleanup ---

USAGE_STRING="Usage: $0 -i <input_dir> -o <output_dir> -c <config.yaml> -f <hdf5_filespec> -u <robot.urdf> -m <output_mcap_template> [--custom-handlers <handlers>] [--log-to-file] [--debug]"

if [ $# -eq 0 ]; then
    echo "Error: No arguments provided to run_converter.sh (running inside container)."
    echo -e "$USAGE_STRING"
    exit 1
fi

while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -i|--input-dir)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      INPUT_DIR="$2"
      shift; shift
      ;;
    -o|--output-dir)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      OUTPUT_DIR="$2"
      shift; shift
      ;;
    -c|--config)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      CONFIG_FILE_NAME="$2"
      shift; shift
      ;;
    -f|--hdf5)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      HDF5_INPUT_SPEC="$2" # Store the raw input spec
      shift; shift
      ;;
    -u|--urdf)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      URDF_FILE_NAME="$2"
      shift; shift
      ;;
    -m|--output-file)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      OUTPUT_MCAP_TEMPLATE="$2" # Store the raw template
      shift; shift
      ;;
    --custom-handlers)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then
        CUSTOM_HANDLERS_ARG=""
        shift
      else
        CUSTOM_HANDLERS_ARG="$2"
        shift; shift
      fi
      ;;
    --log-to-file)
      LOG_TO_FILE_FLAG=true
      shift
      ;;
    --debug)
      DEBUG_FLAG="--debug"
      shift
      ;;
    -h|--help)
      echo -e "$USAGE_STRING"
      echo "This script is run inside the Docker container by Argo."
      echo "Options:"
      echo "  -i, --input-dir  <path>       Base directory for input files (HDF5, URDF, Config YAML). Paths are inside container."
      echo "  -o, --output-dir <path>       Base directory for output MCAP and log files. Paths are inside container."
      echo "  -c, --config <filename>       Configuration YAML file name (relative to input-dir)."
      echo "  -f, --hdf5   <filespec>       Input HDF5 file(s). Can be a single filename, a comma-separated list (no spaces),"
      echo "                                or a glob pattern (e.g., \"*.hdf5\") relative to input-dir."
      echo "  -u, --urdf   <path_or_url>    Input URDF. Can be a file path relative to <input-dir> or an http(s) URL. If a URL, it is downloaded temporarily. The content of this URDF file is also published to the /robot_description topic in the MCAP."
      echo "  -m, --output-file <template>  Output MCAP filename template (e.g., \"output.mcap\" or \"output\")."
      echo "                                For HDF5 \"input.hdf5\", will generate \"output_input.mcap\"."
      echo "  --custom-handlers <handlers>  Custom handler scripts (comma-separated list or glob pattern,"
      echo "                                e.g., \"handler1.py,handler2.py\" or \"*.py\")."
      echo "  --log-to-file                 Enable logging Python script output to a file in <output-dir>."
      echo "  --debug                       Enable debug output from the Python converter script."
      echo "  -h, --help                    Show this help message."
      exit 0
      ;;
    *)    # unknown option
      echo "Error: Unknown option: $1" >&2
      echo -e "$USAGE_STRING"
      exit 1
      ;;
  esac
done

# Check if all required arguments were provided
if [ -z "$INPUT_DIR" ] || [ -z "$OUTPUT_DIR" ] || [ -z "$CONFIG_FILE_NAME" ] || [ -z "$HDF5_INPUT_SPEC" ] || [ -z "$URDF_FILE_NAME" ] || [ -z "$OUTPUT_MCAP_TEMPLATE" ]; then
    echo "Error: Missing one or more required arguments (-i, -o, -c, -f, -u, -m)." >&2
    echo -e "$USAGE_STRING"
    exit 1
fi

# Process HDF5_INPUT_SPEC to populate HDF5_FILES_TO_PROCESS
if [[ "$HDF5_INPUT_SPEC" == *","* ]]; then
    IFS=',' read -r -a HDF5_FILES_TO_PROCESS <<< "$HDF5_INPUT_SPEC"
else
    # Handle glob or single file. Need to change directory for glob to work correctly.
    # Store current dir, cd, glob, then cd back
    ORIGINAL_PWD=$(pwd)
    if [ -d "$INPUT_DIR" ]; then
        cd "$INPUT_DIR"
        # Use a loop to correctly handle filenames with spaces if glob expands to them
        for f_glob in $HDF5_INPUT_SPEC; do
            if [ -f "$f_glob" ]; then # Check if glob match is an actual file
                HDF5_FILES_TO_PROCESS+=("$f_glob")
            fi
        done
        cd "$ORIGINAL_PWD"
    else
        echo "Error: Input directory '$INPUT_DIR' not found." >&2
        exit 1
    fi

    # If after globbing, HDF5_FILES_TO_PROCESS is empty, it might be a single file not caught by glob
    # or the glob didn't match. If it was not a glob, add it.
    if [ ${#HDF5_FILES_TO_PROCESS[@]} -eq 0 ] && [[ "$HDF5_INPUT_SPEC" != *"*"* ]] && [[ "$HDF5_INPUT_SPEC" != *"?"* ]]; then
        # Check if this single file exists relative to input_dir
        if [ -f "$INPUT_DIR/$HDF5_INPUT_SPEC" ]; then
             HDF5_FILES_TO_PROCESS+=("$HDF5_INPUT_SPEC")
        fi
    fi
fi

if [ ${#HDF5_FILES_TO_PROCESS[@]} -eq 0 ]; then
    echo "Error: No HDF5 files found or specified matching '$HDF5_INPUT_SPEC' in '$INPUT_DIR'." >&2
    echo -e "$USAGE_STRING"
    exit 1
fi

# Process output MCAP template
MCAP_TEMPLATE_BASE="${OUTPUT_MCAP_TEMPLATE%.mcap}" # remove .mcap if present
if [ "$MCAP_TEMPLATE_BASE" == "$OUTPUT_MCAP_TEMPLATE" ]; then # no .mcap was present
    MCAP_TEMPLATE_PREFIX="${OUTPUT_MCAP_TEMPLATE}_"
else # .mcap was present
    MCAP_TEMPLATE_PREFIX="${MCAP_TEMPLATE_BASE}_"
fi


# --- URDF and Config File Path Resolution (inside container) ---
# URDF_FILE_NAME is the raw input from -u argument
# CONFIG_FILE_NAME is the raw input from -c argument

if [[ "$URDF_FILE_NAME" == http://* || "$URDF_FILE_NAME" == https://* ]]; then
    echo "URDF argument is a URL: $URDF_FILE_NAME (to be downloaded inside container)"
    TEMP_URDF_DOWNLOAD_DIR=$(mktemp -d) # Creates in /tmp inside container
    if [ ! -d "$TEMP_URDF_DOWNLOAD_DIR" ]; then
        echo "Error: Could not create system temporary directory for URDF download (in container)." >&2
        exit 1
    fi
    echo "Created temporary URDF download directory (in container): $TEMP_URDF_DOWNLOAD_DIR"

    URL_NO_QUERY="${URDF_FILE_NAME%%\?*}"
    DOWNLOADED_URDF_BASENAME=$(basename "$URL_NO_QUERY")
    if [ -z "$DOWNLOADED_URDF_BASENAME" ] || [[ "$DOWNLOADED_URDF_BASENAME" == "." ]] || [[ "$DOWNLOADED_URDF_BASENAME" == "/" ]]; then
        DOWNLOADED_URDF_BASENAME="downloaded.urdf"
    fi
    ACTUAL_URDF_FILE_PATH_IN_CONTAINER="$TEMP_URDF_DOWNLOAD_DIR/$DOWNLOADED_URDF_BASENAME"

    echo "Attempting to download URDF to: $ACTUAL_URDF_FILE_PATH_IN_CONTAINER using Python (in container)"
    PYTHON_DOWNLOAD_SCRIPT='import sys, urllib.request; url, dest_path = sys.argv[1], sys.argv[2];
try:
    print(f"Python (in-container): Downloading {url} to {dest_path}", file=sys.stderr);
    urllib.request.urlretrieve(url, dest_path);
    print(f"Python (in-container): Successfully downloaded {url} to {dest_path}", file=sys.stderr);
except Exception as e:
    print(f"Python (in-container): Error downloading {url} - {e}", file=sys.stderr);
    sys.exit(1)'

    if python3 -c "$PYTHON_DOWNLOAD_SCRIPT" "$URDF_FILE_NAME" "$ACTUAL_URDF_FILE_PATH_IN_CONTAINER"; then
        echo "URDF downloaded successfully from URL (in container)."
        FULL_URDF_PATH="$ACTUAL_URDF_FILE_PATH_IN_CONTAINER"
    else
        python_exit_code=$?
        echo "Error: Failed to download URDF from URL using Python (in container): $URDF_FILE_NAME (Python exit code: $python_exit_code)" >&2
        exit 1
    fi
else # URDF_FILE_NAME is a local file path relative to INPUT_DIR
    echo "URDF argument is a local file path (in container): $URDF_FILE_NAME"
    FULL_URDF_PATH="$INPUT_DIR/$URDF_FILE_NAME"
fi

# Config file path is relative to INPUT_DIR
FULL_CONFIG_PATH="$INPUT_DIR/$CONFIG_FILE_NAME"
# --- End URDF and Config File Path Resolution ---

# Create output directory within container if it doesn't exist
mkdir -p "$OUTPUT_DIR"

LOG_FILE=""
if $LOG_TO_FILE_FLAG; then
    LOG_FILE="$OUTPUT_DIR/converter_log_$(date +%Y%m%d_%H%M%S).log"
    # Create the log file, or clear it if it somehow exists (though timestamp should make it unique)
    echo "Python script output log. Timestamp: $(date)" > "$LOG_FILE"
    echo "Logging Python script output to $LOG_FILE (inside container)"
fi

# Check if common input files exist *inside the container*
for f_path in "$FULL_URDF_PATH" "$FULL_CONFIG_PATH"; do
    if [ ! -f "$f_path" ]; then echo "Error: Common input file not found inside container: $f_path" >&2; exit 1; fi
done

echo "Using URDF file (container): $FULL_URDF_PATH"
echo "Using configuration file (container): $FULL_CONFIG_PATH"

# Python script to execute
PYTHON_SCRIPT="/script/hdf5_to_mcap_converter.py"

ALL_CONVERSIONS_SUCCESSFUL=true

for CURRENT_HDF5_BASENAME in "${HDF5_FILES_TO_PROCESS[@]}"; do
    echo "" # Add a blank line for readability between file processing
    FULL_HDF5_PATH="$INPUT_DIR/$CURRENT_HDF5_BASENAME"

    if [ ! -f "$FULL_HDF5_PATH" ]; then
        echo "Error: Input HDF5 file not found inside container: $FULL_HDF5_PATH" >&2
        ALL_CONVERSIONS_SUCCESSFUL=false
        continue # Skip to the next file
    fi

    # Use only the base filename (no directory) and strip .hdf5 extension
    HDF5_FILE_BASE_NO_EXT="$(basename "${CURRENT_HDF5_BASENAME%.hdf5}")"
    CURRENT_OUTPUT_MCAP_FILENAME="${MCAP_TEMPLATE_PREFIX}${HDF5_FILE_BASE_NO_EXT}.mcap"
    FULL_OUTPUT_MCAP_PATH="$OUTPUT_DIR/$CURRENT_OUTPUT_MCAP_FILENAME"

    echo "Processing HDF5 file (container): $FULL_HDF5_PATH"
    echo "Target output MCAP (container): $FULL_OUTPUT_MCAP_PATH"

    CUSTOM_HANDLERS_FLAG=""
    if [ -n "$CUSTOM_HANDLERS_ARG" ]; then
        CUSTOM_HANDLERS_FLAG="--custom-handlers \"$CUSTOM_HANDLERS_ARG\""
    fi

    COMMAND_TO_RUN="python3 $PYTHON_SCRIPT \
        --hdf5-file \"$FULL_HDF5_PATH\" \
        --urdf-file \"$FULL_URDF_PATH\" \
        --output-file \"$FULL_OUTPUT_MCAP_PATH\" \
        --config-file \"$FULL_CONFIG_PATH\" \
        --input-dir \"$INPUT_DIR\" \
        $CUSTOM_HANDLERS_FLAG \
        $DEBUG_FLAG"

    echo "Executing for $CURRENT_HDF5_BASENAME: $COMMAND_TO_RUN"

    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        # Log command before execution for this specific file
        echo "--- Processing $CURRENT_HDF5_BASENAME ---" >> "$LOG_FILE"
        echo "Command: $COMMAND_TO_RUN" >> "$LOG_FILE"
        eval "$COMMAND_TO_RUN" >> "$LOG_FILE" 2>&1
    else
        eval "$COMMAND_TO_RUN"
    fi

    CONVERTER_EXIT_CODE=$?

    if [ $CONVERTER_EXIT_CODE -ne 0 ]; then
        echo ""
        echo "--- ERROR: Python converter script failed for $CURRENT_HDF5_BASENAME with exit code $CONVERTER_EXIT_CODE ---" >&2
        ALL_CONVERSIONS_SUCCESSFUL=false
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
             echo "--- ERROR: Python converter script failed for $CURRENT_HDF5_BASENAME with exit code $CONVERTER_EXIT_CODE ---" >> "$LOG_FILE"
        fi
        # Optionally, could exit immediately: exit $CONVERTER_EXIT_CODE
    else
        echo "Conversion script finished successfully for $CURRENT_HDF5_BASENAME."
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
             echo "Conversion script finished successfully for $CURRENT_HDF5_BASENAME." >> "$LOG_FILE"
        fi
    fi
done

echo "" # Final blank line for clarity

if $ALL_CONVERSIONS_SUCCESSFUL; then
    echo "All HDF5 to MCAP conversions finished successfully."
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "Full log is at $LOG_FILE (inside container)."
    fi
    exit 0
else
    echo "--- ERROR: One or more HDF5 to MCAP conversions failed. ---" >&2
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "--- ERROR: One or more HDF5 to MCAP conversions failed. See details above. ---" >> "$LOG_FILE"
        echo "Full log is at $LOG_FILE (inside container)."
    fi
    exit 1
fi
