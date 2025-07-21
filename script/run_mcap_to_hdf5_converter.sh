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
MCAP_INPUT_SPEC="" # Changed from MCAP_FILE_NAME to support glob/comma-separated specs
URDF_FILE_NAME=""
OUTPUT_HDF5_TEMPLATE="" # Changed from OUTPUT_HDF5_FILE_NAME to support template naming
CUSTOM_HANDLERS_ARG=""
MCAP_FILES_TO_PROCESS=() # Array to store actual MCAP filenames

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

USAGE_STRING="Usage: $0 -i <input_dir> -o <output_dir> -c <config.yaml> -m <mcap_filespec> -u <robot.urdf> -f <output_template.hdf5> [--custom-handlers <handlers>] [--log-to-file] [--debug]

This script runs inside Docker containers for Argo workflows.

Arguments:
  -m, --mcap-file <filespec>    Input MCAP file(s). Can be a single filename, comma-separated list,
                                or glob pattern (e.g., \"*.mcap\")
  -f, --hdf5-file <template>    Output HDF5 filename template (e.g., \"output.hdf5\").
                                For MCAP \"input.mcap\", will generate \"output_input.hdf5\".
"

if [ $# -eq 0 ]; then
    echo "Error: No arguments provided to run_mcap_to_hdf5_converter.sh (running inside container)."
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
    -m|--mcap-file)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      MCAP_INPUT_SPEC="$2"
      shift; shift
      ;;
    -u|--urdf)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      URDF_FILE_NAME="$2"
      shift; shift
      ;;
    -f|--hdf5-file)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      OUTPUT_HDF5_TEMPLATE="$2"
      shift; shift
      ;;
    --custom-handlers)
      CUSTOM_HANDLERS_ARG="$2"
      shift; shift
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
      echo "  -i, --input-dir  <path>       Base directory for input files (MCAP, URDF, Config YAML). Paths are inside container."
      echo "  -o, --output-dir <path>       Base directory for output HDF5 and log files. Paths are inside container."
      echo "  -c, --config <filename>       Configuration YAML file name (relative to input-dir)."
      echo "  -m, --mcap-file <filespec>    Input MCAP file(s). Can be a single filename, comma-separated list, or glob pattern (e.g., \"*.mcap\")."
      echo "  -u, --urdf   <path_or_url>    Input URDF. Can be a file path relative to <input-dir> or an http(s) URL. If a URL, it is downloaded temporarily."
      echo "  -f, --hdf5-file <template>    Output HDF5 filename template (e.g., \"output.hdf5\"). For MCAP \"input.mcap\", will generate \"output_input.hdf5\"."
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
if [ -z "$INPUT_DIR" ] || [ -z "$OUTPUT_DIR" ] || [ -z "$CONFIG_FILE_NAME" ] || [ -z "$MCAP_INPUT_SPEC" ] || [ -z "$URDF_FILE_NAME" ] || [ -z "$OUTPUT_HDF5_TEMPLATE" ]; then
    echo "Error: Missing one or more required arguments (-i, -o, -c, -m, -u, -f)." >&2
    echo -e "$USAGE_STRING"
    exit 1
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

# Process MCAP_INPUT_SPEC to populate MCAP_FILES_TO_PROCESS
if [[ "$MCAP_INPUT_SPEC" == *","* ]]; then
    IFS=',' read -r -a MCAP_FILES_TO_PROCESS <<< "$MCAP_INPUT_SPEC"
else
    # Handle glob or single file
    ORIGINAL_PWD=$(pwd)
    if [ -d "$INPUT_DIR" ]; then
        cd "$INPUT_DIR"
        # Use a loop to correctly handle filenames with spaces if glob expands to them
        for f_glob in $MCAP_INPUT_SPEC; do
            if [ -f "$f_glob" ]; then # Check if glob match is an actual file
                MCAP_FILES_TO_PROCESS+=("$f_glob")
            fi
        done
        cd "$ORIGINAL_PWD"
    else
        echo "Error: Input directory '$INPUT_DIR' not found." >&2
        exit 1
    fi

    # If after globbing, MCAP_FILES_TO_PROCESS is empty, it might be a single file not caught by glob
    if [ ${#MCAP_FILES_TO_PROCESS[@]} -eq 0 ] && [[ "$MCAP_INPUT_SPEC" != *"*"* ]] && [[ "$MCAP_INPUT_SPEC" != *"?"* ]]; then
        # Check if this single file exists relative to input_dir
        if [ -f "$INPUT_DIR/$MCAP_INPUT_SPEC" ]; then
            MCAP_FILES_TO_PROCESS+=("$MCAP_INPUT_SPEC")
        fi
    fi
fi

if [ ${#MCAP_FILES_TO_PROCESS[@]} -eq 0 ]; then
    echo "Error: No MCAP files found or specified matching '$MCAP_INPUT_SPEC'." >&2
    echo -e "$USAGE_STRING"
    exit 1
fi

# Process output HDF5 template - ensure it has .hdf5 extension
if [[ "$OUTPUT_HDF5_TEMPLATE" != *.hdf5 ]]; then
    OUTPUT_HDF5_TEMPLATE="${OUTPUT_HDF5_TEMPLATE}.hdf5"
fi

# Create output directory within container if it doesn't exist
mkdir -p "$OUTPUT_DIR"

LOG_FILE=""
if $LOG_TO_FILE_FLAG; then
    LOG_FILE="$OUTPUT_DIR/mcap_to_hdf5_converter_log_$(date +%Y%m%d_%H%M%S).log"
    # Create the log file, or clear it if it somehow exists (though timestamp should make it unique)
    echo "Python script output log. Timestamp: $(date)" > "$LOG_FILE"
    echo "Processing files: ${MCAP_FILES_TO_PROCESS[@]}" >> "$LOG_FILE"
    echo "" >> "$LOG_FILE"
    echo "Logging Python script output to $LOG_FILE (inside container)"
fi

# Check if common input files exist *inside the container*
for f_path in "$FULL_URDF_PATH" "$FULL_CONFIG_PATH"; do
    if [ ! -f "$f_path" ]; then echo "Error: Input file not found inside container: $f_path" >&2; exit 1; fi
done

echo "Using URDF file (container): $FULL_URDF_PATH"
echo "Using configuration file (container): $FULL_CONFIG_PATH"

# Python script to execute
PYTHON_SCRIPT="/script/mcap_to_hdf5_converter.py"

echo "Processing MCAP files: ${MCAP_FILES_TO_PROCESS[@]}"

CUSTOM_HANDLERS_FLAG=""
if [ -n "$CUSTOM_HANDLERS_ARG" ]; then
    CUSTOM_HANDLERS_FLAG="--custom-handlers \"$CUSTOM_HANDLERS_ARG\""
fi

ALL_CONVERSIONS_SUCCESSFUL=true

# Process each MCAP file
for CURRENT_MCAP_BASENAME in "${MCAP_FILES_TO_PROCESS[@]}"; do
    echo ""
    echo "Processing MCAP file: $CURRENT_MCAP_BASENAME"
    
    # Construct full path for current MCAP file
    FULL_MCAP_PATH="$INPUT_DIR/$CURRENT_MCAP_BASENAME"
    
    # Check if MCAP file exists
    if [ ! -f "$FULL_MCAP_PATH" ]; then
        echo "Error: Input MCAP file not found inside container: $FULL_MCAP_PATH" >&2
        ALL_CONVERSIONS_SUCCESSFUL=false
        continue
    fi
    
    # Generate meaningful output filename from template and MCAP basename
    MCAP_FILE_BASE_NO_EXT="$(basename "${CURRENT_MCAP_BASENAME%.mcap}")"
    
    # Simplified output filename - just add the basename before extension
    OUTPUT_HDF5_FILENAME="${OUTPUT_HDF5_TEMPLATE%.hdf5}_${MCAP_FILE_BASE_NO_EXT}.hdf5"
    
    # Construct full output path
    FULL_OUTPUT_PATH="$OUTPUT_DIR/$OUTPUT_HDF5_FILENAME"
    
    echo "Target output HDF5: $FULL_OUTPUT_PATH"

    COMMAND_TO_RUN="python3 $PYTHON_SCRIPT \
        --mcap-file \"$FULL_MCAP_PATH\" \
        --urdf-file \"$FULL_URDF_PATH\" \
        --hdf5-file \"$FULL_OUTPUT_PATH\" \
        --config-file \"$FULL_CONFIG_PATH\" \
        --input-dir \"$INPUT_DIR\" \
        --output-dir \"$OUTPUT_DIR\" \
        $CUSTOM_HANDLERS_FLAG \
        $DEBUG_FLAG"

    echo "Executing: $COMMAND_TO_RUN"

    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        # Log command before execution
        echo "--- Processing $CURRENT_MCAP_BASENAME ---" >> "$LOG_FILE"
        echo "Command: $COMMAND_TO_RUN" >> "$LOG_FILE"
        eval "$COMMAND_TO_RUN" >> "$LOG_FILE" 2>&1
    else
        eval "$COMMAND_TO_RUN"
    fi

    CONVERTER_EXIT_CODE=$?

    if [ $CONVERTER_EXIT_CODE -ne 0 ]; then
        echo ""
        echo "--- ERROR: Python converter script failed for $CURRENT_MCAP_BASENAME with exit code $CONVERTER_EXIT_CODE ---" >&2
        ALL_CONVERSIONS_SUCCESSFUL=false
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
             echo "--- ERROR: Python converter script failed for $CURRENT_MCAP_BASENAME with exit code $CONVERTER_EXIT_CODE ---" >> "$LOG_FILE"
        fi
    else
        echo "Conversion complete for $CURRENT_MCAP_BASENAME. Output at: $FULL_OUTPUT_PATH"
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
             echo "Conversion complete for $CURRENT_MCAP_BASENAME. Output at: $FULL_OUTPUT_PATH" >> "$LOG_FILE"
        fi
    fi
done

echo ""
if $ALL_CONVERSIONS_SUCCESSFUL; then
    echo "✅ All MCAP to HDF5 conversions finished successfully!"
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "Full log is at $LOG_FILE (inside container)."
    fi
    exit 0
else
    echo "❌ One or more MCAP to HDF5 conversions failed." >&2
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "--- ERROR: One or more MCAP to HDF5 conversions failed. See details above. ---" >> "$LOG_FILE"
        echo "Full log is at $LOG_FILE (inside container)."
    fi
    exit 1
fi 