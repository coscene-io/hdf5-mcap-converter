#!/bin/bash

set -o pipefail # Exit with error if any command in a pipe fails

# This script builds and runs the HDF5 to MCAP converter using Python 3.10
# It orchestrates Docker to run the conversion.

DEBUG_FLAG=""
BUILD_FLAG=false

# --- Argument Parsing ---
CONFIG_FILE_NAME=""
HDF5_INPUT_SPEC="" # Renamed from HDF5_FILE_NAME, will store raw -f input
URDF_FILE_NAME=""
OUTPUT_MCAP_TEMPLATE="" # Renamed from OUTPUT_FILE_NAME, will store raw -m input
INPUT_DIR_ARG=""
OUTPUT_DIR_ARG=""
CUSTOM_HANDLERS_ARG=""
LOG_TO_FILE_FLAG=false # Default to not logging to file
HDF5_FILES_TO_PROCESS=() # Array to store actual HDF5 filenames

# --- Temporary URDF Download Cleanup ---
TEMP_URDF_DOWNLOAD_DIR="" # Initialize for trap cleanup

cleanup_temp_urdf_dir() {
    if [ -n "$TEMP_URDF_DOWNLOAD_DIR" ] && [ -d "$TEMP_URDF_DOWNLOAD_DIR" ]; then
        echo "Cleaning up temporary URDF download directory: $TEMP_URDF_DOWNLOAD_DIR"
        rm -rf "$TEMP_URDF_DOWNLOAD_DIR"
    fi
}
trap cleanup_temp_urdf_dir EXIT SIGHUP SIGINT SIGQUIT SIGTERM
# --- End Temporary URDF Download Cleanup ---

USAGE_STRING="Usage: $0 -c <config.yaml> -f <hdf5_filespec> -u <robot.urdf> -m <output_mcap_template> [-i <input_dir>] [-o <output_dir>] [--custom-handlers <handlers>] [--log-to-file] [--debug] [--build]"

if [ $# -eq 0 ]; then
    echo "Error: No arguments provided."
    echo -e "$USAGE_STRING"
    exit 1
fi

while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -i|--input-dir)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      INPUT_DIR_ARG="$2"
      shift; shift
      ;;
    -o|--output-dir)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      OUTPUT_DIR_ARG="$2"
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
    --build)
      BUILD_FLAG=true
      shift
      ;;
    -h|--help)
      echo -e "$USAGE_STRING"
      echo "Options:"
      echo "  -i, --input-dir  <path>       Base directory for input files (HDF5, URDF, Config YAML). If specified, filenames are relative to this."
      echo "                                If not, assumes default subdirs (hdf5/, urdf/, config/) under ./hdf5-mcap-converter."
      echo "  -o, --output-dir <path>       Base directory for output MCAP and log files. If specified, output filename is relative to this."
      echo "                                If not, assumes default subdirs (output/, log/) under ./hdf5-mcap-converter."
      echo "  -c, --config <filename>       Configuration YAML file name. (Required)"
      echo "  -f, --hdf5   <filespec>       Input HDF5 file(s). Can be a single filename, a comma-separated list (no spaces),"
      echo "                                or a glob pattern (e.g., \"*.hdf5\") relative to input-dir."
      echo "  -u, --urdf   <path_or_url>    Input URDF. Can be a local file path or an http(s) URL. (Required) The content of this URDF file is also published to the /robot_description topic in the MCAP."
      echo "  -m, --output-file <template>  Output MCAP filename template (e.g., \"output.mcap\" or \"output\")."
      echo "                                For HDF5 \"input.hdf5\", will generate \"output_input.mcap\"."
      echo "  --custom-handlers <handlers>  Custom handler scripts (comma-separated list or glob pattern,"
      echo "                                e.g., \"handler1.py,handler2.py\" or \"*.py\")."
      echo "  --log-to-file                 Enable logging converter output to a file."
      echo "  --debug                       Enable debug output from the Python converter script."
      echo "  --build                       Force rebuild of the Docker image."
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
if [ -z "$CONFIG_FILE_NAME" ] || [ -z "$HDF5_INPUT_SPEC" ] || [ -z "$URDF_FILE_NAME" ] || [ -z "$OUTPUT_MCAP_TEMPLATE" ]; then
    echo "Error: Missing one or more required arguments (-c, -f, -u, -m)." >&2
    echo -e "$USAGE_STRING"
    exit 1
fi

# Determine script directory and project base (hdf5-mcap-converter directory)
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_BASE_DIR=$(dirname "$SCRIPT_DIR") # This is hdf5-mcap-converter

# Resolve EFFECTIVE_INPUT_DIR and EFFECTIVE_OUTPUT_DIR
if [ -n "$INPUT_DIR_ARG" ]; then
    EFFECTIVE_INPUT_DIR=$(readlink -m "$INPUT_DIR_ARG")
else
    EFFECTIVE_INPUT_DIR="$PROJECT_BASE_DIR"
fi

if [ -n "$OUTPUT_DIR_ARG" ]; then
    EFFECTIVE_OUTPUT_DIR=$(readlink -m "$OUTPUT_DIR_ARG")
else
    EFFECTIVE_OUTPUT_DIR="$PROJECT_BASE_DIR"
fi

echo "Effective Input Directory: $EFFECTIVE_INPUT_DIR"
echo "Effective Output Directory: $EFFECTIVE_OUTPUT_DIR"

# Process HDF5_INPUT_SPEC to populate HDF5_FILES_TO_PROCESS
if [[ "$HDF5_INPUT_SPEC" == *","* ]]; then
    IFS=',' read -r -a HDF5_FILES_TO_PROCESS <<< "$HDF5_INPUT_SPEC"
else
    # Handle glob or single file
    ORIGINAL_PWD=$(pwd)
    if [ -d "$EFFECTIVE_INPUT_DIR" ]; then
        cd "$EFFECTIVE_INPUT_DIR"
        # Use a loop to correctly handle filenames with spaces if glob expands to them
        for f_glob in $HDF5_INPUT_SPEC; do
            if [ -f "$f_glob" ]; then # Check if glob match is an actual file
                HDF5_FILES_TO_PROCESS+=("$f_glob")
            fi
        done
        cd "$ORIGINAL_PWD"
    else
        echo "Error: Input directory '$EFFECTIVE_INPUT_DIR' not found." >&2
        exit 1
    fi

    # If after globbing, HDF5_FILES_TO_PROCESS is empty, it might be a single file not caught by glob
    if [ ${#HDF5_FILES_TO_PROCESS[@]} -eq 0 ] && [[ "$HDF5_INPUT_SPEC" != *"*"* ]] && [[ "$HDF5_INPUT_SPEC" != *"?"* ]]; then
        # Check if this single file exists relative to input_dir
        if [ -n "$INPUT_DIR_ARG" ]; then
            if [ -f "$EFFECTIVE_INPUT_DIR/$HDF5_INPUT_SPEC" ]; then
                HDF5_FILES_TO_PROCESS+=("$HDF5_INPUT_SPEC")
            fi
        else
            if [ -f "$PROJECT_BASE_DIR/hdf5/$HDF5_INPUT_SPEC" ]; then
                HDF5_FILES_TO_PROCESS+=("hdf5/$HDF5_INPUT_SPEC")
            fi
        fi
    fi
fi

if [ ${#HDF5_FILES_TO_PROCESS[@]} -eq 0 ]; then
    echo "Error: No HDF5 files found or specified matching '$HDF5_INPUT_SPEC'." >&2
    echo -e "$USAGE_STRING"
    exit 1
fi

# Process output MCAP template - much simpler approach now
if [[ "$OUTPUT_MCAP_TEMPLATE" != *.mcap ]]; then
    OUTPUT_MCAP_TEMPLATE="${OUTPUT_MCAP_TEMPLATE}.mcap"
fi

# --- URDF and Config File Path Resolution ---
# URDF_FILE_NAME is the raw input from -u argument
# CONFIG_FILE_NAME is the raw input from -c argument
ACTUAL_URDF_FILE_PATH_ON_HOST="" # This will hold the final path for URDF

if [[ "$URDF_FILE_NAME" == http://* || "$URDF_FILE_NAME" == https://* ]]; then
    echo "URDF argument is a URL: $URDF_FILE_NAME"
    # Create a unique temporary directory for this run using system default
    TEMP_URDF_DOWNLOAD_DIR=$(mktemp -d)
    if [ ! -d "$TEMP_URDF_DOWNLOAD_DIR" ]; then
        echo "Error: Could not create system temporary directory for URDF download." >&2
        exit 1
    fi
    echo "Created temporary URDF download directory: $TEMP_URDF_DOWNLOAD_DIR"

    DOWNLOADED_URDF_BASENAME=$(basename "$URDF_FILE_NAME")
    # Handle cases where basename might be empty or problematic (e.g. if URL ends with / or has query params)
    # A simple approach: remove query string, then basename
    URL_NO_QUERY="${URDF_FILE_NAME%%\?*}"
    DOWNLOADED_URDF_BASENAME=$(basename "$URL_NO_QUERY")

    if [ -z "$DOWNLOADED_URDF_BASENAME" ] || [[ "$DOWNLOADED_URDF_BASENAME" == "." ]] || [[ "$DOWNLOADED_URDF_BASENAME" == "/" ]]; then
        DOWNLOADED_URDF_BASENAME="downloaded.urdf"
    fi
    ACTUAL_URDF_FILE_PATH_ON_HOST="$TEMP_URDF_DOWNLOAD_DIR/$DOWNLOADED_URDF_BASENAME"

    echo "Attempting to download URDF to: $ACTUAL_URDF_FILE_PATH_ON_HOST using Python"
    # Use Python to download
    PYTHON_DOWNLOAD_SCRIPT='import sys, urllib.request; url, dest_path = sys.argv[1], sys.argv[2];
try:
    print(f"Python: Downloading {url} to {dest_path}", file=sys.stderr)
    urllib.request.urlretrieve(url, dest_path)
    print(f"Python: Successfully downloaded {url} to {dest_path}", file=sys.stderr)
except Exception as e:
    print(f"Python: Error downloading {url} - {e}", file=sys.stderr)
    sys.exit(1)'

    if python3 -c "$PYTHON_DOWNLOAD_SCRIPT" "$URDF_FILE_NAME" "$ACTUAL_URDF_FILE_PATH_ON_HOST"; then
        echo "URDF downloaded successfully from URL."
    else
        python_exit_code=$?
        echo "Error: Failed to download URDF from URL using Python: $URDF_FILE_NAME (Python exit code: $python_exit_code)" >&2
        # The trap will attempt cleanup
        exit 1
    fi
else # URDF_FILE_NAME is a local file path
    echo "URDF argument is a local file path: $URDF_FILE_NAME"
    if [ -n "$INPUT_DIR_ARG" ]; then
        ACTUAL_URDF_FILE_PATH_ON_HOST=$(readlink -m "$EFFECTIVE_INPUT_DIR/$URDF_FILE_NAME")
    else
        ACTUAL_URDF_FILE_PATH_ON_HOST=$(readlink -m "$PROJECT_BASE_DIR/urdf/$URDF_FILE_NAME")
    fi
fi

# Assign to the main URDF_FILE_PATH_HOST variable used by the rest of the script
URDF_FILE_PATH_HOST="$ACTUAL_URDF_FILE_PATH_ON_HOST"

# Config file path resolution (remains as per original logic relative to URDF_FILE_PATH_HOST determination)
if [ -n "$INPUT_DIR_ARG" ]; then # Custom input dir means filenames are directly inside it
    CONFIG_FILE_PATH_HOST=$(readlink -m "$EFFECTIVE_INPUT_DIR/$CONFIG_FILE_NAME")
else # Default structure with subdirectories relative to PROJECT_BASE_DIR
    CONFIG_FILE_PATH_HOST=$(readlink -m "$PROJECT_BASE_DIR/config/$CONFIG_FILE_NAME")
fi
# --- End URDF and Config File Path Resolution ---

LOG_FILE=""
if $LOG_TO_FILE_FLAG; then
    if [ -n "$OUTPUT_DIR_ARG" ]; then
        LOG_DIR_HOST_FOR_FILE_LOGGING="$EFFECTIVE_OUTPUT_DIR"
    else
        LOG_DIR_HOST_FOR_FILE_LOGGING="$PROJECT_BASE_DIR/log"
    fi
    mkdir -p "$LOG_DIR_HOST_FOR_FILE_LOGGING"
    LOG_FILE="$LOG_DIR_HOST_FOR_FILE_LOGGING/converter_log_$(date +%Y%m%d_%H%M%S).log"
    touch "$LOG_FILE"
    echo "Logging output to $LOG_FILE"
fi

# Define fixed container paths for common input files - these never change
CONTAINER_HDF5_PATH="/mnt/data.hdf5"
CONTAINER_URDF_PATH="/mnt/robot.urdf"
CONTAINER_CONFIG_PATH="/mnt/config.yaml"
CONTAINER_OUTPUT_DIR="/mnt_out" # Important: This is the root dir in the container for output

# Build the Docker image (context is $PROJECT_BASE_DIR)
if $BUILD_FLAG || ! docker image inspect hdf5tomcap &>/dev/null; then
    echo "Building Docker image from $PROJECT_BASE_DIR..."
    (cd "$PROJECT_BASE_DIR" && docker build -t hdf5tomcap .)
    if [ $? -ne 0 ]; then echo "Docker build failed." >&2; exit 1; fi
fi

if [ -n "$DEBUG_FLAG" ]; then echo "Running in debug mode..."; fi

# Check if common input files exist on the host
for f_path in "$URDF_FILE_PATH_HOST" "$CONFIG_FILE_PATH_HOST"; do
    if [ ! -f "$f_path" ]; then echo "Error: Input file not found: $f_path" >&2; exit 1; fi
done

echo "Using URDF file: $URDF_FILE_PATH_HOST"
echo "Using configuration file: $CONFIG_FILE_PATH_HOST"

ALL_CONVERSIONS_SUCCESSFUL=true

for CURRENT_HDF5_BASENAME in "${HDF5_FILES_TO_PROCESS[@]}"; do
    echo "" # Add a blank line for readability between file processing

    # Construct full host path for current HDF5 file
    if [ -n "$INPUT_DIR_ARG" ]; then
        HDF5_FILE_PATH_HOST=$(readlink -m "$EFFECTIVE_INPUT_DIR/$CURRENT_HDF5_BASENAME")
    else
        HDF5_FILE_PATH_HOST=$(readlink -m "$PROJECT_BASE_DIR/$CURRENT_HDF5_BASENAME")
    fi

    if [ ! -f "$HDF5_FILE_PATH_HOST" ]; then
        echo "Error: Input HDF5 file not found: $HDF5_FILE_PATH_HOST" >&2
        ALL_CONVERSIONS_SUCCESSFUL=false
        continue
    fi

    # Generate meaningful output filename from template and HDF5 basename
    HDF5_FILE_BASE_NO_EXT="$(basename "${CURRENT_HDF5_BASENAME%.hdf5}")"

    # Simplified output filename - just add the basename before extension
    OUTPUT_MCAP_FILENAME="${OUTPUT_MCAP_TEMPLATE%.mcap}_${HDF5_FILE_BASE_NO_EXT}.mcap"

    # Construct full host output path
    if [ -n "$OUTPUT_DIR_ARG" ]; then
        OUTPUT_FILE_PATH_HOST="$EFFECTIVE_OUTPUT_DIR/$OUTPUT_MCAP_FILENAME"
    else
        OUTPUT_FILE_PATH_HOST="$PROJECT_BASE_DIR/$OUTPUT_MCAP_FILENAME"
    fi

    # Define the full output path inside container - directly in /mnt_out
    CONTAINER_OUTPUT_FILE_PATH="$CONTAINER_OUTPUT_DIR/$(basename "$OUTPUT_FILE_PATH_HOST")"

    # Create output directory
    mkdir -p "$(dirname "$OUTPUT_FILE_PATH_HOST")"

    echo "Processing HDF5 file: $HDF5_FILE_PATH_HOST"
    echo "Target output MCAP: $OUTPUT_FILE_PATH_HOST"

    # Check if output file already exists, report it but don't stop
    if [ -f "$OUTPUT_FILE_PATH_HOST" ]; then
        echo "Warning: Output file already exists and will be overwritten: $OUTPUT_FILE_PATH_HOST"
    fi

    # Set up the Python command to run inside the container
    CONTAINER_INPUT_DIR=""
    if [ -n "$INPUT_DIR_ARG" ]; then
        CONTAINER_INPUT_DIR="--input-dir /mnt_input"
    fi
    
    CUSTOM_HANDLERS_FLAG=""
    if [ -n "$CUSTOM_HANDLERS_ARG" ]; then
        CUSTOM_HANDLERS_FLAG="--custom-handlers \"$CUSTOM_HANDLERS_ARG\""
    fi

    PYTHON_CMD="/script/hdf5_to_mcap_converter.py \
        --hdf5-file $CONTAINER_HDF5_PATH \
        --urdf-file $CONTAINER_URDF_PATH \
        --output-file $CONTAINER_OUTPUT_FILE_PATH \
        --config-file $CONTAINER_CONFIG_PATH \
        $CONTAINER_INPUT_DIR \
        $CUSTOM_HANDLERS_FLAG \
        $DEBUG_FLAG"

    # Run Docker directly with all output captured and displayed
    echo "Running Docker conversion command..."
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "--- Processing $CURRENT_HDF5_BASENAME ---" >> "$LOG_FILE"
        echo "Command: python3 $PYTHON_CMD" >> "$LOG_FILE"

        # Run Docker with output captured to both terminal and log file
        DOCKER_VOLUMES="-v $HDF5_FILE_PATH_HOST:$CONTAINER_HDF5_PATH:ro \
            -v $URDF_FILE_PATH_HOST:$CONTAINER_URDF_PATH:ro \
            -v $CONFIG_FILE_PATH_HOST:$CONTAINER_CONFIG_PATH:ro \
            -v $(dirname "$OUTPUT_FILE_PATH_HOST"):$CONTAINER_OUTPUT_DIR \
            -v $SCRIPT_DIR:/script:ro"
        
        if [ -n "$INPUT_DIR_ARG" ]; then
            DOCKER_VOLUMES="$DOCKER_VOLUMES -v $EFFECTIVE_INPUT_DIR:/mnt_input:ro"
        fi
        
        docker run --rm $DOCKER_VOLUMES hdf5tomcap python3 $PYTHON_CMD 2>&1 | tee -a "$LOG_FILE"
    else
        # Run Docker with output to terminal only
        DOCKER_VOLUMES="-v $HDF5_FILE_PATH_HOST:$CONTAINER_HDF5_PATH:ro \
            -v $URDF_FILE_PATH_HOST:$CONTAINER_URDF_PATH:ro \
            -v $CONFIG_FILE_PATH_HOST:$CONTAINER_CONFIG_PATH:ro \
            -v $(dirname "$OUTPUT_FILE_PATH_HOST"):$CONTAINER_OUTPUT_DIR \
            -v $SCRIPT_DIR:/script:ro"
        
        if [ -n "$INPUT_DIR_ARG" ]; then
            DOCKER_VOLUMES="$DOCKER_VOLUMES -v $EFFECTIVE_INPUT_DIR:/mnt_input:ro"
        fi
        
        docker run --rm $DOCKER_VOLUMES hdf5tomcap python3 $PYTHON_CMD
    fi

    CONVERTER_EXIT_CODE=$?

    # Verify the output file exists and has non-zero size
    echo "Verifying output file..."
    if [ -f "$OUTPUT_FILE_PATH_HOST" ]; then
        FILE_SIZE=$(du -h "$OUTPUT_FILE_PATH_HOST" | cut -f1)
        echo "VERIFICATION SUCCESS: Output file exists with size: $FILE_SIZE"
    else
        echo "VERIFICATION ERROR: Output file doesn't exist: $OUTPUT_FILE_PATH_HOST"

        # Try to create a simple test file in the same location to check permissions
        echo "Testing write permissions by creating a test file..."
        echo "This is a test file from run_converter_standalone.sh" > "$(dirname "$OUTPUT_FILE_PATH_HOST")/test_permissions.txt"
        if [ -f "$(dirname "$OUTPUT_FILE_PATH_HOST")/test_permissions.txt" ]; then
            echo "PERMISSION TEST: Successfully wrote a test file, so permissions are OK."
            echo "The Python script likely encountered issues during the conversion process."
        else
            echo "PERMISSION TEST: Failed to write a test file. There may be permission issues."
        fi

        # If the exit code was 0 but no file exists, something is wrong
        if [ $CONVERTER_EXIT_CODE -eq 0 ]; then
            echo "WARNING: Converter script returned success (0) but no output file was created."
            CONVERTER_EXIT_CODE=1  # Force an error for this case
        fi
    fi

    if [ $CONVERTER_EXIT_CODE -ne 0 ]; then
        echo ""
        echo "--- ERROR: Converter script failed for $CURRENT_HDF5_BASENAME with exit code $CONVERTER_EXIT_CODE ---" >&2
        ALL_CONVERSIONS_SUCCESSFUL=false
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
            echo "--- ERROR: Converter script failed for $CURRENT_HDF5_BASENAME with exit code $CONVERTER_EXIT_CODE ---" >> "$LOG_FILE"
        fi
    else
        echo "Conversion complete for $CURRENT_HDF5_BASENAME. Output at: $OUTPUT_FILE_PATH_HOST"
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
            echo "Conversion complete for $CURRENT_HDF5_BASENAME. Output at: $OUTPUT_FILE_PATH_HOST" >> "$LOG_FILE"
        fi
    fi
done

echo "" # Final blank line for clarity

if $ALL_CONVERSIONS_SUCCESSFUL; then
    echo "All HDF5 to MCAP conversions finished successfully."
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "Full log is at $LOG_FILE"
    fi
    exit 0
else
    echo "--- ERROR: One or more HDF5 to MCAP conversions failed. ---" >&2
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "--- ERROR: One or more HDF5 to MCAP conversions failed. See details above. ---" >> "$LOG_FILE"
        echo "Full log is at $LOG_FILE"
    fi
    exit 1
fi
