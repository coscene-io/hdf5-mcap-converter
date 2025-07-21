#!/bin/bash

# run_mcap_to_hdf5_converter_standalone.sh
# Standalone script for MCAP to HDF5 conversion using Docker
# This script builds and runs the Docker container with proper volume mounts

set -e

# Default values
PROJECT_BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INPUT_DIR_ARG=""
OUTPUT_DIR_ARG=""
MCAP_INPUT_SPEC="" # Changed from MCAP_FILE to support glob/comma-separated specs
CONFIG_FILE=""
URDF_FILE=""
OUTPUT_HDF5_TEMPLATE="" # Changed from OUTPUT_HDF5_FILE to support template naming
CUSTOM_HANDLERS_ARG=""
DEBUG_FLAG=""
BUILD_FLAG=""
LOG_TO_FILE_FLAG=false
MCAP_FILES_TO_PROCESS=() # Array to store actual MCAP filenames

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

# Usage information
USAGE_STRING="Usage: $0 [OPTIONS]
Convert MCAP files to HDF5 format using Docker.

Required Arguments:
  -m, --mcap-file <filespec>    Input MCAP file(s). Can be a single filename, comma-separated list,
                                or glob pattern (e.g., \"*.mcap\")
  -c, --config <file>           Configuration YAML file
  -u, --urdf <path_or_url>      Robot URDF file. Can be a file path or an http(s) URL. If a URL, it is downloaded temporarily.
  -f, --hdf5-file <template>    Output HDF5 filename template (e.g., \"output.hdf5\" or \"output\").
                                For MCAP \"input.mcap\", will generate \"output_input.hdf5\".

Optional Arguments:
  -i, --input-dir <path>        Base directory for input files (default: project directory)
  -o, --output-dir <path>       Base directory for output files (default: project/output)
  --custom-handlers <handlers>  Custom handler scripts (comma-separated list or glob pattern,
                                e.g., \"handler1.py,handler2.py\" or \"*.py\")
  --debug                       Enable debug output
  --build                       Force rebuild of Docker image
  --log-to-file                 Enable logging to file
  -h, --help                    Show this help message

Examples:
  # Basic conversion
  $0 -m output.mcap -c config_aloha_unified.yaml -u dual_vx300s.urdf -f converted.hdf5

  # Multiple files with glob
  $0 -m \"*.mcap\" -c config.yaml -u robot.urdf -f result.hdf5

  # Comma-separated files
  $0 -m \"file1.mcap,file2.mcap\" -c config.yaml -u robot.urdf -f output.hdf5

  # With custom directories
  $0 -i /path/to/inputs -o /path/to/outputs -m data.mcap -c config.yaml -u robot.urdf -f result.hdf5

  # Using URDF from URL
  $0 -m output.mcap -c config.yaml -u https://example.com/robot.urdf -f converted.hdf5
"

# Parse command line arguments
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
    -m|--mcap-file)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      MCAP_INPUT_SPEC="$2"
      shift; shift
      ;;
    -c|--config)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      CONFIG_FILE="$2"
      shift; shift
      ;;
    -u|--urdf)
      if [ -z "$2" ] || [[ "$2" == -* ]]; then echo "Error: Missing value for $1"; echo -e "$USAGE_STRING"; exit 1; fi
      URDF_FILE="$2"
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
    --debug)
      DEBUG_FLAG="--debug"
      shift
      ;;
    --build)
      BUILD_FLAG="--build"
      shift
      ;;
    --log-to-file)
      LOG_TO_FILE_FLAG=true
      shift
      ;;
    -h|--help)
      echo -e "$USAGE_STRING"
      exit 0
      ;;
    *)
      echo "Error: Unknown option: $1" >&2
      echo -e "$USAGE_STRING"
      exit 1
      ;;
  esac
done

# Check required arguments
if [ -z "$MCAP_INPUT_SPEC" ] || [ -z "$CONFIG_FILE" ] || [ -z "$URDF_FILE" ] || [ -z "$OUTPUT_HDF5_TEMPLATE" ]; then
    echo "Error: Missing required arguments (-m, -c, -u, -f)." >&2
    echo -e "$USAGE_STRING"
    exit 1
fi

# Determine effective input and output directories
if [ -n "$INPUT_DIR_ARG" ]; then
    EFFECTIVE_INPUT_DIR="$(readlink -m "$INPUT_DIR_ARG")"
else
    EFFECTIVE_INPUT_DIR="$PROJECT_BASE_DIR"
fi

if [ -n "$OUTPUT_DIR_ARG" ]; then
    EFFECTIVE_OUTPUT_DIR="$(readlink -m "$OUTPUT_DIR_ARG")"
else
    EFFECTIVE_OUTPUT_DIR="$PROJECT_BASE_DIR/output"
fi

echo "Effective Input Directory: $EFFECTIVE_INPUT_DIR"
echo "Effective Output Directory: $EFFECTIVE_OUTPUT_DIR"

# Create output directory if it doesn't exist
mkdir -p "$EFFECTIVE_OUTPUT_DIR"

# Construct full paths
CONFIG_FILE_PATH="$EFFECTIVE_INPUT_DIR/config/$CONFIG_FILE"

# --- URDF File Path Resolution ---
# URDF_FILE is the raw input from -u argument
ACTUAL_URDF_FILE_PATH="" # This will hold the final path for URDF

if [[ "$URDF_FILE" == http://* || "$URDF_FILE" == https://* ]]; then
    echo "URDF argument is a URL: $URDF_FILE"
    # Create a unique temporary directory for this run using system default
    TEMP_URDF_DOWNLOAD_DIR=$(mktemp -d)
    if [ ! -d "$TEMP_URDF_DOWNLOAD_DIR" ]; then
        echo "Error: Could not create system temporary directory for URDF download." >&2
        exit 1
    fi
    echo "Created temporary URDF download directory: $TEMP_URDF_DOWNLOAD_DIR"

    # Handle cases where basename might be empty or problematic (e.g. if URL ends with / or has query params)
    # A simple approach: remove query string, then basename
    URL_NO_QUERY="${URDF_FILE%%\?*}"
    DOWNLOADED_URDF_BASENAME=$(basename "$URL_NO_QUERY")

    if [ -z "$DOWNLOADED_URDF_BASENAME" ] || [[ "$DOWNLOADED_URDF_BASENAME" == "." ]] || [[ "$DOWNLOADED_URDF_BASENAME" == "/" ]]; then
        DOWNLOADED_URDF_BASENAME="downloaded.urdf"
    fi
    ACTUAL_URDF_FILE_PATH="$TEMP_URDF_DOWNLOAD_DIR/$DOWNLOADED_URDF_BASENAME"

    echo "Attempting to download URDF to: $ACTUAL_URDF_FILE_PATH using Python"
    # Use Python to download
    PYTHON_DOWNLOAD_SCRIPT='import sys, urllib.request; url, dest_path = sys.argv[1], sys.argv[2];
try:
    print(f"Python: Downloading {url} to {dest_path}", file=sys.stderr)
    urllib.request.urlretrieve(url, dest_path)
    print(f"Python: Successfully downloaded {url} to {dest_path}", file=sys.stderr)
except Exception as e:
    print(f"Python: Error downloading {url} - {e}", file=sys.stderr)
    sys.exit(1)'

    if python3 -c "$PYTHON_DOWNLOAD_SCRIPT" "$URDF_FILE" "$ACTUAL_URDF_FILE_PATH"; then
        echo "URDF downloaded successfully from URL."
        URDF_FILE_PATH="$ACTUAL_URDF_FILE_PATH"
    else
        python_exit_code=$?
        echo "Error: Failed to download URDF from URL using Python: $URDF_FILE (Python exit code: $python_exit_code)" >&2
        # The trap will attempt cleanup
        exit 1
    fi
else # URDF_FILE is a local file path
    echo "URDF argument is a local file path: $URDF_FILE"
    URDF_FILE_PATH="$EFFECTIVE_INPUT_DIR/urdf/$URDF_FILE"
fi
# --- End URDF File Path Resolution ---

# Process MCAP_INPUT_SPEC to populate MCAP_FILES_TO_PROCESS
if [[ "$MCAP_INPUT_SPEC" == *","* ]]; then
    IFS=',' read -r -a MCAP_FILES_TO_PROCESS <<< "$MCAP_INPUT_SPEC"
else
    # Handle glob or single file
    ORIGINAL_PWD=$(pwd)
    if [ -d "$EFFECTIVE_INPUT_DIR" ]; then
        cd "$EFFECTIVE_INPUT_DIR"
        # Use a loop to correctly handle filenames with spaces if glob expands to them
        for f_glob in $MCAP_INPUT_SPEC; do
            if [ -f "$f_glob" ]; then # Check if glob match is an actual file
                MCAP_FILES_TO_PROCESS+=("$f_glob")
            fi
        done
        cd "$ORIGINAL_PWD"
    else
        echo "Error: Input directory '$EFFECTIVE_INPUT_DIR' not found." >&2
        exit 1
    fi

    # If after globbing, MCAP_FILES_TO_PROCESS is empty, it might be a single file not caught by glob
    if [ ${#MCAP_FILES_TO_PROCESS[@]} -eq 0 ] && [[ "$MCAP_INPUT_SPEC" != *"*"* ]] && [[ "$MCAP_INPUT_SPEC" != *"?"* ]]; then
        # Check if this single file exists relative to input_dir
        if [ -n "$INPUT_DIR_ARG" ]; then
            if [ -f "$EFFECTIVE_INPUT_DIR/$MCAP_INPUT_SPEC" ]; then
                MCAP_FILES_TO_PROCESS+=("$MCAP_INPUT_SPEC")
            fi
        else
            if [ -f "$PROJECT_BASE_DIR/$MCAP_INPUT_SPEC" ]; then
                MCAP_FILES_TO_PROCESS+=("$MCAP_INPUT_SPEC")
            fi
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

if [ ! -f "$CONFIG_FILE_PATH" ]; then
    echo "Error: Config file not found: $CONFIG_FILE_PATH" >&2
    exit 1
fi

# URDF file existence is already checked in the URL resolution section above
if [ ! -f "$URDF_FILE_PATH" ]; then
    echo "Error: URDF file not found: $URDF_FILE_PATH" >&2
    exit 1
fi

echo "Using configuration file: $CONFIG_FILE_PATH"
echo "Using URDF file: $URDF_FILE_PATH"

# Docker image name
DOCKER_IMAGE_NAME="hdf5tomcap:latest"

# Check if Docker is available first
USE_DOCKER=true
if ! docker ps >/dev/null 2>&1; then
    echo "⚠️  Docker is not available or not running. Using local execution..."
    USE_DOCKER=false
fi

# Build Docker image if needed and Docker is available
if [ "$USE_DOCKER" = "true" ]; then
    if [ -n "$BUILD_FLAG" ] || ! docker image inspect "$DOCKER_IMAGE_NAME" >/dev/null 2>&1; then
        echo "Building Docker image from $PROJECT_BASE_DIR..."
        if ! docker build -t "$DOCKER_IMAGE_NAME" "$PROJECT_BASE_DIR"; then
            echo "⚠️  Docker build failed. Falling back to local execution..."
            USE_DOCKER=false
        fi
    fi
fi

# Prepare log file if requested
LOG_FILE=""
if $LOG_TO_FILE_FLAG; then
    if [ "$USE_DOCKER" = "true" ]; then
        LOG_FILE="$EFFECTIVE_OUTPUT_DIR/mcap_to_hdf5_conversion.log"
    else
        LOG_FILE="$EFFECTIVE_OUTPUT_DIR/mcap_to_hdf5_conversion_local.log"
    fi
    echo "Logging to: $LOG_FILE"
    
    # Create log file with header
    echo "--- MCAP to HDF5 Conversion Log ---" > "$LOG_FILE"
    echo "Started at: $(date)" >> "$LOG_FILE"
    echo "Processing files: ${MCAP_FILES_TO_PROCESS[@]}" >> "$LOG_FILE"
    echo "" >> "$LOG_FILE"
fi

# Check if common input files exist on the host
for f_path in "$URDF_FILE_PATH" "$CONFIG_FILE_PATH"; do
    if [ ! -f "$f_path" ]; then echo "Error: Input file not found: $f_path" >&2; exit 1; fi
done

if [ -n "$DEBUG_FLAG" ]; then echo "Running in debug mode..."; fi

# Container paths
CONTAINER_INPUT_DIR="/mnt_input"
CONTAINER_OUTPUT_DIR="/mnt_output"
CONTAINER_MCAP_FILE="/mnt/data.mcap"
CONTAINER_URDF_FILE="/mnt/robot.urdf"
CONTAINER_CONFIG_FILE="/mnt/config.yaml"

ALL_CONVERSIONS_SUCCESSFUL=true

for CURRENT_MCAP_BASENAME in "${MCAP_FILES_TO_PROCESS[@]}"; do
    echo "" # Add a blank line for readability between file processing

    # Construct full host path for current MCAP file
    if [ -n "$INPUT_DIR_ARG" ]; then
        MCAP_FILE_PATH_HOST=$(readlink -m "$EFFECTIVE_INPUT_DIR/$CURRENT_MCAP_BASENAME")
    else
        MCAP_FILE_PATH_HOST=$(readlink -m "$PROJECT_BASE_DIR/$CURRENT_MCAP_BASENAME")
    fi

    if [ ! -f "$MCAP_FILE_PATH_HOST" ]; then
        echo "Error: Input MCAP file not found: $MCAP_FILE_PATH_HOST" >&2
        ALL_CONVERSIONS_SUCCESSFUL=false
        continue
    fi

    # Generate meaningful output filename from template and MCAP basename
    MCAP_FILE_BASE_NO_EXT="$(basename "${CURRENT_MCAP_BASENAME%.mcap}")"

    # Simplified output filename - just add the basename before extension
    OUTPUT_HDF5_FILENAME="${OUTPUT_HDF5_TEMPLATE%.hdf5}_${MCAP_FILE_BASE_NO_EXT}.hdf5"

    # Construct full host output path
    if [ -n "$OUTPUT_DIR_ARG" ]; then
        OUTPUT_FILE_PATH_HOST="$EFFECTIVE_OUTPUT_DIR/$OUTPUT_HDF5_FILENAME"
    else
        OUTPUT_FILE_PATH_HOST="$PROJECT_BASE_DIR/output/$OUTPUT_HDF5_FILENAME"
    fi

    # Define the full output path inside container
    CONTAINER_OUTPUT_FILE_PATH="$CONTAINER_OUTPUT_DIR/$(basename "$OUTPUT_FILE_PATH_HOST")"

    # Create output directory
    mkdir -p "$(dirname "$OUTPUT_FILE_PATH_HOST")"

    echo "Processing MCAP file: $MCAP_FILE_PATH_HOST"
    echo "Target output HDF5: $OUTPUT_FILE_PATH_HOST"

    # Check if output file already exists, report it but don't stop
    if [ -f "$OUTPUT_FILE_PATH_HOST" ]; then
        echo "Warning: Output file already exists and will be overwritten: $OUTPUT_FILE_PATH_HOST"
    fi

    # Set up the Python command to run inside the container
    CONTAINER_INPUT_DIR_ARG=""
    if [ -n "$INPUT_DIR_ARG" ]; then
        CONTAINER_INPUT_DIR_ARG="--input-dir $CONTAINER_INPUT_DIR"
    fi
    
    CUSTOM_HANDLERS_FLAG=""
    if [ -n "$CUSTOM_HANDLERS_ARG" ]; then
        CUSTOM_HANDLERS_FLAG="--custom-handlers \"$CUSTOM_HANDLERS_ARG\""
    fi

    if [ "$USE_DOCKER" = "true" ]; then
        PYTHON_CMD="python3 /script/mcap_to_hdf5_converter.py \
            --mcap-file $CONTAINER_MCAP_FILE \
            --hdf5-file $CONTAINER_OUTPUT_FILE_PATH \
            --config-file $CONTAINER_CONFIG_FILE \
            --urdf-file $CONTAINER_URDF_FILE \
            $CONTAINER_INPUT_DIR_ARG \
            --output-dir $CONTAINER_OUTPUT_DIR \
            $CUSTOM_HANDLERS_FLAG \
            $DEBUG_FLAG"

        echo "Running Docker conversion command..."
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
            echo "--- Processing $CURRENT_MCAP_BASENAME ---" >> "$LOG_FILE"
            echo "Command: $PYTHON_CMD" >> "$LOG_FILE"

            # Run Docker with output captured to both terminal and log file
            DOCKER_VOLUMES="-v $MCAP_FILE_PATH_HOST:$CONTAINER_MCAP_FILE:ro \
                -v $URDF_FILE_PATH:$CONTAINER_URDF_FILE:ro \
                -v $CONFIG_FILE_PATH:$CONTAINER_CONFIG_FILE:ro \
                -v $(dirname "$OUTPUT_FILE_PATH_HOST"):$CONTAINER_OUTPUT_DIR"
            
            if [ -n "$INPUT_DIR_ARG" ]; then
                DOCKER_VOLUMES="$DOCKER_VOLUMES -v $EFFECTIVE_INPUT_DIR:$CONTAINER_INPUT_DIR:ro"
            fi
            
            docker run --rm $DOCKER_VOLUMES "$DOCKER_IMAGE_NAME" $PYTHON_CMD 2>&1 | tee -a "$LOG_FILE"
        else
            # Run Docker with output to terminal only
            DOCKER_VOLUMES="-v $MCAP_FILE_PATH_HOST:$CONTAINER_MCAP_FILE:ro \
                -v $URDF_FILE_PATH:$CONTAINER_URDF_FILE:ro \
                -v $CONFIG_FILE_PATH:$CONTAINER_CONFIG_FILE:ro \
                -v $(dirname "$OUTPUT_FILE_PATH_HOST"):$CONTAINER_OUTPUT_DIR"
            
            if [ -n "$INPUT_DIR_ARG" ]; then
                DOCKER_VOLUMES="$DOCKER_VOLUMES -v $EFFECTIVE_INPUT_DIR:$CONTAINER_INPUT_DIR:ro"
            fi
            
            docker run --rm $DOCKER_VOLUMES "$DOCKER_IMAGE_NAME" $PYTHON_CMD
        fi
    else
        # Local execution
        LOCAL_INPUT_DIR_ARG="--input-dir \"$EFFECTIVE_INPUT_DIR\""
        
        LOCAL_CUSTOM_HANDLERS_FLAG=""
        if [ -n "$CUSTOM_HANDLERS_ARG" ]; then
            LOCAL_CUSTOM_HANDLERS_FLAG="--custom-handlers \"$CUSTOM_HANDLERS_ARG\""
        fi

        LOCAL_PYTHON_COMMAND="python3 \"$PROJECT_BASE_DIR/script/mcap_to_hdf5_converter.py\" \
            --mcap-file \"$MCAP_FILE_PATH_HOST\" \
            --hdf5-file \"$OUTPUT_FILE_PATH_HOST\" \
            --config-file \"$CONFIG_FILE_PATH\" \
            --urdf-file \"$URDF_FILE_PATH\" \
            $LOCAL_INPUT_DIR_ARG \
            --output-dir \"$EFFECTIVE_OUTPUT_DIR\" \
            $LOCAL_CUSTOM_HANDLERS_FLAG \
            $DEBUG_FLAG"
        
        echo "Running local conversion command (Docker not available)..."
        
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
            echo "--- Processing $CURRENT_MCAP_BASENAME (Local) ---" >> "$LOG_FILE"
            echo "Command: $LOCAL_PYTHON_COMMAND" >> "$LOG_FILE"
            eval "$LOCAL_PYTHON_COMMAND" >> "$LOG_FILE" 2>&1
        else
            eval "$LOCAL_PYTHON_COMMAND"
        fi
    fi

    CONVERTER_EXIT_CODE=$?

    # Verify the output file exists and has non-zero size
    echo "Verifying output file..."
    if [ -f "$OUTPUT_FILE_PATH_HOST" ]; then
        FILE_SIZE=$(du -h "$OUTPUT_FILE_PATH_HOST" | cut -f1)
        echo "VERIFICATION SUCCESS: Output file exists with size: $FILE_SIZE"
    else
        echo "VERIFICATION ERROR: Output file doesn't exist: $OUTPUT_FILE_PATH_HOST"

        # If the exit code was 0 but no file exists, something is wrong
        if [ $CONVERTER_EXIT_CODE -eq 0 ]; then
            echo "WARNING: Converter script returned success (0) but no output file was created."
            CONVERTER_EXIT_CODE=1  # Force an error for this case
        fi
    fi

    if [ $CONVERTER_EXIT_CODE -ne 0 ]; then
        echo ""
        echo "--- ERROR: Converter script failed for $CURRENT_MCAP_BASENAME with exit code $CONVERTER_EXIT_CODE ---" >&2
        ALL_CONVERSIONS_SUCCESSFUL=false
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
            echo "--- ERROR: Converter script failed for $CURRENT_MCAP_BASENAME with exit code $CONVERTER_EXIT_CODE ---" >> "$LOG_FILE"
        fi
    else
        echo "Conversion complete for $CURRENT_MCAP_BASENAME. Output at: $OUTPUT_FILE_PATH_HOST"
        if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
            echo "Conversion complete for $CURRENT_MCAP_BASENAME. Output at: $OUTPUT_FILE_PATH_HOST" >> "$LOG_FILE"
        fi
    fi
done

echo "" # Final blank line for clarity

if $ALL_CONVERSIONS_SUCCESSFUL; then
    echo "✅ All MCAP to HDF5 conversions finished successfully!"
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "Full log is at $LOG_FILE"
    fi
    exit 0
else
    echo "❌ One or more MCAP to HDF5 conversions failed." >&2
    if $LOG_TO_FILE_FLAG && [ -n "$LOG_FILE" ]; then
        echo "--- ERROR: One or more MCAP to HDF5 conversions failed. See details above. ---" >> "$LOG_FILE"
        echo "Full log is at $LOG_FILE"
    fi
    exit 1
fi