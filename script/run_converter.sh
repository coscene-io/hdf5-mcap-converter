#!/bin/bash

# run_converter.sh
# Unified HDF5 ↔ MCAP Conversion Toolkit
# Single entry point for all conversions with automatic execution mode selection

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default execution mode
EXECUTION_MODE="docker"
DIRECTION=""

# Usage information
USAGE_STRING="Usage: $0 <direction> [OPTIONS]

HDF5 ↔ MCAP Conversion Toolkit - Unified Entry Point

DIRECTIONS:
  hdf5-to-mcap    Convert HDF5 files to MCAP format
  mcap-to-hdf5    Convert MCAP files to HDF5 format

EXECUTION MODES:
  Default: Docker execution (for workflow integration)
  --standalone: Local Docker execution (for development)

EXAMPLES:
  # HDF5 → MCAP conversion (Docker mode)
  $0 hdf5-to-mcap -c config.yaml -f data.hdf5 -u robot.urdf -m output.mcap

  # MCAP → HDF5 conversion (Docker mode)
  $0 mcap-to-hdf5 -m input.mcap -c config.yaml -o output.hdf5

  # HDF5 → MCAP conversion (Standalone mode)
  $0 hdf5-to-mcap --standalone -c config.yaml -f data.hdf5 -u robot.urdf -m output.mcap

  # MCAP → HDF5 conversion (Standalone mode)
  $0 mcap-to-hdf5 --standalone -m input.mcap -c config.yaml -o output.hdf5

  # Show help for specific direction
  $0 hdf5-to-mcap --help
  $0 mcap-to-hdf5 --help

EXECUTION MODES:
  Docker (default):    Runs inside pre-orchestrated containers (workflow integration)
  Standalone:          Builds and runs Docker containers locally (development)
"

# Parse arguments to extract direction and execution mode
ARGS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        hdf5-to-mcap|mcap-to-hdf5)
            if [ -n "$DIRECTION" ]; then
                echo "Error: Multiple directions specified" >&2
                exit 1
            fi
            DIRECTION="$1"
            shift
            ;;
        --standalone)
            EXECUTION_MODE="standalone"
            shift
            ;;
        -h|--help)
            if [ -n "$DIRECTION" ]; then
                # Direction-specific help will be handled by the target script
                ARGS+=("$1")
                shift
            else
                # General help
                echo -e "$USAGE_STRING"
                exit 0
            fi
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done

# Check if direction is provided
if [ -z "$DIRECTION" ]; then
    echo "Error: No conversion direction specified." >&2
    echo -e "$USAGE_STRING"
    exit 1
fi

# Dispatch to appropriate converter based on direction and execution mode
case "$DIRECTION" in
    hdf5-to-mcap)
        if [ "$EXECUTION_MODE" = "standalone" ]; then
            echo "🔄 HDF5 → MCAP conversion (standalone mode)..."
            exec "$SCRIPT_DIR/run_hdf5_to_mcap_converter_standalone.sh" "${ARGS[@]}"
        else
            echo "🔄 HDF5 → MCAP conversion (docker mode)..."
            exec "$SCRIPT_DIR/run_hdf5_to_mcap_converter.sh" "${ARGS[@]}"
        fi
        ;;
    mcap-to-hdf5)
        if [ "$EXECUTION_MODE" = "standalone" ]; then
            echo "🔄 MCAP → HDF5 conversion (standalone mode)..."
            exec "$SCRIPT_DIR/run_mcap_to_hdf5_converter_standalone.sh" "${ARGS[@]}"
        else
            echo "🔄 MCAP → HDF5 conversion (docker mode)..."
            exec "$SCRIPT_DIR/run_mcap_to_hdf5_converter.sh" "${ARGS[@]}"
        fi
        ;;
    *)
        echo "Error: Unknown conversion direction '$DIRECTION'" >&2
        echo "Valid directions: hdf5-to-mcap, mcap-to-hdf5" >&2
        echo -e "$USAGE_STRING"
        exit 1
        ;;
esac 