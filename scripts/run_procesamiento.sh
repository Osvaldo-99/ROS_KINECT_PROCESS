```bash
#!/bin/bash

# ============================================================
# 3D POINT CLOUD PROCESSING EXECUTION SCRIPT
# Repository: ROS_KINECT_PROCESS
# ============================================================

set -e

echo "========================================"
echo "3D POINT CLOUD PROCESSING"
echo "========================================"

# Move to repository root directory
# even if the script is executed from another location
cd "$(dirname "$0")/.."

# Activate virtual environment if available
if [ -d ".venv" ]; then
    echo "Activating virtual environment..."
    source .venv/bin/activate
fi

# Input point cloud file
INPUT_PLY="data/input/original_point_cloud.ply"

# Output directory
OUTPUT_DIR="results/experiment_01"

# Create output directory if it does not exist
mkdir -p "$OUTPUT_DIR"

# Execute processing pipeline
python3 src/process_point_cloud.py \
    --input "$INPUT_PLY" \
    --output-dir "$OUTPUT_DIR" \
    --target-points 100000 \
    --tolerance 10000 \
    --voxel-start 0.015 \
    --voxel-max 0.05 \
    --voxel-step 0.001 \
    --scale-cloud 1.0 \
    --random-state 42 \
    --export-dae

echo "========================================"
echo "PROCESS COMPLETED"
echo "Results saved to: $OUTPUT_DIR"
echo "========================================"
```
