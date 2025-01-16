#!/bin/bash

# Function to print usage
print_usage() {
    echo "Usage: $0 <model_path> [model_shape]"
    echo "  <model_path>: Path to your .pt model file (required)"
    echo "  [model_shape]: Shape of the input tensors (optional)"
    echo "                 For single input: '[1,615]'"
    echo "                 For multiple inputs: '[[1,615],[1,615]]'"
    echo "                 Default is '[1,615]' if not specified"
}

# Check if at least one argument is provided
if [ $# -lt 1 ]; then
    echo "Error: Model path is required."
    print_usage
    exit 1
fi

# Set model path and shape
MODEL_PATH="$1"
MODEL_SHAPE="${2:-[1,615]}"

# Extract directory, filename, and name without extension
MODEL_DIR=$(dirname "$MODEL_PATH")
MODEL_FILENAME=$(basename "$MODEL_PATH")
MODEL_NAME="${MODEL_FILENAME%.*}"
ARTIFACTS_DIR="${MODEL_DIR}/${MODEL_NAME}_artifacts"

# Create artifacts directory and copy the original file
mkdir -p "$ARTIFACTS_DIR"
cp "$MODEL_PATH" "$ARTIFACTS_DIR/"

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Run the Docker container
docker run --platform linux/amd64 --rm --privileged --name DuoTPU \
    -v "$ARTIFACTS_DIR":/workspace \
    -v "$SCRIPT_DIR/tpu-mlir":/tpu-mlir \
    -it sophgo/tpuc_dev:v3.1 /bin/bash -c "
    # Source the environment setup script
    source /tpu-mlir/envsetup.sh

    # Change to the workspace directory
    cd /workspace

    # Step 1: Convert to .mlir
    model_transform.py --model_name $MODEL_NAME --model_def $MODEL_FILENAME --input_shapes $MODEL_SHAPE --mlir $MODEL_NAME.mlir

    # Step 2: Convert to .cvimodel
    model_deploy.py --mlir $MODEL_NAME.mlir --quantize BF16 --chip cv181x --model $MODEL_NAME.cvimodel

    # Set permissions so the host user can access the generated files
    chmod 666 $MODEL_NAME.mlir $MODEL_NAME.cvimodel
"

echo "Conversion complete. Output files are in $ARTIFACTS_DIR"
