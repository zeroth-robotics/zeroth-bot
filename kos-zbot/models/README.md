# TPU-MLIR Model Conversion Tool

This tool converts PyTorch (.pt) models to the format compatible with Milk-V Duo(S).

## Prerequisites

- Docker installed and running on your system
- The `tpu-mlir` directory in the same location as this README and the `convert_model.sh` script

## Usage

The basic syntax for using the conversion tool is:

```bash
MODEL_PATH=/path/to/your/model.pt [MODEL_SHAPE='1,615'] ./convert_model.sh [model_shape]
```

- `MODEL_PATH`: Path to your .pt model file (required)
- `MODEL_SHAPE`: Shape of the input tensor (optional, can be set as env var or command-line arg)
                 Default is '1,615' if not specified

## Example: Converting 'standing.pt'

Let's say you have a model named 'standing.pt' in the same directory as the `convert_model.sh` script. Here's how you would convert it:

1. **Using default shape (1,615)**:

   ```bash
   MODEL_PATH=./standing.pt ./convert_model.sh
   ```

2. **Specifying a different shape via environment variable**:

   ```bash
   MODEL_PATH=./standing.pt MODEL_SHAPE="1,512" ./convert_model.sh
   ```

3. **Specifying shape as a command-line argument**:

   ```bash
   MODEL_PATH=./standing.pt ./convert_model.sh "1,1024"
   ```

## Output

After running the script, you'll find a new directory named `standing_artifacts` in the same location as your 'standing.pt' file. This directory will contain:

- The original `standing.pt` file
- `standing.mlir`: The MLIR representation of your model
- `standing.cvimodel`: The CVIMODEL file compatible with Milk-V Duo

## Troubleshooting

If you encounter any issues:

1. Ensure Docker is running and you have the necessary permissions.
2. Check that the `tpu-mlir` directory is present (submodule initialized) and contains the required files.
3. Verify that the input shape matches your model's expected input.

## Additional Notes

- The conversion process uses the Docker image `sophgo/tpuc_dev:v3.1`.
- If you need to modify the quantization method or target chip, you can edit the `convert_model.sh` script.

For more detailed information about the conversion process or advanced usage, please refer to the TPU-MLIR documentation.
