#include <cviruntime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

CVI_MODEL_HANDLE model = NULL;
CVI_TENSOR *inputs = NULL;
CVI_TENSOR *outputs = NULL;
int32_t input_num = 0;
int32_t output_num = 0;

int init_model(const char* model_path) {
    CVI_RC rc = CVI_NN_RegisterModel(model_path, &model);
    if (rc != 0) {
        return -1;
    }
    rc = CVI_NN_GetInputOutputTensors(model, &inputs, &input_num, &outputs, &output_num);
    if (rc != 0) {
        return -1;
    }
    return 0;
}

int forward(float* input_data, float* output_data) {
    if (!model || !inputs || !outputs) {
        return -1;
    }
    
    memcpy(CVI_NN_TensorPtr(&inputs[0]), input_data, CVI_NN_TensorSize(&inputs[0]));
    
    CVI_RC rc = CVI_NN_Forward(model, inputs, input_num, outputs, output_num);
    if (rc != 0) {
        return -1;
    }
    
    memcpy(output_data, CVI_NN_TensorPtr(&outputs[0]), CVI_NN_TensorSize(&outputs[0]));
    
    return 0;
}

void cleanup() {
    if (model) {
        CVI_NN_CleanupModel(model);
        model = NULL;
    }
}

size_t get_input_size() {
    return inputs ? CVI_NN_TensorSize(&inputs[0]) : 0;
}

size_t get_output_size() {
    return outputs ? CVI_NN_TensorSize(&outputs[0]) : 0;
}
