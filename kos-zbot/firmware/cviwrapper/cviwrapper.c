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

int forward(float* input_data[], int input_count, float* output_data[], int output_count) {
    if (!model || !inputs || !outputs || 
        input_count != input_num || output_count != output_num) {
        return -1;
    }
    
    // Copy all input data
    for (int i = 0; i < input_count; i++) {
        memcpy(CVI_NN_TensorPtr(&inputs[i]), 
               input_data[i], 
               CVI_NN_TensorSize(&inputs[i]));
    }
    
    CVI_RC rc = CVI_NN_Forward(model, inputs, input_num, outputs, output_num);
    if (rc != 0) {
        return -1;
    }
    
    // Copy all output data
    for (int i = 0; i < output_count; i++) {
        memcpy(output_data[i], 
               CVI_NN_TensorPtr(&outputs[i]), 
               CVI_NN_TensorSize(&outputs[i]));
    }
    
    return 0;
}

void cleanup() {
    if (model) {
        CVI_NN_CleanupModel(model);
        model = NULL;
    }
}

int get_input_count() {
    return input_num;
}

int get_output_count() {
    return output_num;
}

size_t get_input_size_at(int index) {
    if (index < 0 || index >= input_num || !inputs) {
        return 0;
    }
    return CVI_NN_TensorSize(&inputs[index]);
}

size_t get_output_size_at(int index) {
    if (index < 0 || index >= output_num || !outputs) {
        return 0;
    }
    return CVI_NN_TensorSize(&outputs[index]);
}

const char* get_input_name_at(int index) {
    if (index < 0 || index >= input_num || !inputs) {
        return NULL;
    }
    return CVI_NN_TensorName(&inputs[index]);
}

const char* get_output_name_at(int index) {
    if (index < 0 || index >= output_num || !outputs) {
        return NULL;
    }
    return CVI_NN_TensorName(&outputs[index]);
}

// Get shape dimensions and size
int get_input_shape_at(int index, int32_t* dims, size_t* dim_count) {
    if (index < 0 || index >= input_num || !inputs || !dims || !dim_count) {
        return -1;
    }
    CVI_SHAPE shape = CVI_NN_TensorShape(&inputs[index]);
    *dim_count = shape.dim_size;
    for (size_t i = 0; i < shape.dim_size; i++) {
        dims[i] = shape.dim[i];
    }
    return 0;
}

int get_output_shape_at(int index, int32_t* dims, size_t* dim_count) {
    if (index < 0 || index >= output_num || !outputs || !dims || !dim_count) {
        return -1;
    }
    CVI_SHAPE shape = CVI_NN_TensorShape(&outputs[index]);
    *dim_count = shape.dim_size;
    for (size_t i = 0; i < shape.dim_size; i++) {
        dims[i] = shape.dim[i];
    }
    return 0;
}

// Find tensor index by name
int find_input_index(const char* name) {
    if (!name || !inputs) {
        return -1;
    }
    for (int i = 0; i < input_num; i++) {
        const char* tensor_name = CVI_NN_TensorName(&inputs[i]);
        if (tensor_name && strcmp(tensor_name, name) == 0) {
            return i;
        }
    }
    return -1;
}

int find_output_index(const char* name) {
    if (!name || !outputs) {
        return -1;
    }
    for (int i = 0; i < output_num; i++) {
        const char* tensor_name = CVI_NN_TensorName(&outputs[i]);
        if (tensor_name && strcmp(tensor_name, name) == 0) {
            return i;
        }
    }
    return -1;
}

// Get size by name
size_t get_input_size_by_name(const char* name) {
    int index = find_input_index(name);
    if (index < 0) {
        return 0;
    }
    return get_input_size_at(index);
}

size_t get_output_size_by_name(const char* name) {
    int index = find_output_index(name);
    if (index < 0) {
        return 0;
    }
    return get_output_size_at(index);
}
