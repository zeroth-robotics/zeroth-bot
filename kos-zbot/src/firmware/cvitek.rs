use eyre::Result;
use lazy_static::lazy_static;
use std::collections::HashMap;
use std::ffi::CString;
use std::os::raw::{c_char, c_float, c_int};
use std::path::Path;
use std::sync::Mutex;

#[link(name = "cviwrapper")]
extern "C" {
    fn init_model(model_path: *const c_char) -> c_int;
    fn forward(
        input_data: *const *const c_float,
        input_count: c_int,
        output_data: *const *mut c_float,
        output_count: c_int,
    ) -> c_int;
    fn cleanup();

    // Input/output information functions
    fn get_input_count() -> c_int;
    fn get_output_count() -> c_int;
    fn get_input_name_at(index: c_int) -> *const c_char;
    fn get_output_name_at(index: c_int) -> *const c_char;
    fn get_input_size_at(index: c_int) -> usize;
    fn get_output_size_at(index: c_int) -> usize;

    fn get_input_shape_at(index: c_int, dims: *mut i32, dim_count: *mut usize) -> c_int;
    fn get_output_shape_at(index: c_int, dims: *mut i32, dim_count: *mut usize) -> c_int;
}

pub struct Model {
    _private: (), // Prevent direct construction
}

#[derive(Debug)]
pub struct TensorInfo {
    pub name: String,
    pub shape: Vec<i32>,
    pub size: usize,
}

impl Model {
    pub fn new<P: AsRef<Path>>(model_path: P) -> Result<Self> {
        let c_model_path = CString::new(model_path.as_ref().to_str().unwrap())?;
        let result = unsafe { init_model(c_model_path.as_ptr()) };
        if result != 0 {
            eyre::bail!("Failed to initialize MilkV model");
        }
        Ok(Model { _private: () })
    }

    pub fn infer(&self, inputs: HashMap<String, Vec<f32>>) -> Result<HashMap<String, Vec<f32>>> {
        let input_count = unsafe { get_input_count() } as usize;
        let output_count = unsafe { get_output_count() } as usize;

        // Create arrays to hold input pointers and data
        let mut input_ptrs: Vec<*const c_float> = Vec::with_capacity(input_count);
        let mut input_data: Vec<Vec<f32>> = Vec::with_capacity(input_count);

        // Process inputs in the correct order based on tensor names
        for i in 0..input_count {
            let name = unsafe {
                let name_ptr = get_input_name_at(i as c_int);
                if name_ptr.is_null() {
                    return Err(eyre::eyre!("Failed to get input name at index {}", i));
                }
                std::ffi::CStr::from_ptr(name_ptr)
                    .to_string_lossy()
                    .into_owned()
            };

            let input = inputs
                .get(&name)
                .ok_or_else(|| eyre::eyre!("Missing input tensor: {}", name))?;

            let expected_size =
                unsafe { get_input_size_at(i as c_int) } / std::mem::size_of::<f32>();
            if input.len() != expected_size {
                return Err(eyre::eyre!(
                    "Input '{}' size mismatch: expected {}, got {}",
                    name,
                    expected_size,
                    input.len()
                ));
            }

            input_data.push(input.clone());
            input_ptrs.push(input_data.last().unwrap().as_ptr());
        }

        // Prepare output buffers
        let mut output_data: Vec<Vec<f32>> = Vec::with_capacity(output_count);
        let mut output_ptrs: Vec<*mut c_float> = Vec::with_capacity(output_count);

        for i in 0..output_count {
            let size = unsafe { get_output_size_at(i as c_int) } / std::mem::size_of::<f32>();
            output_data.push(vec![0.0f32; size]);
            output_ptrs.push(output_data.last_mut().unwrap().as_mut_ptr());
        }

        // Perform inference
        let result = unsafe {
            forward(
                input_ptrs.as_ptr(),
                input_count as c_int,
                output_ptrs.as_ptr(),
                output_count as c_int,
            )
        };

        if result != 0 {
            return Err(eyre::eyre!("Forward pass failed"));
        }

        // Build output hashmap with correct tensor names
        let mut outputs = HashMap::new();
        for i in 0..output_count {
            let name = unsafe {
                let name_ptr = get_output_name_at(i as c_int);
                if name_ptr.is_null() {
                    return Err(eyre::eyre!("Failed to get output name at index {}", i));
                }
                std::ffi::CStr::from_ptr(name_ptr)
                    .to_string_lossy()
                    .into_owned()
            };
            outputs.insert(name, output_data[i].clone());
        }

        Ok(outputs)
    }

    pub fn get_input_info(&self) -> Result<Vec<TensorInfo>> {
        let input_count = unsafe { get_input_count() } as usize;
        let mut info = Vec::with_capacity(input_count);

        for i in 0..input_count {
            let name = unsafe {
                let name_ptr = get_input_name_at(i as c_int);
                if name_ptr.is_null() {
                    eyre::bail!("Failed to get input name at index {}", i);
                }
                std::ffi::CStr::from_ptr(name_ptr)
                    .to_string_lossy()
                    .into_owned()
            };

            let size = unsafe { get_input_size_at(i as c_int) } / std::mem::size_of::<f32>();

            let mut dims = [0i32; 6];
            let mut dim_count: usize = 0;
            let shape_result =
                unsafe { get_input_shape_at(i as c_int, dims.as_mut_ptr(), &mut dim_count) };
            if shape_result != 0 {
                eyre::bail!("Failed to get input shape at index {}", i);
            }

            info.push(TensorInfo {
                name,
                shape: dims[..dim_count].to_vec(),
                size,
            });
        }

        Ok(info)
    }

    pub fn get_output_info(&self) -> Result<Vec<TensorInfo>> {
        let output_count = unsafe { get_output_count() } as usize;
        let mut info = Vec::with_capacity(output_count);

        for i in 0..output_count {
            // Get the actual tensor name from the model
            let name = unsafe {
                let name_ptr = get_output_name_at(i as c_int);
                if name_ptr.is_null() {
                    eyre::bail!("Failed to get output name at index {}", i);
                }
                std::ffi::CStr::from_ptr(name_ptr)
                    .to_string_lossy()
                    .into_owned()
            };

            let size = unsafe { get_output_size_at(i as c_int) } / std::mem::size_of::<f32>();

            let mut dims = [0i32; 6];
            let mut dim_count: usize = 0;
            let shape_result =
                unsafe { get_output_shape_at(i as c_int, dims.as_mut_ptr(), &mut dim_count) };
            if shape_result != 0 {
                eyre::bail!("Failed to get output shape at index {}", i);
            }

            info.push(TensorInfo {
                name, // Use the actual tensor name from the model
                shape: dims[..dim_count].to_vec(),
                size,
            });
        }

        Ok(info)
    }
}

lazy_static! {
    static ref CLEANUP_MUTEX: Mutex<()> = Mutex::new(());
}

impl Drop for Model {
    fn drop(&mut self) {
        let _lock = CLEANUP_MUTEX.lock().unwrap();
        unsafe { cleanup() };
    }
}
