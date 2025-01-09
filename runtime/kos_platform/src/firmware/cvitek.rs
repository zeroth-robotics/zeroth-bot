use std::ffi::CString;
use std::os::raw::{c_char, c_float, c_int};
use std::path::Path;
use eyre::Result;
use std::sync::{Mutex, LazyLock};

#[link(name = "cviwrapper")]
extern "C" {
    fn init_model(model_path: *const c_char) -> c_int;
    fn forward(input_data: *const c_float, output_data: *mut c_float) -> c_int;
    fn cleanup();
    fn get_input_size() -> usize;
    fn get_output_size() -> usize;
}

pub struct Model {
    _private: (), // Prevent direct construction
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

    pub fn infer(&self, input: &[f32]) -> Result<Vec<f32>> {
        let input_size = unsafe { get_input_size() };
        let output_size = unsafe { get_output_size() };

        if input.len() != input_size / std::mem::size_of::<f32>() {
            eyre::bail!("Input size mismatch");
        }

        let mut output = vec![0.0f32; output_size / std::mem::size_of::<f32>()];

        let result = unsafe { forward(input.as_ptr(), output.as_mut_ptr()) };

        if result != 0 {
            eyre::bail!("Forward pass failed");
        }

        Ok(output)
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
