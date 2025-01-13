mod controller;
mod model;

use anyhow::{Context, Result};
use std::path::PathBuf;
use model::Model;
use crate::controller::Robot;
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<()> {
    // initialize robot
    let robot = Robot::new().context("Failed to initialize robot")?;

    // load model
    let model_path = PathBuf::from("/root/models/ppo_walking.cvimodel"); // PATH IN MILK-V
    let model = Model::new(model_path).context("Failed to load model")?;
    let model_arc = Arc::new(model);

    // run controller
    controller::run(model_arc, Arc::new(robot)).context("Controller encountered an error")
}