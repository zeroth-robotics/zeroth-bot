use kos::daemon::kos_runtime;
use kos_zeroth_01::ZBotPlatform;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let platform = Box::new(ZBotPlatform::new());
    kos_runtime(platform).await.map_err(|e| {
        eprintln!("Runtime error: {}", e);
        e
    })?;
    Ok(())
}