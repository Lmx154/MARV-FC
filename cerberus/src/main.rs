mod app;
mod backend;
mod bridge;
mod frontend;

use app::CerberusApp;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions::default();

    eframe::run_native(
        "Cerberus",
        native_options,
        Box::new(|creation_context| Ok(Box::new(CerberusApp::new(creation_context)))),
    )
}
