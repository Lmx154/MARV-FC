mod api;
mod app;
mod fs_access;
mod sim_loop;
mod state;
mod types;
mod ui;
mod ws;

#[tokio::main]
async fn main() -> Result<(), std::io::Error> {
    app::run().await
}
