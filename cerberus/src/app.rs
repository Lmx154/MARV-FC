use std::time::Duration;

use eframe::egui;

use crate::bridge::AppBridge;
use crate::frontend::FrontendView;

pub struct CerberusApp {
    bridge: AppBridge,
    frontend_view: FrontendView,
}

impl CerberusApp {
    pub fn new(_creation_context: &eframe::CreationContext<'_>) -> Self {
        Self {
            bridge: AppBridge::new(),
            frontend_view: FrontendView::new(),
        }
    }
}

impl eframe::App for CerberusApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.bridge.tick();
        let state = self.bridge.snapshot();
        self.frontend_view.render(ctx, &state, &mut self.bridge);
        ctx.request_repaint_after(Duration::from_millis(20));
    }
}
