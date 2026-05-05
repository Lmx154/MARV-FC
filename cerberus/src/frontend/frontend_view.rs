use eframe::egui;

use crate::bridge::{AppBridge, AppState};

use super::tabs::mission_tab::MissionTab;
use super::tabs::settings_tab::{GlobalTheme, SettingsTabState};
use super::tabs::{data_tab, debug_tab, hil_tab, settings_tab};

#[derive(Clone, Copy, PartialEq, Eq)]
enum FrontendTab {
    Settings,
    Debug,
    Mission,
    Hil,
    Data,
}

pub struct FrontendView {
    active_tab: FrontendTab,
    theme: GlobalTheme,
    debug_tab: debug_tab::DebugTabState,
    mission_tab: MissionTab,
    hil_tab: hil_tab::HilTabState,
    settings_tab: SettingsTabState,
}

impl FrontendView {
    pub fn new() -> Self {
        Self {
            active_tab: FrontendTab::Mission,
            theme: GlobalTheme::Dark,
            debug_tab: debug_tab::DebugTabState::new(),
            mission_tab: MissionTab::new(),
            hil_tab: hil_tab::HilTabState::new(),
            settings_tab: SettingsTabState::new(),
        }
    }

    pub fn render(&mut self, ctx: &egui::Context, state: &AppState, bridge: &mut AppBridge) {
        self.theme.apply(ctx);

        egui::TopBottomPanel::top("tab_bar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                self.render_tab_button(ui, FrontendTab::Settings, "Settings");
                self.render_tab_button(ui, FrontendTab::Debug, "Debug");
                self.render_tab_button(ui, FrontendTab::Mission, "Mission");
                self.render_tab_button(ui, FrontendTab::Hil, "HIL");
                self.render_tab_button(ui, FrontendTab::Data, "Data");
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::both()
                .auto_shrink([false, false])
                .show(ui, |ui| match self.active_tab {
                    FrontendTab::Settings => settings_tab::render(
                        ui,
                        state,
                        bridge,
                        &mut self.theme,
                        &mut self.settings_tab,
                    ),
                    FrontendTab::Debug => debug_tab::render(ui, state, &mut self.debug_tab, bridge),
                    FrontendTab::Mission => self.mission_tab.render(ui, state, bridge),
                    FrontendTab::Hil => hil_tab::render(ui, state, &mut self.hil_tab, bridge),
                    FrontendTab::Data => data_tab::render(ui),
                });
        });
    }

    fn render_tab_button(&mut self, ui: &mut egui::Ui, tab: FrontendTab, label: &str) {
        if ui.selectable_label(self.active_tab == tab, label).clicked() {
            self.active_tab = tab;
        }
    }
}
