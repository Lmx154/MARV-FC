//! Legacy compatibility surface for preserved LoRa RF config modules.

pub mod lora {
    #[path = "rf_config.rs"]
    pub mod rf_config;

    #[path = "rf_presets.rs"]
    pub mod rf_presets;
}
