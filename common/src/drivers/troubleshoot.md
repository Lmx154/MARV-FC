#### Entry: SX1262 LoRa transceiver stuck in StandbyRc mode, unable to enter TX mode
- Title: SX1262 accepts SetTx command but remains in StandbyRc instead of transitioning to TX mode; IRQ register shows 0x0000 (no interrupts)
- Date: 2025-10-04
- Scope: repo MARV, branch sx1262-driver, binary gs
- Symptom: After hardware reset and initialization sequence (TCXO config, calibration, LoRa parameter setup), the SX1262 successfully enters StandbyRc mode. However, when transmit() calls SetTx (command 0x83), the device accepts the command (BUSY pin goes low) but never transitions to TX mode. GetStatus continues reporting StandbyRc, IRQ status remains 0x0000, and DIO1 never asserts, causing TX timeout after 5 seconds.
- Root cause:
  - **Missing SetRegulatorMode command**: The SX1262 defaults to LDO regulator mode, but most high-frequency modules (like the Waveshare Core1262-HF at 915 MHz) require DC-DC regulator mode for sufficient power delivery during TX operations. Without explicitly setting DC-DC mode via SetRegulatorMode (0x96), the chip lacks power headroom to transition from standby to the higher-current TX state.
  - **Missing IRQ reconfiguration before TX**: The lora-rs reference implementation calls set_irq_params() with mode-specific masks before each operation. During initialization, IRQs are configured for Standby mode (all interrupts on DIO1). Before transmitting, the IRQ configuration must be updated to TX mode (only TxDone + RxTxTimeout on DIO1). Without this reconfiguration, the chip may not properly route TX completion events to DIO1, preventing interrupt-driven TX completion detection.
- Fix:
  - Add SetRegulatorMode (0x96) command at the very beginning of initialize(), immediately after hardware reset, to configure DC-DC regulator mode (parameter 0x01).
  - Add configure_irq() call in transmit() before SetTx to reconfigure IRQ masks for TX mode (0x0201 = TxDone | RxTxTimeout on DIO1).
  - Follow lora-rs initialization order exactly: SetRegulatorMode → ClearDeviceErrors → SetTCXOMode → Calibrate → SetStandby → configure LoRa params.
- Steps:
  1) In `gs/drivers/sx1262.rs`, add `set_regulator_mode()` function:
     ```rust
     fn set_regulator_mode(&mut self, use_dcdc: bool) -> Result<()> {
         let param = if use_dcdc { 0x01 } else { 0x00 };
         self.write_cmd(0x96, &[param])
     }
     ```
  2) In `initialize()`, call `set_regulator_mode(true)` immediately after `hardware_reset()`, before clearing device errors:
     ```rust
     pub fn initialize(&mut self) -> Result<()> {
         info!("SX1262: initialize start");
         self.hardware_reset()?;
         self.set_regulator_mode(true)?;  // ← CRITICAL: DC-DC mode first
         self.clear_device_errors()?;
         // ... rest of initialization
     }
     ```
  3) In `transmit()`, add IRQ reconfiguration before SetTx:
     ```rust
     pub fn transmit(&mut self, payload: &[u8], timeout_ms: u32) -> Result<()> {
         // ... enter standby, configure packet, write buffer ...
         
         // CRITICAL - Reconfigure IRQ for TX mode!
         let tx_irq_mask: u16 = 0x0201; // TxDone | RxTxTimeout
         self.configure_irq(tx_irq_mask)?;
         
         self.set_rf_switch(true)?;
         self.set_tx(0)?;
         // ... wait for completion ...
     }
     ```
  4) Build with `cargo build --bin gs` and flash via `cargo run --bin gs`.
- Verification:
  - RTT logs confirm initialization completes without errors: "SX1262: initialize done"
  - Transmit succeeds with "SX1262: TX done len=4 irq=0x0001" (TxDone interrupt received)
  - GetStatus after SetTx would show mode=Tx (though current code waits for IRQ, mode already transitions back to StandbyRc after TX completes)
  - DIO1 asserts upon TX completion, triggering successful wait_for_dio1() return
- Affected files:
  - `gs/drivers/sx1262.rs`:
    - Added `set_regulator_mode()` function (command 0x96)
    - Modified `initialize()` to call `set_regulator_mode(true)` after reset
    - Modified `transmit()` to call `configure_irq(0x0201)` before SetTx
  - Initialization sequence now matches lora-rs exactly:
    1. Hardware reset
    2. **SetRegulatorMode(DC-DC)** ← NEW
    3. ClearDeviceErrors
    4. SetTCXOMode (1.8V, 20ms delay)
    5. SetStandby(RC)
    6. Calibrate (all blocks: 0x7F)
    7. CalibrateImage
    8. Configure LoRa parameters (packet type, sync word, buffer base, PA config, RF frequency, modulation, packet params, TX params, RX gain)
- Notes/Gotchas:
  - **SetRegulatorMode MUST be called before TCXO configuration**: The regulator affects power delivery which impacts all subsequent operations. Call it immediately after reset.
  - **DC-DC vs LDO**: Most SX1262 modules at 868/915 MHz require DC-DC mode for TX. LDO mode (default) is less efficient and may not provide enough current for high-power TX. Always check your module's datasheet.
  - **IRQ reconfiguration is mandatory**: The SX126x requires different IRQ masks for different operations (Standby, TX, RX, CAD). Always call configure_irq() before mode transitions, not just during initialization.
  - **DIO2 as RF switch**: For modules where DIO2 is not connected (like Waveshare Core1262-HF which uses external TXEN/RXEN pins), do NOT configure DIO2 as RF switch. Manually control RF switch via GPIOs in set_rf_switch().
  - **TCXO voltage**: Common values are 1.7V or 1.8V depending on module. Waveshare Core1262-HF uses 1.8V. Wrong voltage can cause XOSC_START_ERR.
  - **StandbyRC vs StandbyXOSC**: Use StandbyRC (parameter 0x00) as default. The TCXO will automatically activate during TX/RX operations. StandbyXOSC can cause issues if TCXO isn't fully stabilized.
  - **SetTx timeout**: lora-rs always uses timeout=0 (disabled), relying on IRQ-based completion detection rather than hardware timeout. This is more reliable than calculated timeout values.
- References:
  - SX1262 Datasheet: https://www.semtech.com/products/wireless-rf/lora-connect/sx1262 (Section 13.1.14: SetRegulatorMode command; Section 13.1.4: SetDIO2AsRfSwitchCtrl)
  - lora-rs repository: https://github.com/lora-rs/lora-rs (Reference implementation for SX126x initialization and TX sequences)
  - lora-phy/src/sx126x/mod.rs: init_lora() function showing exact command order
  - Waveshare Core1262-HF schematic: DIO2 not connected, TXEN/RXEN for RF switch control