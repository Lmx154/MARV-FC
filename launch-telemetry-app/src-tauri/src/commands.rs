//! Tauri commands invoked from the React frontend.
//!
//! Connect/disconnect/send_command are fire-and-forget: they hand a `ControlMsg` to the serial
//! worker and return immediately. The real outcome (success, errors, link drops) arrives on the
//! `link-status` and `debug` events.

use serialport::{SerialPortInfo, SerialPortType};
use tauri::State;

use crate::model::PortInfo;
use crate::state::{AppState, CommandKind, ControlMsg};

/// Enumerate available serial ports for the Settings dropdown.
#[tauri::command]
pub fn list_serial_ports() -> Vec<PortInfo> {
    let mut ports: Vec<PortInfo> = match serialport::available_ports() {
        Ok(ports) => ports
            .into_iter()
            .map(|p| PortInfo {
                display_name: format_port_display_name(&p),
                port_name: p.port_name,
            })
            .collect(),
        Err(_) => Vec::new(),
    };
    ports.sort_by(|a, b| a.display_name.cmp(&b.display_name));
    ports
}

#[tauri::command]
pub fn connect(state: State<AppState>, port: String, baud: u32) -> Result<(), String> {
    if port.trim().is_empty() {
        return Err("no serial port selected".to_string());
    }
    if baud == 0 {
        return Err("baud rate must be greater than zero".to_string());
    }
    state
        .tx
        .send(ControlMsg::Connect { port, baud })
        .map_err(|e| e.to_string())
}

#[tauri::command]
pub fn disconnect(state: State<AppState>) -> Result<(), String> {
    state
        .tx
        .send(ControlMsg::Disconnect)
        .map_err(|e| e.to_string())
}

#[tauri::command]
pub fn send_command(state: State<AppState>, kind: String) -> Result<(), String> {
    let kind = match kind.as_str() {
        "arm" => CommandKind::Arm,
        "disarm" => CommandKind::Disarm,
        "ping" => CommandKind::Ping,
        "motor_stop" => CommandKind::MotorStop,
        other => return Err(format!("unknown command: {other}")),
    };
    state
        .tx
        .send(ControlMsg::SendCommand(kind))
        .map_err(|e| e.to_string())
}

fn format_port_display_name(port: &SerialPortInfo) -> String {
    let label = match &port.port_type {
        SerialPortType::UsbPort(usb) => {
            let mut parts = Vec::new();
            if let Some(product) = usb.product.as_deref().filter(|v| !v.trim().is_empty()) {
                parts.push(product.trim().to_string());
            }
            if let Some(manufacturer) =
                usb.manufacturer.as_deref().filter(|v| !v.trim().is_empty())
            {
                parts.push(manufacturer.trim().to_string());
            }
            if parts.is_empty() {
                "USB serial device".to_string()
            } else {
                parts.join(" - ")
            }
        }
        SerialPortType::BluetoothPort => "Bluetooth serial device".to_string(),
        SerialPortType::PciPort => "PCI serial device".to_string(),
        SerialPortType::Unknown => "Serial device".to_string(),
    };
    format!("{label} ({})", port.port_name)
}
