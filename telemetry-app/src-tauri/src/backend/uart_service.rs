use std::io::{ErrorKind, Read, Write};
use std::time::Duration;

use serde::Serialize;
use serialport::{
    DataBits, FlowControl, Parity, SerialPort, SerialPortInfo, SerialPortType, StopBits,
};

pub const DEFAULT_UART_BAUD_RATE: u32 = 115_200;

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct UartPortInfo {
    pub port_name: String,
    pub display_name: String,
}

pub struct UartService {
    port: Option<Box<dyn SerialPort>>,
    port_name: Option<String>,
    baud_rate: u32,
    available_ports: Vec<UartPortInfo>,
    last_error: Option<String>,
}

impl UartService {
    pub fn new() -> Self {
        let mut service = Self {
            port: None,
            port_name: None,
            baud_rate: DEFAULT_UART_BAUD_RATE,
            available_ports: Vec::new(),
            last_error: None,
        };
        service.list_ports();
        service
    }

    pub fn open_with_baud_rate(&mut self, port_name: &str, baud_rate: u32) -> Result<(), String> {
        let port_name = port_name.trim();
        if port_name.is_empty() {
            let error = "no UART port selected".to_string();
            self.last_error = Some(error.clone());
            return Err(error);
        }

        if baud_rate == 0 {
            let error = "UART baud rate must be greater than zero".to_string();
            self.last_error = Some(error.clone());
            return Err(error);
        }

        if self.is_open() {
            self.close();
        }

        match serialport::new(port_name, baud_rate)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(Duration::from_millis(100))
            .open()
        {
            Ok(port) => {
                self.port = Some(port);
                self.port_name = Some(port_name.to_string());
                self.baud_rate = baud_rate;
                self.last_error = None;
                Ok(())
            }
            Err(error) => {
                let error = format!("failed to open {port_name}: {error}");
                self.last_error = Some(error.clone());
                Err(error)
            }
        }
    }

    pub fn close(&mut self) {
        self.port = None;
        self.port_name = None;
    }

    pub fn list_ports(&mut self) -> Vec<UartPortInfo> {
        match serialport::available_ports() {
            Ok(ports) => {
                self.available_ports = ports
                    .into_iter()
                    .map(|port| UartPortInfo {
                        display_name: format_port_display_name(&port),
                        port_name: port.port_name,
                    })
                    .collect();
                self.available_ports
                    .sort_by(|a, b| a.display_name.cmp(&b.display_name));
                self.last_error = None;
            }
            Err(error) => {
                self.available_ports.clear();
                self.last_error = Some(format!("failed to list UART ports: {error}"));
            }
        }

        self.available_ports.clone()
    }

    pub fn poll_connection(&mut self) {
        let Some(active_port) = self.port_name.clone() else {
            return;
        };

        let io_error = self
            .port
            .as_mut()
            .and_then(|port| port.bytes_to_read().err());

        let mut port_missing = false;
        match serialport::available_ports() {
            Ok(ports) => {
                self.available_ports = ports
                    .into_iter()
                    .map(|port| UartPortInfo {
                        display_name: format_port_display_name(&port),
                        port_name: port.port_name,
                    })
                    .collect();
                self.available_ports
                    .sort_by(|a, b| a.display_name.cmp(&b.display_name));
                port_missing = !self
                    .available_ports
                    .iter()
                    .any(|port| port.port_name == active_port);
            }
            Err(error) => {
                self.last_error = Some(format!("failed to list UART ports: {error}"));
            }
        }

        if io_error.is_none() && !port_missing {
            return;
        }

        self.close();
        let reason = if let Some(error) = io_error {
            format!("I/O probe failed: {error}")
        } else {
            "device is no longer present".to_string()
        };
        self.last_error = Some(format!("UART port {active_port} disconnected ({reason})"));
    }

    pub fn read_available(&mut self) -> Result<Vec<u8>, String> {
        let Some(port) = self.port.as_mut() else {
            return Ok(Vec::new());
        };

        let bytes_to_read = match port.bytes_to_read() {
            Ok(bytes_to_read) => bytes_to_read as usize,
            Err(error) => {
                let error = format!("failed to inspect UART read buffer: {error}");
                self.last_error = Some(error.clone());
                return Err(error);
            }
        };

        if bytes_to_read == 0 {
            return Ok(Vec::new());
        }

        let mut bytes = vec![0u8; bytes_to_read.min(4096)];
        match port.read(&mut bytes) {
            Ok(read_len) => {
                bytes.truncate(read_len);
                Ok(bytes)
            }
            Err(error) if error.kind() == ErrorKind::TimedOut => Ok(Vec::new()),
            Err(error) => {
                let error = format!("failed to read UART bytes: {error}");
                self.last_error = Some(error.clone());
                Err(error)
            }
        }
    }

    pub fn write_all(&mut self, bytes: &[u8]) -> Result<(), String> {
        let Some(port) = self.port.as_mut() else {
            let error = "UART port is not open".to_string();
            self.last_error = Some(error.clone());
            return Err(error);
        };

        if let Err(error) = port.write_all(bytes) {
            let error = format!("failed to write UART bytes: {error}");
            self.last_error = Some(error.clone());
            return Err(error);
        }

        if let Err(error) = port.flush() {
            let error = format!("failed to flush UART bytes: {error}");
            self.last_error = Some(error.clone());
            return Err(error);
        }

        Ok(())
    }

    #[allow(dead_code)]
    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), String> {
        if baud_rate == 0 {
            let error = "UART baud rate must be greater than zero".to_string();
            self.last_error = Some(error.clone());
            return Err(error);
        }

        if let Some(port) = self.port.as_mut() {
            if let Err(error) = port.set_baud_rate(baud_rate) {
                let error = format!("failed to set UART baud rate to {baud_rate}: {error}");
                self.last_error = Some(error.clone());
                return Err(error);
            }
        }

        self.baud_rate = baud_rate;
        self.last_error = None;
        Ok(())
    }

    pub fn available_ports(&self) -> &[UartPortInfo] {
        &self.available_ports
    }

    pub fn is_open(&self) -> bool {
        self.port.is_some()
    }

    pub fn port_name(&self) -> Option<&str> {
        self.port_name.as_deref()
    }

    pub fn baud_rate(&self) -> u32 {
        self.baud_rate
    }

    pub fn line_coding_label(&self) -> &'static str {
        "8N1"
    }

    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }
}

fn format_port_display_name(port: &SerialPortInfo) -> String {
    let label = match &port.port_type {
        SerialPortType::UsbPort(usb) => {
            let mut parts = Vec::new();
            if let Some(product) = usb
                .product
                .as_deref()
                .filter(|value| !value.trim().is_empty())
            {
                parts.push(product.trim().to_string());
            }
            if let Some(manufacturer) = usb
                .manufacturer
                .as_deref()
                .filter(|value| !value.trim().is_empty())
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
