use std::env;
use std::net::SocketAddr;
use std::path::PathBuf;

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct Config {
    pub bind_addr: SocketAddr,
    pub actuator_addr: SocketAddr,
    pub tick_timeout_ms: u64,
    pub log_dir: PathBuf,
    pub system_id: u8,
    pub component_id: u8,
}

impl Config {
    pub fn from_env() -> Result<Self, String> {
        Ok(Self {
            bind_addr: socket_addr_var("FC_SITL_BIND", "127.0.0.1:14560")?,
            actuator_addr: socket_addr_var("FC_SITL_ACTUATOR_ADDR", "127.0.0.1:14561")?,
            tick_timeout_ms: u64_var("FC_SITL_TICK_TIMEOUT_MS", 500)?,
            log_dir: PathBuf::from(
                env::var("FC_SITL_LOG_DIR").unwrap_or_else(|_| "simulator/out".into()),
            ),
            system_id: u8_var("FC_SITL_SYSTEM_ID", 42)?,
            component_id: u8_var("FC_SITL_COMPONENT_ID", 1)?,
        })
    }
}

fn socket_addr_var(key: &str, default: &str) -> Result<SocketAddr, String> {
    env::var(key)
        .unwrap_or_else(|_| default.into())
        .parse()
        .map_err(|error| format!("invalid {key}: {error}"))
}

fn u64_var(key: &str, default: u64) -> Result<u64, String> {
    env::var(key)
        .ok()
        .map(|value| {
            value
                .parse()
                .map_err(|error| format!("invalid {key}: {error}"))
        })
        .transpose()?
        .map(|value: u64| value.max(1))
        .ok_or_else(|| format!("invalid {key}"))
        .or(Ok(default))
}

fn u8_var(key: &str, default: u8) -> Result<u8, String> {
    env::var(key)
        .ok()
        .map(|value| {
            value
                .parse()
                .map_err(|error| format!("invalid {key}: {error}"))
        })
        .transpose()?
        .ok_or_else(|| format!("invalid {key}"))
        .or(Ok(default))
}
