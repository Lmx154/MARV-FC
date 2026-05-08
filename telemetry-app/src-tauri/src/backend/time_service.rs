use chrono::Local;

pub struct TimeService;

impl TimeService {
    pub fn new() -> Self {
        Self
    }

    pub fn current_time(&self) -> String {
        Local::now().format("%I:%M:%S %p").to_string()
    }
}
