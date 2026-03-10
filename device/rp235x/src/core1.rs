use super::*;

#[embassy_executor::task]
pub(crate) async fn imu_task(mut imu: Bmi088<Spi1Device, Spi1Device>, interval_ms: u32) {
    let mut delay = EmbassyDelay;
    let mut sink = ImuSink;
    run_bmi088_task(&mut imu, &mut delay, &mut sink, interval_ms).await;
}

#[embassy_executor::task]
pub(crate) async fn imu2_task(mut imu: Lsm6dsv32x<Spi1Device>, interval_ms: u32) {
    let mut delay = EmbassyDelay;
    let mut sink = Imu2Sink;
    run_lsm6dsv32x_task(&mut imu, &mut delay, &mut sink, interval_ms).await;
}

#[embassy_executor::task]
pub(crate) async fn mag_task(i2c_dev: SharedI2c<'static>, interval_ms: u32) {
    let mut delay = EmbassyDelay;
    info!("BMM350: starting task on I2C1 @0x{:02X}", BMM350_ADDR);
    let mut mag = Bmm350::new(i2c_dev, BMM350_ADDR);
    mag.set_debug(false);
    let mut sink = MagSink;
    run_bmm350_task(&mut mag, &mut delay, &mut sink, interval_ms, false).await;
}

#[embassy_executor::task]
pub(crate) async fn baro_task(i2c_dev: SharedI2c0<'static>, interval_ms: u32, bmp_addr: u8) {
    let mut pacing = EmbassyDelay;
    info!("BMP581: starting task on I2C0 @0x{:02X}", bmp_addr);
    let bmp_cfg = Bmp581Config::default();
    let mut bmp = Bmp581::new(i2c_dev, EmbassyDelay, bmp_addr, bmp_cfg);
    let mut sink = BaroSink;
    run_bmp581_task(&mut bmp, &mut pacing, &mut sink, interval_ms).await;
}

#[embassy_executor::task]
pub(crate) async fn gps_task(i2c_dev: SharedI2c<'static>, interval_ms: u32) {
    let mut delay = EmbassyDelay;
    info!("NEO-M9N: starting task on I2C1 @0x{:02X}", UBLOX_I2C_ADDR);
    let mut gps = NeoM9n::new(i2c_dev, UBLOX_I2C_ADDR);
    gps.set_debug(false);
    let mut sink = GpsSink;
    run_neom9n_task(&mut gps, &mut delay, &mut sink, interval_ms, false).await;
}

#[embassy_executor::task]
pub(crate) async fn usb_device_task(mut device: RpUsbDevice) {
    device.run().await;
}

#[embassy_executor::task]
pub(crate) async fn usb_mavlink_task(
    mut usb: UsbCdc<'static>,
    cfg: MavEndpointConfig,
    params: &'static Mutex<RawMutex, ParamRegistry>,
    fc_state: &'static Mutex<RawMutex, FcState>,
    i2c0: &'static Mutex<RawMutex, I2c0Type>,
    i2c1: &'static Mutex<RawMutex, I2c1Type>,
) {
    info!("USB-CDC: Starting bidirectional MAVLink task");
    let source = StateTelemetrySource;
    let hb_cfg = HeartbeatConfig::default();

    let mut seq: u8 = 0;
    let mut tx_buf = [0u8; MAVLINK_MAX_FRAME];
    let mut rx_buf = [0u8; MAVLINK_MAX_FRAME];

    let now_ms = Instant::now().as_millis() as u32;
    let mut sched = LinkScheduler::new(now_ms, 1, 50);

    loop {
        info!("USB: Waiting for host connection...");
            }
        }

        {
            let policy = {
                let p = params.lock().await;
                fc_policy_from_params(&p)
            };
            let mut state = fc_state.lock().await;
            let _ = state.set_usb_connected(false, policy);
        }
    }
}
