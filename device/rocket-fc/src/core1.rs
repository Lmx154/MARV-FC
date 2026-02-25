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
        usb.wait_connection().await;
        info!("USB: Host connected");

        {
            let (statustext_en, policy) = {
                let p = params.lock().await;
                (p.bool(ParamId::StatustextEn), fc_policy_from_params(&p))
            };
            let mut link_failed = false;

            {
                let mut state = fc_state.lock().await;
                if let Some(snapshot) = state.set_usb_connected(true, policy) {
                    if statustext_en {
                        let status = build_statustext(snapshot.status_text());
                        let frame = build_statustext_frame(cfg, seq, status);
                        seq = seq.wrapping_add(1);
                        if let Ok(len) = frame.serialize(&mut tx_buf) {
                            if let Err(e) = usb.write(&tx_buf[..len]).await {
                                if matches!(e, EndpointError::Disabled) {
                                    link_failed = true;
                                }
                            }
                        }
                    }
                }
            }

            if statustext_en {
                let status = build_statustext("MARV-FC ready");
                let frame = build_statustext_frame(cfg, seq, status);
                seq = seq.wrapping_add(1);
                if let Ok(len) = frame.serialize(&mut tx_buf) {
                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                        if matches!(e, EndpointError::Disabled) {
                            link_failed = true;
                        }
                    }
                }
            }

            if link_failed {
                let mut state = fc_state.lock().await;
                let _ = state.set_usb_connected(false, policy);
                continue;
            }
        }

        loop {
            let recv_result = embassy_futures::select::select(
                usb.read_packet(&mut rx_buf),
                Timer::after_millis(10),
            )
            .await;

            let mut link_alive = true;

            match recv_result {
                embassy_futures::select::Either::First(Ok(n)) if n > 0 => {
                    if let Ok(frame) = unsafe { Frame::<V2>::deserialize(&rx_buf[..n]) } {
                        let (result, params_changed) = {
                            let mut state = fc_state.lock().await;
                            let mut p = params.lock().await;
                            let policy = fc_policy_from_params(&p);
                            let statustext_en = p.bool(ParamId::StatustextEn);
                            dispatch_mavlink_message(
                                frame,
                                cfg,
                                seq,
                                &mut p,
                                &mut state,
                                policy,
                                statustext_en,
                            )
                        };

                        match result {
                            MessageHandlerResult::SendFrame(reply) => {
                                seq = seq.wrapping_add(1);
                                if let Ok(len) = reply.serialize(&mut tx_buf) {
                                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                                        if matches!(e, EndpointError::Disabled) {
                                            link_alive = false;
                                        }
                                    }
                                }
                            }
                            MessageHandlerResult::SendFrames(frames) => {
                                for frame in frames.into_iter() {
                                    seq = seq.wrapping_add(1);
                                    if let Ok(len) = frame.serialize(&mut tx_buf) {
                                        if let Err(e) = usb.write(&tx_buf[..len]).await {
                                            if matches!(e, EndpointError::Disabled) {
                                                link_alive = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                            MessageHandlerResult::SendAllParams => {
                                let param_count = {
                                    let p = params.lock().await;
                                    p.count()
                                };
                                info!(
                                    "MAVLink: PARAM_REQUEST_LIST - sending {} params",
                                    param_count
                                );
                                for idx in 0..param_count {
                                    let reply = {
                                        let p = params.lock().await;
                                        build_param_value_frame(cfg, seq, &p, idx)
                                    };
                                    if let Some(reply) = reply {
                                        seq = seq.wrapping_add(1);
                                        if let Ok(len) = reply.serialize(&mut tx_buf) {
                                            if let Err(e) = usb.write(&tx_buf[..len]).await {
                                                if matches!(e, EndpointError::Disabled) {
                                                    link_alive = false;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                                info!("MAVLink: PARAM_REQUEST_LIST complete");
                            }
                            MessageHandlerResult::NoResponse | MessageHandlerResult::Unhandled => {}
                            MessageHandlerResult::DeviceOpRead(req) => {
                                const OP_OK: u8 = 0;
                                const OP_BAD_BUS: u8 = 1;
                                const OP_BAD_DEV: u8 = 2;
                                const OP_BAD_RESPONSE: u8 = 4;
                                const BUS_I2C: u8 = 0;

                                let mut data = [0u8; 128];
                                let mut data_len: usize = 0;

                                let result_code = if req.bustype != BUS_I2C {
                                    OP_BAD_BUS
                                } else {
                                    let addr = req.address;
                                    if req.count == 0 {
                                        let probe: [u8; 1] = [0x00];
                                        let res = match req.bus {
                                            0 => {
                                                let mut bus = SharedI2c0 { bus: i2c0 };
                                                bus.write(addr, &probe).await
                                            }
                                            1 => {
                                                let mut bus = SharedI2c { bus: i2c1 };
                                                bus.write(addr, &probe).await
                                            }
                                            _ => return,
                                        };

                                        match res {
                                            Ok(()) => OP_OK,
                                            Err(_) => OP_BAD_DEV,
                                        }
                                    } else {
                                        let n = core::cmp::min(req.count as usize, data.len());
                                        let reg = [req.regstart];
                                        let res = match req.bus {
                                            0 => {
                                                let mut bus = SharedI2c0 { bus: i2c0 };
                                                bus.write_read(addr, &reg, &mut data[..n]).await
                                            }
                                            1 => {
                                                let mut bus = SharedI2c { bus: i2c1 };
                                                bus.write_read(addr, &reg, &mut data[..n]).await
                                            }
                                            _ => return,
                                        };

                                        match res {
                                            Ok(()) => {
                                                data_len = n;
                                                OP_OK
                                            }
                                            Err(_) => OP_BAD_RESPONSE,
                                        }
                                    }
                                };

                                let reply = build_device_op_read_reply_frame(
                                    cfg,
                                    seq,
                                    req.request_id,
                                    result_code,
                                    req.regstart,
                                    &data[..data_len],
                                );
                                seq = seq.wrapping_add(1);
                                if let Ok(len) = reply.serialize(&mut tx_buf) {
                                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                                        if matches!(e, EndpointError::Disabled) {
                                            link_alive = false;
                                        }
                                    }
                                }
                            }
                            MessageHandlerResult::DeviceOpWrite(req) => {
                                const OP_OK: u8 = 0;
                                const OP_BAD_BUS: u8 = 1;
                                const OP_BAD_RESPONSE: u8 = 4;
                                const BUS_I2C: u8 = 0;

                                let result_code = if req.bustype != BUS_I2C {
                                    OP_BAD_BUS
                                } else {
                                    let addr = req.address;
                                    let n = core::cmp::min(req.count as usize, 128);
                                    let mut payload = [0u8; 129];
                                    payload[0] = req.regstart;
                                    payload[1..1 + n].copy_from_slice(&req.data[..n]);

                                    let res = match req.bus {
                                        0 => {
                                            let mut bus = SharedI2c0 { bus: i2c0 };
                                            bus.write(addr, &payload[..1 + n]).await
                                        }
                                        1 => {
                                            let mut bus = SharedI2c { bus: i2c1 };
                                            bus.write(addr, &payload[..1 + n]).await
                                        }
                                        _ => return,
                                    };

                                    match res {
                                        Ok(()) => OP_OK,
                                        Err(_) => OP_BAD_RESPONSE,
                                    }
                                };

                                let reply = build_device_op_write_reply_frame(
                                    cfg,
                                    seq,
                                    req.request_id,
                                    result_code,
                                );
                                seq = seq.wrapping_add(1);
                                if let Ok(len) = reply.serialize(&mut tx_buf) {
                                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                                        if matches!(e, EndpointError::Disabled) {
                                            link_alive = false;
                                        }
                                    }
                                }
                            }
                        }

                        if params_changed {
                            PARAMS_DIRTY.store(true, Ordering::Relaxed);
                            PARAMS_LAST_MODIFIED
                                .store(Instant::now().as_millis() as u32, Ordering::Relaxed);
                        }
                    }
                }
                embassy_futures::select::Either::First(Ok(_)) => {}
                embassy_futures::select::Either::First(Err(e)) => {
                    if matches!(e, EndpointError::Disabled) {
                        link_alive = false;
                    } else {
                        debug!("USB: Read error");
                    }
                }
                embassy_futures::select::Either::Second(_) => {}
            }

            if !link_alive {
                break;
            }

            let now_ms = Instant::now().as_millis() as u32;
            {
                let p = params.lock().await;
                let telem_rate_hz = p.u32(ParamId::TelemRateHz).min(1000);
                let hb_en = p.bool(ParamId::HeartbeatEn);
                let hb_hz = fc_heartbeat_hz(&p);

                sched.set_telemetry_hz(now_ms, telem_rate_hz);
                sched.set_heartbeat_enabled(hb_en);
                sched.set_heartbeat_hz(now_ms, hb_hz);
            }

            let decision = sched.poll(now_ms);

            if decision.send_telemetry {
                let now_us = (now_ms as u64) * 1000;
                let bundle = source.bundle(now_ms, now_us).await;

                let frames = [
                    build_frame_from_msg(cfg, seq.wrapping_add(0), &bundle.sys_status),
                    build_frame_from_msg(cfg, seq.wrapping_add(1), &bundle.raw_imu),
                    build_frame_from_msg(cfg, seq.wrapping_add(2), &bundle.scaled_pressure),
                    build_frame_from_msg(cfg, seq.wrapping_add(3), &bundle.gps_raw_int),
                    build_frame_from_msg(cfg, seq.wrapping_add(4), &bundle.attitude),
                ];

                for frame in frames.iter() {
                    seq = seq.wrapping_add(1);
                    if let Ok(len) = frame.serialize(&mut tx_buf) {
                        if let Err(e) = usb.write(&tx_buf[..len]).await {
                            if matches!(e, EndpointError::Disabled) {
                                link_alive = false;
                                break;
                            }
                        }
                    }
                }
            }

            if decision.send_heartbeat {
                let snapshot = {
                    let state = fc_state.lock().await;
                    state.snapshot()
                };
                let frame = build_heartbeat_frame(cfg, seq, &snapshot, hb_cfg);
                seq = seq.wrapping_add(1);
                if let Ok(len) = frame.serialize(&mut tx_buf) {
                    if let Err(e) = usb.write(&tx_buf[..len]).await {
                        if matches!(e, EndpointError::Disabled) {
                            link_alive = false;
                        }
                    }
                }
            }

            if PARAMS_DIRTY.load(Ordering::Relaxed) {
                let last_mod = PARAMS_LAST_MODIFIED.load(Ordering::Relaxed);
                let now_ms = Instant::now().as_millis() as u32;

                if now_ms.wrapping_sub(last_mod) >= 5000 {
                    info!("Saving parameters to SD card...");
                    let p = params.lock().await;
                    match save_params_to_sd(&p) {
                        Ok(()) => {
                            info!("Parameters saved successfully");
                            PARAMS_DIRTY.store(false, Ordering::Relaxed);
                        }
                        Err(e) => {
                            warn!("Failed to save parameters: {}", e);
                        }
                    }
                    drop(p);
                }
            }

            if !link_alive {
                break;
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
