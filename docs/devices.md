# MARV V2 Rev 1 device facts for driver authors

Register, timing and behaviour facts for the devices on the board; wiring is in `pinout.md`. Page numbers are PDF pages of the datasheets listed at the end. Items marked UNKNOWN or "unverified" have not been checked against the datasheet or on hardware. "Measured" means measured on this board.

## ICM-45686 (TDK InvenSense, IMU on SPI0)

Pins: see `pinout.md`, SPI0 / ICM-45686.

- SPI: first byte bit7 = 1 for read, bits 6:0 address; MSB first; burst read supported; max 24 MHz (p49).
- WHO_AM_I 0x72 = 0xE9 (p107).
- PWR_MGMT0 0x10: bits 3:2 GYRO_MODE, bits 1:0 ACCEL_MODE; 11 = low noise. 6-axis low noise = 0x0F (p70).
- ACCEL_CONFIG0 0x1B: bits 6:4 FS (000 = +-32 g, 001 = +-16 g, 010 = +-8 g, 011 = +-4 g, 100 = +-2 g); bits 3:0 ODR
  (0111 = 400 Hz, 1000 = 200 Hz, 1001 = 100 Hz). Reset 0x06 = +-32 g, 800 Hz (p78).
- GYRO_CONFIG0 0x1C: bits 7:4 FS (0000 = +-4000, 0001 = +-2000, 0010 = +-1000, 0011 = +-500, 0100 = +-250 dps);
  bits 3:0 ODR, same codes. Reset 0x06 (p79).
- Data: accel X 0x00/0x01, Y 0x02/0x03, Z 0x04/0x05; gyro X 0x06/0x07, Y 0x08/0x09, Z 0x0A/0x0B; temperature
  0x0C/0x0D (p67-69). Byte order: the datasheet contradicts itself (p57 vs p64). Measured: data is little-endian.
- Accel sensitivity LSB/g: +-32 g 1024, +-16 g 2048, +-8 g 4096, +-4 g 8192, +-2 g 16384 (p19).
  Gyro sensitivity: table on p18; measured: +-250 dps is 131 LSB/dps. Other full-scale values: UNKNOWN here.
- Temperature: degC = raw/128 + 25 (p69).
- INT1_CONFIG0 0x16: bit2 = data-ready enable on INT1; reset 0x80 (p72).
- INT1_CONFIG2 0x18: bit2 drive (0 push-pull), bit1 mode (0 pulse), bit0 polarity (1 active high); reset 0x04.
  Push-pull, pulse, active high = 0x01. Write it while interrupts are disabled (p75).
- Indirect registers (p55-56, p108-109): IREG_ADDR_15_8 0x7C, IREG_ADDR_7_0 0x7D, IREG_DATA 0x7E.
  Bases: IPREG_BAR 0xA000, IPREG_TOP1 0xA200, IPREG_SYS1 0xA400, IPREG_SYS2 0xA500. Write = one burst of
  {addr_hi, addr_lo, data} starting at 0x7C. Read = write the address, CS high, wait >= 4 us, read 0x7E.
  Wait >= 4 us between accesses.
- REG_MISC2 0x7F: bit1 SOFT_RST (self-clearing), bit0 IREG_DONE (p109). Delay after reset: UNKNOWN.
  Measured: RESET_DONE (INT1_STATUS0 bit 7) appears about 1.06 ms after a soft reset.
- Register access is allowed 1 ms after power-up; accel start-up about 10 ms (p19, p21).
- External clock (pin 9 is wired to the 32.000 kHz oscillator X1, see `pinout.md`): RTC_CONFIG 0x26 bit5
  RTC_MODE = 1; it only works if ACCEL_LP_CLK_SEL = 1, which is bit 4 of SMC_CONTROL_0, IREG address 0xA258
  (p178; bit 3 is TEMP_DIS). CLKIN accepted range 20-40 kHz (p37). The datasheet does not say how pin 9 becomes
  CLKIN; the sequence that works on this board, taken from TDK's reference driver, is: IOC_PAD_SCENARIO_OVRD 0x31 =
  0x06; SIFS_I3C_STC_CFG 0xA268 bit 2 = 0; IPREG_SYS2_REG_123 0xA57B ACCEL_SRC_CTRL = 2; IPREG_SYS1_REG_166 0xA4A6
  GYRO_SRC_CTRL = 2; SMC_CONTROL_0 bit 4 = 1; RTC_CONFIG bit 5 = 1. Output data rate scales as CLKIN/32 kHz (TDK
  application note, not the datasheet): 200.000 Hz nominal with the 32.000 kHz X1.
- Measured output rate: internal oscillator 200.895 Hz for a nominal 200 Hz; external clock 200.000 Hz. On the
  external clock the rate is exact up to 3.2 kHz but 6388.7 Hz at the 6.4 kHz setting. The external-clock set-up
  also changes the ICM's filter path, so compare noise only with the filter path held equal.

## LSM6DSK320X (STMicroelectronics, IMU with high-g channel on SPI1)

Pins: see `pinout.md`, SPI1 / LSM6DSK320X.

- SPI: first byte bit7 = 1 for read (ST convention: RW bit first, then AD6:0), modes 0 and 3, address
  auto-increment on by default (CTRL3 bit2) (p22, p25, p67).
- WHO_AM_I 0x0F = 0x75 (p64).
- CTRL1 0x10: bits 6:4 OP_MODE_XL (000 high performance), bits 3:0 ODR_XL (0101 = 60 Hz, 0110 = 120 Hz,
  0111 = 240 Hz) (p65).
- CTRL2 0x11: same layout for the gyroscope (p66).
- CTRL6 0x15: bits 2:0 FS_G (001 = +-250, 010 = +-500, 011 = +-1000, 100 = +-2000, 101 = +-4000 dps). The
  reset value selects a reserved code, so FS_G must be written. Keep bit3 as the datasheet requires (p69).
- CTRL8 0x17: bits 1:0 FS_XL (00 = +-2 g, 01 = +-4 g, 10 = +-8 g, 11 = +-16 g) (p71).
- High-g channel: CTRL1_XL_HG 0x4E: bit7 XL_HG_REGOUT_EN, bits 5:3 ODR_XL_HG (011 = 480 Hz, 100 = 960 Hz),
  bits 2:0 FS_XL_HG (000 = +-32 g ... 100 = +-320 g) (p91). Field positions: verify on p91 before use.
  Table 21 (p33) allows the high-g channel with any low-g ODR in high-performance mode; lowest high-g ODR is 480 Hz.
- Interrupt routing: INT1_CTRL 0x0D bit0 = low-g accel data-ready, bit1 = gyro data-ready. INT2_CTRL 0x0E same
  bits. CTRL7 0x16 bit7 = high-g data-ready on INT1, bit6 = on INT2 (p63, p64, p70).
- CTRL4 0x13 bit1 DRDY_PULSED (verify bit position on p68): 1 = 65 us pulses, 0 = latched (default).
- IF_CFG 0x03: bit4 H_LACTIVE (0 = active high), bit3 PP_OD (0 = push-pull), bit2 SIM (0 = 4-wire), bit0
  I2C_I3C_disable; not reset by software reset (p58, Table 31).
- CTRL3 0x12: bit0 SW_RESET (self-clearing), bit7 BOOT (p67). Boot time is not in the datasheet; measured:
  SW_RESET self-clears in about 230 us.
- STATUS_REG 0x1E: bit0 XLDA, bit1 GDA, bit2 TDA, bit3 XLHGDA (p77).
- Outputs, little-endian 16-bit: temperature 0x20; gyro X 0x22, Y 0x24, Z 0x26; low-g accel X 0x28, Y 0x2A,
  Z 0x2C; high-g accel X 0x34, Y 0x36, Z 0x38 (p52-53, p78-84).
- Sensitivity (p12): low-g mg/LSB +-2 g 0.061, +-4 g 0.122, +-8 g 0.244, +-16 g 0.488. High-g mg/LSB +-32 g
  0.976, +-64 g 1.952, +-128 g 3.904, +-256 g 7.808, +-320 g 10.417. Gyro mdps/LSB +-250 8.75, +-500 17.5,
  +-1000 35, +-2000 70, +-4000 140.
- Temperature: degC = raw/256 + 25 (p16). Gyro turn-on 40 ms (p15).
- ODR accuracy in normal high-performance mode is not specified (the +-1 %/+-3 % figures on p13 are for
  high-accuracy-ODR mode). Measured: the output rate runs 1.8 % below nominal (the ODR setting is not recorded).
- High-g zero-g offset: +-250 mg typical (Table 3, p12).

## BMP581 (Bosch Sensortec, barometer on I2C1)

Pins: see `pinout.md`, I2C1 / BMP581 (7-bit address 0x46).

- CHIP_ID 0x01 = 0x50; REV_ID 0x02 = 0x32 (p51).
- STATUS 0x28: bit1 nvm_rdy (expect 1), bit2 nvm_err (expect 0) (p58, p18).
- INT_STATUS 0x27, clear-on-read: bit4 por, bit0 drdy. 0x10 right after power-up or soft reset (p58).
- OSR_CONFIG 0x36: bit6 press_en, bits 5:3 osr_p, bits 2:0 osr_t (p64).
- ODR_CONFIG 0x37: bit7 deep_dis, bits 6:2 odr (0x0F = 50 Hz, 0x11 = 40 Hz, 0x15 = 20 Hz, 0x17 = 10 Hz,
  0x1C = 1 Hz), bits 1:0 pwr_mode (00 standby, 01 normal). Write configuration in standby first, power mode
  last (p65, p18).
- Data, little-endian 24-bit signed: temperature 0x1D-0x1F, degC = raw/65536; pressure 0x20-0x22,
  Pa = raw/64 (p56-57).
- INT_CONFIG 0x14: bit0 mode (0 pulsed), bit1 polarity (1 active high), bit2 open-drain (0 push-pull), bit3
  enable, bits 7:4 pad drive; reset 0x35. INT_SOURCE 0x15 bit0 = data-ready enable (p52-54).
  Pulse length 90-121 us (p12).
- Soft reset: write 0xB6 to CMD 0x7E; ready after 2 ms; the write may not be acknowledged (p67, p11).
- Measured: after a reset the pending por flag holds INT high and no data-ready edges appear (pin high on
  2000 of 2000 samples) until INT_STATUS has been read once. Read INT_STATUS before enabling the interrupt.
- ODR code 0x00 = 240 Hz. pwr_mode 10 = forced (one measurement, then standby), 11 = continuous.
  OSR_EFF bit7 odr_is_valid reports whether the chip accepts the ODR for the chosen oversampling; it
  updates in standby. Measured: 240 Hz with 128x/128x oversampling is rejected.
- IIR filter: coefficients in DSP_IIR 0x31 (pressure in bits 5:3). The data registers hold unfiltered values
  unless DSP_CONFIG 0x30 bit3 (shdw_sel_iir_t) and bit5 (shdw_sel_iir_p) are set; DSP_CONFIG resets to 0x03.
  Not exercised on the board.
- Measured at 400 kHz I2C: one 6-byte read takes about 242 us. Normal mode at the 240 Hz setting gives
  238.4 Hz; continuous mode with 1x/1x oversampling gives 496 Hz; forced mode with 16x pressure and 2x
  temperature oversampling takes 11.9 ms from trigger to data-ready.

## BMM350 (Bosch Sensortec, magnetometer on I2C0, on the bench)

Pins: see `pinout.md`, Bench configuration (SDA GPIO12, SCL GPIO13, INT on GPIO4). Sources: datasheet BST-BMM350-DS001-27 (rev 1.27, Feb 2025), page numbers below; "vendor" = Bosch `BMM350_SensorAPI` v1.10.0, files `bmm350.c` and `bmm350_defs.h` (BSD-3-Clause), used only where the datasheet defers to the API (OTP layout, compensation, delays).

- Supplies: VDD 1.72-1.98 V, VDDIO 1.72-3.6 V (p8). How the bench breakout derives its supply from the 3.3 V pin: UNKNOWN.
- I2C 7-bit address: 0x14 with ADSEL low, 0x15 with ADSEL high; ADSEL is sampled at boot and must not float (p44-45). Measured: scan of 0x08-0x77 answered at 0x14 only.
- I2C: standard, fast and fast+ modes, no clock stretching (p44). Measured: 400 kHz works. Reset state is I2C; I3C is not used here.
- Read protocol: every read returns 2 dummy bytes first; an n-byte read is n+2 bytes, discard the first 2 (p46, 9.2.3; vendor `BMM350_DUMMY_BYTES` = 2). Register address auto-increments. A write is register, data pairs (p45). A burst read that starts at the sensortime, magnetic or temperature registers freezes their update until it ends; single reads can mix samples (p18).
- CHIP_ID 0x00 = 0x33, read-only (p29, p21). ERR_REG 0x02 bit0 pmu_cmd_error, clears on read (p29).
- Registers (p26-43): PAD_CTRL 0x03 (bits 2:0 drive, reset 0x07); PMU_CMD_AGGR_SET 0x04 (bits 5:4 avg, bits 3:0 odr, reset 0x14); PMU_CMD_AXIS_EN 0x05 (bit0 x, bit1 y, bit2 z, reset 0x07 on p31; the map table on p28 says 0x70, treated as a typo); PMU_CMD 0x06 (bits 3:0); PMU_CMD_STATUS_0 0x07; PMU_CMD_STATUS_1 0x08; I2C_WDT_SET 0x0A; INT_CTRL 0x2E; INT_CTRL_IBI 0x2F (I3C only); INT_STATUS 0x30; data 0x31-0x3C; SENSORTIME 0x3D-0x3F; OTP_CMD_REG 0x50; OTP_DATA_MSB 0x52; OTP_DATA_LSB 0x53; OTP_STATUS 0x55; TMR_SELFTEST_USER 0x60; CTRL_USER 0x61; CMD 0x7E.
- Soft reset: write 0xB6 to CMD 0x7E, then 0x00 (p22, p43); vendor `bmm350_init` writes only 0xB6, waits 24 ms (`BMM350_SOFT_RESET_DELAY`), and waits 3 ms before the reset (`BMM350_START_UP_TIME_FROM_POR`). Datasheet: OFF to suspend 2.5 ms typical, suspend to normal 70 ms typical (p8). The OTP is accessible again after every reset and the boot ends with 0x80 in OTP_CMD_REG (p14).
- OTP read (vendor `read_otp_word`; the datasheet gives only the register layout, p41-42): write OTP_CMD_REG = 0x20 | word (cmd 001 DIR_READ in bits 7:5, word 0-31 in bits 4:0); wait 300 us, poll OTP_STATUS until bit0 otp_cmd_done = 1 (bits 7:5 nonzero = error: 1 boot, 2 page read, 3 page program, 4 sign, 5 invalid command); read OTP_DATA_MSB 0x52 and OTP_DATA_LSB 0x53 (word = MSB << 8 | LSB). The vendor reads all 32 words, then writes 0x80 (PWR_OFF_OTP) to OTP_CMD_REG.
- OTP words used (vendor `update_mag_off_sens`; byte = signed 8 bit): 0x0D low = temperature offset (/5 K), high = temperature sensitivity (/512); 0x0E bits 11:0 = offset X (signed 12 bit), bits 15:12 = offset Y bits 11:8; 0x0F low = offset Y bits 7:0, bits 11:8 = offset Z bits 11:8; 0x10 low = offset Z bits 7:0, high = sensitivity X (/256); 0x11 low = sensitivity Y, high = sensitivity Z (/256); 0x12 / 0x13 / 0x14 low = TCO X / Y / Z (/32), high = TCS X / Y / Z (/16384); 0x15 low = cross X-Y, high = cross Y-X; 0x16 low = cross Z-X, high = cross Z-Y (all four /800); 0x18 = signed 16 bit, T0 = value/512 + 23 degC. Offsets are added to values already in uT.
- Magnetic reset (vendor `bmm350_magnetic_reset_and_wait`; datasheet p17 describes it): chip in suspend; write PMU_CMD = 0x07 (BR, bit reset, full recharge), wait 14 ms, PMU_CMD_STATUS_0 bits 7:5 must read 7; write PMU_CMD = 0x05 (FGR, flux-guide reset), wait 18 ms, bits 7:5 must read 5. The datasheet says the device resets itself after each boot and in normal and forced mode on every conversion; a manual reset is for field shocks and after long suspend (> 2 s). Reset can keep PMU_CMD_STATUS_0 bit0 busy for up to 8 ms (p22).
- Power modes (PMU_CMD, p32): 0x00 SUS, 0x01 NM, 0x02 UPD_OAE (apply odr/avg), 0x03 FM (forced, full CRST recharge), 0x04 FM_FAST (fast recharge, for >= 25 Hz trigger rates; the only way to reach 200 Hz), 0x05 FGR, 0x06 FGR_FAST, 0x07 BR, 0x08 BR_FAST. Suspend is the state after boot; forced mode starts only from suspend and returns to suspend when done; a forced request in normal mode, or during a running forced conversion, is ignored (ERR_REG pmu_cmd_error) (p15-16). In normal mode only SUS and UPD_OAE are accepted. If more than 0.1 s passes between init and the first forced trigger, use FM, not FM_FAST (p15). PMU_CMD_STATUS_0 (p33): bit0 busy, bit1 odr_ovwr, bit2 avg_ovwr, bit3 pwr_mode_is_normal, bit4 cmd_is_illegal, bits 7:5 last command value (3 bits, so BR_FAST reads 7).
- Vendor mode-change delays (`bmm350_defs.h`): normal to suspend 6 ms; suspend to normal 38 ms; suspend to forced 15 / 17 / 20 / 28 ms for avg 1 / 2 / 4 / 8, fast forced 4 / 5 / 9 / 16 ms; UPD_OAE 1 ms. Datasheet: forced conversion under 5 ms (avg 1, FM_FAST), up to 30 ms (avg 8, FM) (p15). Maximum forced trigger rate: 200 / 150 / 80 / 40 Hz for avg 1 / 2 / 4 / 8 with FM_FAST (p16).
- ODR codes (PMU_CMD_AGGR_SET bits 3:0, p30): 0x02 400 Hz, 0x03 200, 0x04 100, 0x05 50, 0x06 25, 0x07 12.5, 0x08 6.25, 0x09 3.125, 0x0A 1.5625 Hz. Typical accuracy +-2 % at room temperature, +-4 % over temperature (p9). avg codes (bits 5:4): 0 no averaging, 1 = 2 samples, 2 = 4, 3 = 8. Allowed combinations (p15 Table 5): 400 Hz avg 0 only; 200 Hz avg 0-1; 100 Hz avg 0-2; 50 Hz and below avg 0-3. The vendor lowers an excessive avg to the highest allowed. After writing odr/avg send UPD_OAE; the next odr/avg update must wait for a data-ready after the busy flag clears (p32). PMU_CMD_STATUS_1 0x08 reports the effective odr (bits 3:0) and avg (bits 5:4).
- Axis enable: PMU_CMD_AXIS_EN bits 0-2; only effective in suspend, ignored on the fly in normal mode; the vendor refuses to disable all three (p20). Registers of disabled axes keep stale data.
- Interrupt (INT_CTRL 0x2E, p35): bit0 int_mode (1 latched, 0 pulsed), bit1 int_pol (1 active high), bit2 int_od (1 push-pull, 0 open-drain), bit3 int_output_en (maps the interrupt to the pin; reserved on A samples), bit7 drdy_data_reg_en. Reset 0x00 = open-drain, active low, pulsed, disabled. Push-pull, active high, pulsed, data-ready on = 0x8E (vendor interrupt example expects the same). INT_STATUS 0x30 bit2 drdy_data_reg, clears on read; in pulsed mode it clears itself after 1.25 ms (p20). INT pulse width: UNKNOWN. Measured: INT_CTRL reads back 0x8E; one rising edge per sample, 200 edges for 200 samples at 100 Hz and 799 for 799 at about 400 Hz, none missed.
- Data (p37-40): X 0x31-0x33, Y 0x34-0x36, Z 0x37-0x39, temperature 0x3A-0x3C, each little-endian (XLSB, LSB, MSB), read as one 12-byte burst from 0x31. The datasheet describes 21 significant bits in a 24-bit register (p13); the vendor sign-extends all 24 bits. Reset value of every data byte 0x7F.
- Temperature: degC = raw * 0.000981282 - 25.49 (vendor), then corrected as below. Measured: about 25.2 degC on the bench.
- Conversion to uT (vendor `read_out_raw_data`, `update_default_coefiecents`): X, Y: raw * 0.007069979 uT/LSB; Z: raw * 0.007174964 uT/LSB; these are (1e6 / 2^20) / (gain products 14.55 * 19.46 for X/Y, 9.0 * 31.0 for Z, * (1/1.5) * 0.714607238769531). Temperature uses 1 / (0.00204 * (1/1.5) * 0.714607238769531 * 2^20) = 0.000981282.
- Compensation (vendor `bmm350_get_compensated_mag_xyz_temp_data`): T = (1 + t_sens) * (degC raw - 25.49) + t_offs. Per axis, B = raw_uT * (1 + sens) + offset + tco * (T - T0), then B = B / (1 + tcs * (T - T0)). Then with d = 1 - cross_y_x * cross_x_y: X = (Bx - cross_x_y * By) / d; Y = (By - cross_y_x * Bx) / d; Z = Bz + (Bx * (cross_y_x * cross_z_y - cross_z_x) - By * (cross_z_y - cross_x_y * cross_z_x)) / d. The macros BMM350_SENS_CORR_Y (0.01) and BMM350_TCS_CORR_Z (0.0001) exist in `bmm350_defs.h` but `bmm350.c` v1.10.0 never uses them.
- Range: +-2000 uT per axis, and the vector sum must stay below 2000 uT (p19). Noise 190 nT rms X/Y and 450 nT rms Z at 100 Hz, avg 2 (p10).
- Not used by the driver: PAD_CTRL drive, I2C watchdog, sensortime, self-test (p17, TMR_SELFTEST_USER), I3C, out-of-range detection (`bmm350_oor.c`).
- Measured on the bench: OTP of this unit: offsets -136 -137 -13 uT, sensitivity -0.0234 -0.0234 0.0039, T0 30.03 degC. Field about 67 uT with std 0.16 / 0.14 / 0.40 uT (x / y / z) at 100 Hz, avg 2, in the bench environment. Rate 100.68 Hz at the 100 Hz setting and 402.7 Hz at 400 Hz (+0.68 %, within the +-2 % typical). Read of 12 data bytes at 400 kHz I2C: about 440 us mean, 480 us worst. init (reset, OTP, magnetic reset, configuration) about 85 ms. Forced mode (FM, avg 2): 15.5 ms from the trigger write to the data-ready edge. Right after init PMU_CMD_STATUS_0 read 0x11 (busy and cmd_is_illegal set, power mode not yet normal) and 0x28 (normal, last command NM) two seconds later; the cause of the transient illegal flag is UNKNOWN.

## microSD (Molex 47219-2001 socket J11, native 4-bit SD)

Pins: see `pinout.md`, native SD.

- Bus: PIO + DMA native 4-bit SD, not SPI mode. 3.3 V signalling only.
- CMD and DAT0-3 have external 10k pull-ups; CLK has none and is driven through a 22 ohm series resistor.
- The socket has no card-detect switch and no GPIO is used for detect; firmware has to probe for the card.
- Measured, card on the bench: SanDisk 64 GB SDXC (CID manufacturer 0x03, OEM "SD", product "SK64G",
  revision 8.0); OCR 0xC0FF8000 (high capacity, 3.2-3.4 V); CSD version 2, 124735488 blocks of 512 bytes;
  ACMD41 ready after about 14 ms; busy after a single-block write about 1 ms.

## PSRAM (ISSI IS66WVS16M8FBLL, hand-fitted in socket U24; Rev A 01/2025)

Pins: see `pinout.md`, QMI CS1 / PSRAM.

- 2.7-3.6 V, 128 Mbit = 16 MiB, 24-bit address, two stacked 64 Mbit dies. A burst must not cross the die
  boundary. 1024-byte wrapped page.
- Powers up in SPI mode; wait >= 150 us after power-up. No recovery time is given for the 0x66/0x99 reset;
  150 us works. Measured: the reset kept the contents (64 KiB checked).
- Commands: 0x66 reset-enable then 0x99 reset (must be consecutive); 0x9F read ID (SPI: 24 address bits
  don't-care, no wait); 0x03 read (33 MHz in SPI mode); 0x0B fast read (8 wait cycles SPI, 4 QPI);
  0xEB quad IO read (6 wait cycles, 104 MHz); 0x02 write; 0x38 quad IO write (0 wait, 104 MHz);
  0x35 enter QPI; 0xF5 exit QPI.
- Read ID: manufacturer 0x9D, KGD 0x5D (0x55 = failed die), then 48-bit EID whose top three bits give the
  density (100 = 128 Mbit).
- CE# low time limit tCEM: 4 us up to 85 degC. CE# high time >= 1 clock. Max clock 104 MHz.
  Measured at 75 MHz: Read ID CE# low 1347 ns; window bursts end at 3.1 us with MAX_SELECT 7 (bound 3.85 us).
- Driver `crates/qmi-psram`, test `tools/run.sh psram_smoke`. At clk_sys 150 MHz: QPI, EBh read with 24 dummy
  bits, 38h write, CLKDIV 2 (75 MHz), RXDELAY 2, COOLDOWN 1, PAGEBREAK 1024, MAX_SELECT 7, MIN_DESELECT 2,
  SELECT_SETUP 1. Measured RXDELAY pass window: 1-4 at CLKDIV 2, 0-5 at CLKDIV 3.
- Measured, 1 MiB sequential: uncached alias write 20.8 MiB/s, read 24.4 MiB/s; cached alias write 7.9 MiB/s
  (write-allocate), read 25.2 MiB/s. The driver hands out the uncached alias 0x15000000.
- Measured with the driver compiled at opt-level 0 with debug assertions: direct-mode CE# low up to 13.4 us (over
  tCEM); every data check still passed. Compile the driver optimised in debug builds too.
- Differences from the APS6404L that `embassy_rp::psram` targets: manufacturer ID 0x9D instead of 0x0D,
  16 MiB instead of 8 MiB, 104 MHz instead of 144 MHz, tCEM 4 us instead of 8 us, QPI fast read at 84 MHz.
- Socket U24 is one SOIC-8 land for either this PSRAM, an APS6404L-SQN-SN (8 MB QSPI PSRAM) or a W25Q32JVSNIQ
  (32 Mbit QSPI NOR). The PCB is assembled with U24 empty.
- Programming the in-package flash resets the QMI configuration; `Psram::remap()` re-programs the window
  without a chip command (verified with the M1 reset simulated).
- The QMI registers used to reach it are under RP2350 below.

## Quectel LC76G (GNSS, UART1 and PPS on the bench)

Pins: see `pinout.md`, Bench configuration. Test: `tools/run.sh lc76g_smoke`; driver: `crates/lc76g`.

- Protocol specification V1.1 (link below). Bench module `LC76GABNR12A05S`, build 2025/09/25 10:10:11 (`$PQTMVERNO`,
  p100). 115200 baud 8N1 confirmed. Get commands answer `$PAIR001,<id>,0`, then the value (p39): `$PAIR051,1000*13`.
- PPS: `$PAIR752,<type>,<width_ms>` (p78): type 0 off, 1 after first fix, 2 3D fix only, 3 2D/3D fix, 4 always.
  Default type: UNKNOWN (the module pulsed at 1 Hz, 100 ms wide, with a fix; not read back).
- Default sentences (measured): GGA, GLL, GSA, GSV, RMC, VTG every fix; ZDA, GRS, GST, GNS off. The spec's "default 1"
  for all types (p41) is wrong here. `$PAIR062,-1` resets the rates (ZDA 1 -> 0 verified); `$PAIR062,-1,0` and `,-1,1`
  are rejected with `$PAIR001,062,4`.
- `$PAIR050,100` gives 10 Hz (p38), but here GGA, GLL, RMC, VTG, `$PQTMEPE` and (at rate 1) GSA and GSV all go at
  10 Hz; the spec says GSA/GSV stay at 1 Hz and VTG/GLL stop. That is 187 sentences, 11.6 kB per second: the whole
  capacity of 115200 baud (11.52 kB/s). Fixes drop (97 per 10 s) and commands are answered a second late or never.
  One epoch at the default rates is 19 sentences (4 GSA, 11 to 12 GSV). `Config::is_valid` therefore requires GSA, GSV
  and GRS off or at most once per second below 1000 ms; GSA/GSV at 10 (`with_fix_interval(100)`): 100 fixes per 10 s,
  ~65 sentences/s, replies in milliseconds. `init` sets 1000 ms first and the target interval last, so no step floods.
- 10 Hz fix times are not on a 100 ms grid: about 57 of 99 steps are exactly 100 ms, the rest 89 to 107 ms, rarely up
  to 172 ms. After a return to 1 Hz one step of 1277 to 1483 ms precedes the 1000 ms steps.
- Drone mode (`$PAIR080,5`): coordinates still have six decimals of minutes (`ddmm.mmmmmm,N,dddmm.mmmmmm,W`), not
  seven as the note on p51 says; the driver accepts both. `$PQTMEPE,2,1.224,1.253,2.415,1.752,2.983*57` follows each fix.
- Bench fix: 3D, 19 to 25 satellites used, HDOP 0.5; at rest the position moves 1 to 13 m within 10 s in drone mode.
- PPS (GPIO5, plain input): 1 s period to 5 ppm by the MCU timer; in about half the runs one period is 1 to 3 ms off
  (module or edge latency, undecided). `read_fix` returns the fix of a second 75 to 90 ms after the PPS edge.
- RP2350 at 150 MHz: framing one GGA 49 us, framing + parse 115 us (RMC 46 / 116 us; 209 / 174 before `Decimal::scaled`
  got a 64-bit path); `Lc76g<BufferedUart>` 440 bytes plus the buffers (RX 4096, TX 256). Bursts are about 1.4 kB, so
  use a buffered UART. After a probe reset the receive line can show a burst of framing errors (0 to 228 seen).
- `$PAIR513*3D` saves the module's settings to its flash. Send it only when a permanent change is intended.

## WS2812C-2020 status LED (D20)

Pins: see `pinout.md`, status LED.

- One LED, data on GPIO43 through a 100 ohm series resistor, supplied from V5_SYS, driven by a PIO program in
  the GPIO16-47 window.
- Fitted part: WS2812C-2020-V6, same pinout and pad drawing as the WS2812C-2020-V1 datasheet.
- Driver `crates/ws2812c`, test `tools/run.sh ws2812c_smoke`.
- Datasheet p4 limits: T0H 220-380 ns; T1H, T0L, T1L 580-1000 ns; reset low over 280 us; GRB, MSB first.
- SPI encoding of the driver: 0 = `1000`, 1 = `1100`, so the SPI clock must be 3.0 to 3.448 MHz (derived from
  the limits; nominal 3.2 MHz, which embassy-rp rounds to 3.125 MHz).
- Measured on the board, PIO on GPIO43 at 150 MHz (divider 47): T0H 306-320, T1H 626, T0L 940, T1L 620-633 ns;
  the 24 decoded bits equal the sent word.
- Measured on the board, SPI1 (TX GPIO43, SCK on the unconnected GPIO26) at 3.125 MHz: mode 1 or 3 is gapless
  (T0H 320, T1H 640, T0L 953-966, T1L 633-646 ns); modes 0 and 2 leave about 0.5 us between bytes and break the
  limits. embassy-rp's blocking SPI write waits for each byte and is not gapless either.
- Measured on the board: low time between two frames 318-356 us (PIO) and 345 us (SPI).
- The pad input reads the driven waveform from another PIO block, so the line is checked without extra hardware.
- Not verified in the driver's runs: the colours as seen by a person (the bring-up run was watched: red, green,
  blue, white).

## ADC channels

Pins: see `pinout.md`, ADC.

| Channel | GPIO | Signal | Conversion |
|---|---|---|---|
| ADC0 | 40 | VBUS sense | V(pin) = 0.6 x VBUS (R29 10k top, R30 15k bottom, 100n) |
| ADC1 | 41 | ESC current sense (J3 pad 1) | 1:1 through 1k R53 with 100n at the pin; source pad floating on the bench |
| ADC2 | 42 | VBAT sense | V(pin) = VBAT / 11 (R27 100k, R28 10k, 100n) |
| ADC4-7 | 44-47 | spare IO pads A44-A47 | no conversion recorded |
| ADC8 | - | die temperature | see RP2350 |

- ADC_AVDD is V3V3_ANA (TPS7A2033 LDO), the same rail as the sensors.
- Current scale, offset and output range of the ESC current output: UNKNOWN, need verification. An assumed
  12.75 mV/A sensitivity from a different ESC must not be reused.
- No other battery-monitor path exists (no I2C power monitor).

## RP2350 / RP2354B (RP2350.pdf)

Pins: see `pinout.md`, MCU and clock.

- ADC on the QFN-80: channels 0-7 = GPIO40-47, channel 8 = temperature sensor;
  T = 27 - (V - 0.706)/0.001721 (p1070, p1074).
- In-package flash: 2 MB Winbond W25Q16JV (p1331); JEDEC ID 0xEF 0x40 0x15 is unverified.
- Measured: the chip reports revision 3.
- QMI: base 0x400d0000. Direct mode: set DIRECT_CSR.EN, wait BUSY low, assert CS1N, push to DIRECT_TX, always
  drain DIRECT_RX. XIP (the running program's flash) is inaccessible while direct mode is enabled, so that code
  must execute from RAM with interrupts off. M1 format/command registers: RFMT/RCMD/WFMT/WCMD
  (p1236-1247). Writes through the window need CTRL.WRITABLE_M1. Verified: window 1 at 0x11000000 (cached), 0x15000000
  (uncached). Direct-mode FIFOs: faster via XIP_AUX 0x50500004/8 (p349). Cache clean by set/way: 0x1bffc001+8n (E11).

## Datasheets

| Device | URL |
|---|---|
| RP2350 / RP2354B | https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf |
| ICM-45686 | https://www.lcsc.com/datasheet/lcsc_datasheet_2411220643_TDK-InvenSense-ICM-45686_C22459454.pdf |
| LSM6DSK320X | https://www.st.com/resource/en/datasheet/lsm6dsk320x.pdf (DS15060 Rev 1, July 2026) |
| BMP581 | https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf |
| BMM350 datasheet (BST-BMM350-DS001-27) | https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf |
| BMM350 Bosch Sensor API (BSD-3-Clause) | https://github.com/boschsensortec/BMM350_SensorAPI |
| IS66WVS16M8FBLL PSRAM | https://cdn.sparkfun.com/assets/a/7/e/0/0/IS66WVS16M8FBLL-104NLI-COM-29556.pdf |
| APS6404L-SQN-SN PSRAM (socket option) | https://www.apmemory.com/tw/downloadFiles/0324112120g3605268 |
| W25Q32JVSNIQ NOR (socket option) | https://www.winbond.com/resource-files/W25Q32JV%20RevJ%2012242024%20Plus.pdf |
| Quectel LC76G protocol specification V1.1 | https://quectel.com/content/uploads/2024/02/Quectel_LC26GABLC76GLC86G_Series_GNSS_Protocol_Specification_V1.1.pdf |
| WS2812C-2020 | https://www.lcsc.com/datasheet/lcsc_datasheet_2202221130_Worldsemi-WS2812C-2020-V1_C2976072.pdf |
| X1 32.000 kHz oscillator HCI 1521H-32K000J18DTSL | https://www.lcsc.com/product-detail/crystal-oscillators_hci-1521h-32k000j18dtsl_C19674133.html |
| Molex 47219-2001 microSD socket | https://www.es.co.th/Schemetic/PDF/472192001.PDF |
