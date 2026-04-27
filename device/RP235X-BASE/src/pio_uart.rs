use embassy_rp::gpio::Level;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, FifoJoin, Instance, LoadedProgram, PioPin,
    ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::uart::{PioUartTx, PioUartTxProgram};
use embassy_rp::{Peri, PeripheralType};
use fixed::traits::ToFixed;

pub struct PioUartPins<Tx, Rx>
where
    Tx: PeripheralType + 'static,
    Rx: PeripheralType + 'static,
{
    pub tx: Peri<'static, Tx>,
    pub rx: Peri<'static, Rx>,
}

pub fn new_duplex_pio_uart<'d, PIO, TxPin, RxPin, const TX_SM: usize, const RX_SM: usize>(
    common: &mut Common<'d, PIO>,
    tx_sm: StateMachine<'d, PIO, TX_SM>,
    rx_sm: StateMachine<'d, PIO, RX_SM>,
    pins: PioUartPins<TxPin, RxPin>,
    baud: u32,
) -> (
    PioUartTx<'d, PIO, TX_SM>,
    PioUartRxNoStopCheck<'d, PIO, RX_SM>,
)
where
    PIO: Instance,
    TxPin: PioPin,
    RxPin: PioPin,
{
    let tx_program = PioUartTxProgram::new(common);
    let uart_tx = PioUartTx::new(baud, common, tx_sm, pins.tx, &tx_program);

    let rx_program = PioUartRxNoStopCheckProgram::new(common);
    let uart_rx = PioUartRxNoStopCheck::new(baud, common, rx_sm, pins.rx, &rx_program);

    (uart_tx, uart_rx)
}

pub struct PioUartRxNoStopCheckProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioUartRxNoStopCheckProgram<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = embassy_rp::pio::program::pio_asm!(
            r#"
                ; 8n1 UART RX variant that avoids JMP PIN for RP235x high GPIOs.
                ; IN pin 0 is mapped to the UART RX pin.

                start:
                    wait 0 pin 0
                    set x, 7 [10]
                rx_bitloop:
                    in pins, 1
                    jmp x-- rx_bitloop [6]
                    in null 24
                    push
            "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

pub struct PioUartRxNoStopCheck<'d, PIO: Instance, const SM: usize> {
    sm_rx: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> PioUartRxNoStopCheck<'d, PIO, SM> {
    pub fn new(
        baud: u32,
        common: &mut Common<'d, PIO>,
        mut sm_rx: StateMachine<'d, PIO, SM>,
        rx_pin: Peri<'d, impl PioPin>,
        program: &PioUartRxNoStopCheckProgram<'d, PIO>,
    ) -> Self {
        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[]);

        let rx_pin = common.make_pio_pin(rx_pin);
        sm_rx.set_pins(Level::High, &[&rx_pin]);
        cfg.set_in_pins(&[&rx_pin]);
        sm_rx.set_pin_dirs(PioDirection::In, &[&rx_pin]);

        cfg.clock_divider = (embassy_rp::clocks::clk_sys_freq() / (8 * baud)).to_fixed();
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::RxOnly;
        sm_rx.set_config(&cfg);
        sm_rx.set_enable(true);

        Self { sm_rx }
    }

    pub async fn read_u8(&mut self) -> u8 {
        self.sm_rx.rx().wait_pull().await as u8
    }
}
