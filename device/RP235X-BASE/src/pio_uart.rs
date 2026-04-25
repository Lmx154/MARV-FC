use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_rp::{Peri, PeripheralType};

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
) -> (PioUartTx<'d, PIO, TX_SM>, PioUartRx<'d, PIO, RX_SM>)
where
    PIO: Instance,
    TxPin: PioPin,
    RxPin: PioPin,
{
    let tx_program = PioUartTxProgram::new(common);
    let uart_tx = PioUartTx::new(baud, common, tx_sm, pins.tx, &tx_program);

    let rx_program = PioUartRxProgram::new(common);
    let uart_rx = PioUartRx::new(baud, common, rx_sm, pins.rx, &rx_program);

    (uart_tx, uart_rx)
}
