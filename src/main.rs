#![no_std]
#![no_main]

use core::{fmt::Write, borrow::BorrowMut};

use nb::block;
use panic_halt as _;

use cortex_m_rt::entry;
use slcan::{parse_slcan_cmd, SlcanCmdSetupType};
use stm32f1xx_hal::{pac, pac::USART1, pac::CAN1, pac::interrupt, prelude::*, serial::{Config, Serial, Rx, self}, can::Can, dma};
use bxcan::{self, filter::Mask32};

use crate::slcan::slcan_responses::*;

mod slcan;

static mut UART_RX: Option<Rx<USART1>> = None;
static mut CAN1: Option<bxcan::Can<Can<CAN1>>> = None;

static mut rx_overrun_error: bool = false;

const BUFFER_LEN: usize = 32;
const NUM_BUFFERS: usize = 16;

#[derive(Default)]
struct UartRxBuffer {
    data: [u8; BUFFER_LEN],
    len: usize,
    ready: bool
}

#[derive(Default)]
struct UartBuffers {
    buffers: [UartRxBuffer; NUM_BUFFERS]
}

static mut UART_BUFFERS: Option<UartBuffers> = None;


#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let _cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific&mut  peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();


    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Clock configuration - run at 72MHz (Max for STM32F103), GD32F103 can run up to 96MHz
    rcc.cfgr = rcc.cfgr.use_hse(8.MHz()).sysclk(72.MHz()).hclk(72.MHz()).adcclk(12.MHz());
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Take ownership of GPIO multiplexing & DMA
    let mut afio = dp.AFIO.constrain();
    let dma_channels = dp.DMA1.split();

    // Take ownership of GPIO
    let mut gpioa = dp.GPIOA.split();
    // let mut gpiob = p.GPIOB.split();

    // USART1 pin setup
    let uart_tx_pin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let uart_rx_pin = gpioa.pa10;

    // Initialize USART1
    let serial = Serial::usart1(
        dp.USART1,
        (uart_tx_pin, uart_rx_pin),
        &mut afio.mapr,
        Config::default().baudrate(115200.bps()),
        clocks,
    );

    // Split serial into send/receive objects, this allows each one to be separately owned
    let (mut uart_tx, uart_rx) = serial.split();

    // Set up DMA channel for transmitting. This allows asynchronous uart transmission without interaction from code.
    // let mut uart_tx = uart_tx.with_dma(dma_channels.4);

    // An interrupt-free block is needed to safely access these global variables.
    // TODO: Look into better ways to do this (lazy_static?)
    cortex_m::interrupt::free(|_| unsafe {
        UART_RX.replace(uart_rx);
        UART_BUFFERS = Some(UartBuffers{..Default::default()});
    });

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);

    // Initialize/configure CAN peripheral
    let can1 = Can::new(dp.CAN1, dp.USB);

    // Initialize CAN pins
    let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
    can1.assign_pins((tx, rx), &mut afio.mapr);

    // We will only have ownership of can1 when the can interface is closed
    let mut can_instance = Some(can1);
    
    // Open can interface, taking ownership of the can instance.
    fn can_open(can: Can<CAN1>, _setup: &SlcanCmdSetupType) {
        // TODO Actually use setup
        // APB1 (PCLK1): 36MHz, Bit rate: 250kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        let mut can = bxcan::Can::builder(can)
                .set_bit_timing(0x001c0008)
                .set_loopback(true) // CAN Loopback for debugging porpoises
                .leave_disabled();
        
        // Configure filters so that all CAN frames can be received.
        {
            let mut filters = can.modify_filters();
            filters.enable_bank(0, Mask32::accept_all());
        } // Filters dropped at the end of scope. This exits initialization mode
    
        block!(can.enable_non_blocking()).unwrap();
        can.enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::USB_LP_CAN_RX0);
        }

        // Interrupt free block to set global variable "safely"
        cortex_m::interrupt::free(|_| unsafe {
            CAN1.replace(can);
        });
    }

    // (tries to) close the CAN interface and return the can instance used to create it.
    fn can_close() -> Option<Can<CAN1>> {
        // Try to get the interface. If it doesn't exist (or is already closed),
        // CAN1 will be None and this function will do nothing and return None.
        let interface = unsafe { CAN1.take() }?;
        Some(interface.free())
    }

    // Enable UART interrupts
    // Enabling interrupts requires some unsafe operations on global registers
    unsafe {
        UART_RX.as_mut().unwrap().listen();
        pac::NVIC::unmask(pac::Interrupt::USART1);
    }



    // Stores which buffer we're reading from
    let mut uart_buffer_idx: usize = 0;
    let mut latest_can_setup: Option<SlcanCmdSetupType> = None;
    let mut timestamp_enabled = false;
    loop {
        let mut buf: &mut UartRxBuffer = unsafe { &mut UART_BUFFERS.as_mut().unwrap().buffers[uart_buffer_idx] };
        if buf.ready {
            led.toggle();
            let data_to_parse = &buf.data[0..buf.len];

            buf.ready = false;
            buf.len = 0;
            uart_buffer_idx = next_buf_idx(uart_buffer_idx);

            let command = parse_slcan_cmd(data_to_parse);
            
            if let Some(cmd) = command {
                match cmd {
                    slcan::SlcanCmd::Setup(s) => {
                        if let Some(c) = can_instance.take() {
                            can_open(c, &s);
                            latest_can_setup.replace(s);
                            uart_transmit(&mut uart_tx, &SLCAN_SUCCESS);
                        } else {
                            uart_transmit(&mut uart_tx, &SLCAN_FAILURE);
                        }
                    },
                    slcan::SlcanCmd::Open => {
                        // if let Some(ref setup) = ... will borrow setup instead of taking it.
                        if let Some(ref setup) = latest_can_setup {
                            if let Some(c) = can_instance.take() {
                                can_open(c, &setup);
                                uart_transmit(&mut uart_tx, &SLCAN_SUCCESS);
                            } else {
                                uart_transmit(&mut uart_tx, &SLCAN_FAILURE);
                            }
                        } else {
                            uart_transmit(&mut uart_tx, &SLCAN_FAILURE);
                        }
                    },
                    slcan::SlcanCmd::Close =>{
                        if let Some(inst) = can_close(){
                            can_instance.replace(inst);
                            uart_transmit(&mut uart_tx, &SLCAN_SUCCESS);
                        } else {
                            uart_transmit(&mut uart_tx, &SLCAN_FAILURE);
                        }
                    },
                    slcan::SlcanCmd::CanFrame(frame) => {
                        if let Some(c) = unsafe{ CAN1.as_mut() } {
                            let res = block!(c.transmit(&frame));
                            if let Ok(_) = res {
                                uart_transmit(&mut uart_tx, &SLCAN_TX_SUCCESS);
                            } else {
                                uart_transmit(&mut uart_tx, &SLCAN_FAILURE);
                            }
                        } else {
                            uart_transmit(&mut uart_tx, &SLCAN_FAILURE);
                        }
                    },
                    slcan::SlcanCmd::ReadStatus => {
                        if let Some(c) = unsafe { CAN1.as_mut() } {
                            uart_transmit(&mut uart_tx, &slcan_status_flags(false, false, false));
                        } else {
                            uart_transmit(&mut uart_tx, &SLCAN_FAILURE);
                        }
                    },
                    slcan::SlcanCmd::SetAcceptanceCode(_) => { uart_transmit(&mut uart_tx, &SLCAN_FAILURE); },
                    slcan::SlcanCmd::SetAcceptanceMask(_) => { uart_transmit(&mut uart_tx, &SLCAN_FAILURE); },
                    slcan::SlcanCmd::GetVersion => { uart_transmit(&mut uart_tx, &SLCAN_VERSION); },
                    slcan::SlcanCmd::GetSerialNumber => { uart_transmit(&mut uart_tx, &SLCAN_SERIAL_NO); },
                    slcan::SlcanCmd::SetTimestamp(enabled) => {
                        timestamp_enabled = enabled;
                        uart_transmit(&mut uart_tx, &SLCAN_SUCCESS);
                    },
                }
            }
        }
    }
}

fn next_buf_idx(buf_idx: usize) -> usize { (buf_idx + 1) % NUM_BUFFERS }

// Interrupt functions must be unsafe
#[interrupt]
unsafe fn USART1() {
    // Stores which buffer we're writing to.
    // By creating a static mut variable in an interrupt context, we create state which belongs only to the interrupt.
    static mut UART_BUFFER_IDX: usize = 0;

    // An interrupt is (typically) executed in an interrupt-free context (unless another, higher priority interrupt is allowed to take over)
    cortex_m::interrupt::free(|_| {
        if let Some(rx) = UART_RX.as_mut() {
            if let Ok(word) = rx.read() {
                let idx: usize = *UART_BUFFER_IDX;
                let mut buf: &mut UartRxBuffer = &mut UART_BUFFERS.as_mut().unwrap().buffers[idx];
                
                buf.data[buf.len] = word;
                buf.len += 1;
                
                const ASCII_CR: u8 = 13;
                if buf.len > BUFFER_LEN || word == ASCII_CR {
                    buf.ready = true;
                    *UART_BUFFER_IDX = next_buf_idx(*UART_BUFFER_IDX);
                }
            }
        }
    })
}

// CAN RX interrupt
#[interrupt]
unsafe fn USB_LP_CAN_RX0() {
    cortex_m::interrupt::free(|_| {
        if let Some(can) = CAN1.as_mut() {
            let res = can.receive();
        }
    });
}

// These functions take ownership of uart1_dma while transmitting, then release it back when done.
fn uart_transmit(uart: &mut serial::Tx<USART1>, buf: &[u8]) {
    for word in buf {
        block!(uart.write(*word)).unwrap();
    }
}
