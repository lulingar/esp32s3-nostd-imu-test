#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{mem::MaybeUninit, str::FromStr};

use esp_backtrace as _;
use esp_hal::{
    system::SystemControl,
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    i2c::I2C,
    gpio::{Io, Output, Level},
    timer::OneShotTimer,
    rng::Rng,
    prelude::*,
};

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};

use esp_println::println;
use static_cell::make_static;

extern crate alloc;

use esp_wifi::wifi::{
    AuthMethod, WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState
};
use embassy_net::{tcp::TcpSocket, Config, Ipv4Address, Stack, StackResources};
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::{
        reason_codes::ReasonCode,
        publish_packet::QualityOfService,
    },
    utils::rng_generator::CountingRng,
};

use icm20948_async::{AccRange, AccDlp, AccUnit, GyrDlp, GyrRange, GyrUnit, IcmError, Icm20948};
use imu_fusion::{FusionMatrix, FusionVector};

mod analysis;
mod config;
mod imu_tracker;

use crate::config::FIRMWARE_CONFIG;
use imu_tracker::ImuTracker;
use analysis::Analysis;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}


#[main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    init_heap();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rng = Rng::new(peripherals.RNG);

    let mut flag = Output::new(io.pins.gpio5, Level::Low);
    flag.set_low();

    // Network setup start
    let seed = ((rng.random() as u64) << 32) | (rng.random() as u64);
    let timer = esp_hal::timer::PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None)
            .timer0
            .into(),
    );
    let init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        timer,
        rng,
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, peripherals.WIFI, WifiStaDevice).unwrap();

    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(
        &clocks,
        make_static!(
            [
             OneShotTimer::new(systimer.alarm0.into()),
             //OneShotTimer::new(systimer.alarm1.into()),
             ]
        )
    );

    // IMU bus start
    let sclk = io.pins.gpio8;
    let mosi = io.pins.gpio10;  // SDA on IMU board
    //let miso = io.pins.gpio7;    // SDO on IMU board
    //let cs = io.pins.gpio5;

    // Create and await IMU object
    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        mosi,
        sclk,
        400.kHz(),
        &clocks,
    );
    let imu_configured = Icm20948::new_i2c(i2c0, Delay)
    // Configure accelerometer
    .acc_range(AccRange::Gs8)
    .acc_dlp(AccDlp::Hz111)
    .acc_unit(AccUnit::Gs)
    // Configure gyroscope
    .gyr_range(GyrRange::Dps1000)
    .gyr_dlp(GyrDlp::Hz120)
    .gyr_unit(GyrUnit::Dps)
    // Final initialization
    .set_address(0x69);
    let imu_result = imu_configured.initialize_9dof().await;
    //spawner.must_spawn(imu_reader(i2c, out_imu_reading, sample_time))
    // Unpack IMU result safely and print error if necessary
    let mut imu = match imu_result {
        Ok(imu) => imu,
        Err(error) => {
            match error {
                IcmError::BusError(_)   => log::error!("IMU_READER : IMU encountered a communication bus error"),
                IcmError::ImuSetupError => log::error!("IMU_READER : IMU encountered an error during setup"),
                IcmError::MagSetupError => log::error!("IMU_READER : IMU encountered an error during mag setup")
            } panic!("Could not init IMU!");
        }
    };
    // Calibrate gyroscope offsets using 100 samples
    log::info!("IMU_READER : Reading gyroscopes, keep still");
    let _gyr_cal = imu.gyr_calibrate(100).await.is_ok();

    const IMU_SAMPLE_PERIOD: Duration = Duration::from_hz(200);
    const EVENT_TOPIC: &str = const_format::formatcp!("{}/event", FIRMWARE_CONFIG.mqtt_id);
    let acc_misalignment = FusionMatrix::identity();
    let acc_offset = FusionVector::zero();
    let acc_sensitivity = FusionVector::ones();
    let gyr_offset = FusionVector::zero();
    let mut tracker = ImuTracker::new(IMU_SAMPLE_PERIOD, Instant::now(), 1000.0f32,
                                      acc_misalignment, acc_sensitivity, acc_offset, gyr_offset);

    let mut analysis = Analysis::default();

    // Init network stack
    let stack = &*make_static!(Stack::new(
        wifi_interface,
        Config::dhcpv4(Default::default()),
        make_static!(StackResources::<3>::new()),
        seed
    ));

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(stack)).ok();

    let remote_endpoint = (
        Ipv4Address::from_str(FIRMWARE_CONFIG.mqtt_host).unwrap(),
        FIRMWARE_CONFIG.mqtt_port.parse::<u16>().unwrap()
    );
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    // Outer loop that maintains WiFi connectivity
    loop {
        log::info!("Bringing network link up...");
        while !stack.is_link_up() {
            Timer::after(Duration::from_millis(500)).await;
        }

        log::info!("Waiting to get IP address...");
        'ip: loop {
            if let Some(config) = stack.config_v4() {
                log::info!("Got IP: {}", config.address);
                break 'ip;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        // Inner loop that maintains connectivity to the MQTT broker
        'mqtt: loop {
            let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
            socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));


            log::info!("Connecting...");
            Timer::after(Duration::from_millis(500)).await;
            if let Err(e) = socket.connect(remote_endpoint).await {
                log::error!("connecting: {:?}", e);
                continue 'mqtt;
            }
            log::info!("Connected!");

            let mut config = ClientConfig::new(
                rust_mqtt::client::client_config::MqttVersion::MQTTv5,
                CountingRng(20000),
            );
            config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
            config.add_client_id(FIRMWARE_CONFIG.mqtt_id);
            config.add_username(FIRMWARE_CONFIG.mqtt_user);
            config.add_password(FIRMWARE_CONFIG.mqtt_pass);
            config.max_packet_size = 100;
            let mut recv_buffer = [0; 80];
            let mut write_buffer = [0; 80];
            let mut client = MqttClient::<_, 5, _>::new(
                socket,
                &mut write_buffer,
                80,
                &mut recv_buffer,
                80,
                config,
            );
            log::info!("Attempting broker connection...");
            'broker: while let Err(result) = client.connect_to_broker().await {
                if result == ReasonCode::Success {
                    log::info!("Connected!");
                    break 'broker;
                }
                else {
                    log::error!("Could not contact broker because {result}")
                }
                Timer::after(Duration::from_millis(500)).await;
            }
            log::info!("Connected to broker!");

            // Main loop: reading the sensor and sending movement detection data to the broker

            // moduli to keep a healthy load for the MQTT link
            const DETECTION_REPORT_FREQ: Duration = Duration::from_hz(8);
            const MOD_DETECTION: u32 = (DETECTION_REPORT_FREQ.as_ticks() / IMU_SAMPLE_PERIOD.as_ticks()) as u32;
            const MQTT_PING_PERIOD: Duration = Duration::from_secs(4);
            const MOD_MQTT_PING: u32 = (MQTT_PING_PERIOD.as_ticks() / IMU_SAMPLE_PERIOD.as_ticks()) as u32;
            log::info!("Mod_det {MOD_DETECTION}, Mod_mq {MOD_MQTT_PING}");
            let mut ticker = Ticker::every(IMU_SAMPLE_PERIOD);
            let mut id: u32 = 0;
            'sense: loop {
                ticker.next().await;
                id += 1;
                let should_send_sample = id % MOD_DETECTION == 0;
                // Adding 1 avoids both events coinciding, which would be redundant.
                let should_send_ping = (id + 1) % MOD_MQTT_PING == 0;

                let now = Instant::now();
                flag.set_high();
                match imu.read_9dof().await {
                    Ok(meas) => {
                        let acc = FusionVector::new(meas.acc.x, meas.acc.y, meas.acc.z);
                        let gyr = FusionVector::new(meas.gyr.x, meas.gyr.y, meas.gyr.z);

                        // Magnetometer axes are reflected along X axis, as per the datasheet
                        let mag = FusionVector::new(meas.mag.x, -meas.mag.y, -meas.mag.z);
                        tracker.update(now, acc, gyr, mag);
                        let new_direction = analysis.add_measurement(tracker.linear_accel);
                        flag.set_low();
                        if should_send_sample {
                            if let Some(dir) = new_direction {
                                let payload: [u8; 1] = [0x30 + dir.as_digit()];
                                if let Err(result) = client
                                    .send_message(
                                        EVENT_TOPIC,
                                        &payload,
                                        QualityOfService::QoS0,
                                        false,
                                    )
                                    .await {
                                    if result != ReasonCode::Success {
                                        log::error!("Could not publish because {result}; Restarting connection!");
                                        break 'mqtt;
                                    }
                                }
                                println!("{:02} {}", (id % 100), dir.as_char());
                            }
                        }
                    },
                    Err(e) => {
                        log::error!("Reading IMU {e:?}");
                        continue 'sense;
                    }
                }

                if should_send_ping {
                    if let Err(result) = client.send_ping().await {
                        if result != ReasonCode::Success {
                            log::error!("Could not send ping because {result}; Restarting connection!");
                            break 'mqtt;
                        }
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    log::info!("start connection task");
    log::info!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        if esp_wifi::wifi::get_wifi_state() == WifiState::StaConnected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
                ssid: FIRMWARE_CONFIG.wifi_ssid.parse().unwrap(),
                password: FIRMWARE_CONFIG.wifi_psk.parse().unwrap(),
                auth_method: AuthMethod::WPA2Personal,
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            log::info!("Starting wifi");
            controller.start().await.unwrap();
            log::info!("Wifi started!");
        }
        log::info!("About to connect...");

        match controller.connect().await {
            Ok(_) => log::info!("Wifi connected!"),
            Err(e) => {
                log::error!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}