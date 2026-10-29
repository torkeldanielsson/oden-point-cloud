use crate::streamer::{InputFrame, Shared};
use oden_plugin_rs::log;
use std::{
    io::{self, ErrorKind, Read},
    net::{Ipv4Addr, TcpListener, TcpStream},
    sync::{atomic::Ordering, Arc},
    thread,
    time::{Duration, Instant},
};

const BRIDGE_PORT: u16 = 29571;
const FRAME_HEADER_BYTES: usize = 5;
const POINT_BYTES: usize = 12;
const MAX_BRIDGE_POINTS: usize = 4_000_000;
const ACCEPT_POLL_INTERVAL: Duration = Duration::from_millis(50);
const READ_POLL_INTERVAL: Duration = Duration::from_millis(100);
const FAST_BIND_RETRIES: i32 = 3;
const FAST_BIND_RETRY_INTERVAL: Duration = Duration::from_secs(1);
const SLOW_BIND_RETRY_INTERVAL: Duration = Duration::from_secs(60);
const INPUT_PERIOD_SMOOTHING: f32 = 0.1;
const MIN_INPUT_PERIOD: Duration = Duration::from_millis(10);
const MAX_INPUT_PERIOD: Duration = Duration::from_secs(1);

pub fn bridge_thread(shared: &Arc<Shared>) {
    let Some(listener) = listen(shared) else {
        return;
    };

    let mut last_accept_error = None;
    while shared.run.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _)) => {
                last_accept_error = None;
                log::info!("Point cloud bridge connected");
                if let Err(error) = receive_frames(stream, shared) {
                    if shared.run.load(Ordering::Relaxed) {
                        log::info!("Point cloud bridge disconnected: {error}");
                    }
                }
            }
            Err(error) if error.kind() == ErrorKind::WouldBlock => {
                thread::sleep(ACCEPT_POLL_INTERVAL);
            }
            Err(error) => {
                if last_accept_error != Some(error.kind()) {
                    log::error!("Point cloud bridge accept failed: {error}");
                    last_accept_error = Some(error.kind());
                }
                thread::sleep(ACCEPT_POLL_INTERVAL);
            }
        }
    }
}

fn listen(shared: &Shared) -> Option<TcpListener> {
    let mut attempt = 0;

    loop {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, BRIDGE_PORT)).and_then(|listener| {
            listener.set_nonblocking(true)?;
            Ok(listener)
        });

        match listener {
            Ok(listener) => return Some(listener),
            Err(error) => log::error!(
                "Could not listen for the point cloud bridge on port {BRIDGE_PORT}: {error}"
            ),
        }

        let retry_interval = if attempt < FAST_BIND_RETRIES {
            FAST_BIND_RETRY_INTERVAL
        } else {
            SLOW_BIND_RETRY_INTERVAL
        };
        attempt += 1;

        if !shared.sleep_while_running(retry_interval) {
            return None;
        }
    }
}

#[derive(Clone, Copy, Default)]
struct Arrivals {
    previous: Option<Instant>,
    period: Option<Duration>,
}

impl Arrivals {
    fn smoothed_period(&mut self, arrival: Instant) -> Duration {
        if let Some(previous) = self.previous {
            let gap = (arrival - previous).clamp(MIN_INPUT_PERIOD, MAX_INPUT_PERIOD);
            self.period = Some(self.period.map_or(gap, |period| {
                period.mul_f32(1.0 - INPUT_PERIOD_SMOOTHING) + gap.mul_f32(INPUT_PERIOD_SMOOTHING)
            }));
        }
        self.previous = Some(arrival);

        self.period.unwrap_or(MAX_INPUT_PERIOD)
    }
}

fn receive_frames(mut stream: TcpStream, shared: &Arc<Shared>) -> io::Result<()> {
    stream.set_nonblocking(false)?;
    stream.set_read_timeout(Some(READ_POLL_INTERVAL))?;

    let mut payload = Vec::new();
    let mut arrivals: Vec<Arrivals> = Vec::new();

    loop {
        let mut frame_header = [0; FRAME_HEADER_BYTES];
        read_full(&mut stream, &mut frame_header, shared)?;
        let [source, count @ ..] = frame_header;

        let point_count = u32::from_le_bytes(count) as usize;
        if point_count > MAX_BRIDGE_POINTS {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("frame of {point_count} points"),
            ));
        }

        payload.resize(point_count * POINT_BYTES, 0);
        read_full(&mut stream, &mut payload, shared)?;

        let points = payload
            .as_chunks::<4>()
            .0
            .as_chunks::<3>()
            .0
            .iter()
            .map(|point| point.map(f32::from_le_bytes))
            .collect();

        let index = usize::from(source);
        if arrivals.len() <= index {
            arrivals.resize(index + 1, Arrivals::default());
        }
        let input_period = arrivals[index].smoothed_period(Instant::now());

        shared.put_frame(
            source,
            InputFrame {
                points,
                input_period,
            },
        );
    }
}

fn read_full(stream: &mut TcpStream, buffer: &mut [u8], shared: &Shared) -> io::Result<()> {
    let mut filled = 0;

    while filled < buffer.len() {
        if !shared.run.load(Ordering::Relaxed) {
            return Err(ErrorKind::Interrupted.into());
        }

        match stream.read(&mut buffer[filled..]) {
            Ok(0) => return Err(ErrorKind::UnexpectedEof.into()),
            Ok(read) => filled += read,
            Err(error) if matches!(error.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(error) => return Err(error),
        }
    }

    Ok(())
}
