use crate::{
    bridge::bridge_thread,
    chunk_message_ids,
    cloud::{self, Crop, Point},
    ChunkHeader, HEADER_BYTES, MAX_CHUNKS,
};
use oden_plugin_rs::{
    async_com_channel_api::{
        async_com_channel_add_message_to_send_queue_raw, async_com_channel_flush_message_queue,
    },
    glam::{Mat4, Quat, Vec3, Vec4},
    log, GuiParams, InitParams, OdenPlugin, ShutdownParams, UpdateParams,
};
use oden_plugin_rs_derive::Properties;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Condvar, Mutex,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

const BYTES_PER_MBIT: f32 = 1e6 / 8.0;
const KBIT_PER_BYTE: f32 = 8.0 / 1000.0;
const KBIT_PER_MBIT: f32 = 1000.0;
const PERCENT: f32 = 100.0;

const COM_CHANNEL_MESSAGE_HEADER_BYTES: i32 = 10;
const MAX_TARGET_CHUNK_BYTES: f32 = 1100.0;
const MIN_TARGET_CHUNK_BYTES: f32 = 250.0;
const CHUNK_BUDGET_TIME_S: f32 = 0.08;
const INITIAL_BYTES_PER_POINT: f32 = 2.0;
const BUDGET_DECREASE_TIME_CONSTANT_S: f32 = 0.25;
const BUDGET_INCREASE_TIME_CONSTANT_S: f32 = 3.0;
const VOXEL_STEP: f32 = 1.25;
const REFINE_HEADROOM: f32 = 1.6;
const REFINE_INTERVAL: Duration = Duration::from_secs(1);
const MAX_VOXEL_SIZE_M: f32 = 1.0;
const MAX_FRAME_INTERVAL: Duration = Duration::from_secs(1);
const MAX_ANNOUNCED_FRAME_INTERVAL: Duration = Duration::from_millis(2500);
const WISH_LIFETIME: Duration = Duration::from_secs(2);
const MIN_BITRATE_MIN_FPS: f32 = 1.0;
const STATS_INTERVAL: Duration = Duration::from_secs(1);
const MAX_PROBED_CAMERA_STREAM: i32 = 16;
const RIG_PROBE_INTERVAL: Duration = Duration::from_secs(5);

const ODEN_FROM_ROS: Mat4 = Mat4::from_cols(
    Vec4::new(0.0, 0.0, -1.0, 0.0),
    Vec4::new(-1.0, 0.0, 0.0, 0.0),
    Vec4::new(0.0, 1.0, 0.0, 0.0),
    Vec4::W,
);

#[derive(Properties)]
pub struct StreamerState {
    #[property(default = 50.0)]
    max_range_m: f32,

    #[property(default = -1000.0)]
    min_height_m: f32,

    #[property(default = 1000.0)]
    max_height_m: f32,

    #[property(default = 0.05)]
    min_voxel_size_m: f32,

    #[property(default = 10.0)]
    min_fps: f32,

    #[property(default = 0.05)]
    min_bitrate_mbps: f32,

    #[property(default = 1.5)]
    max_bitrate_mbps: f32,

    budget_mbps: Option<f32>,
    is_at_min_bitrate: bool,
    video_fraction: f32,
    rig: Rig,
    worker: Worker,
}

impl OdenPlugin for StreamerState {
    fn init(_api: &InitParams) -> Self {
        StreamerState {
            max_range_m: Default::default(),
            min_height_m: Default::default(),
            max_height_m: Default::default(),
            min_voxel_size_m: Default::default(),
            min_fps: Default::default(),
            min_bitrate_mbps: Default::default(),
            max_bitrate_mbps: Default::default(),
            budget_mbps: None,
            is_at_min_bitrate: false,
            video_fraction: 0.0,
            rig: Rig::default(),
            worker: Worker::start(),
        }
    }

    fn shutdown(self, _api: &ShutdownParams) {}

    fn update(&mut self, api: &UpdateParams) {
        self.video_fraction = video_fraction(api);
        let video_share_mbps = self.max_bitrate_mbps * self.video_fraction;
        self.is_at_min_bitrate = video_share_mbps < self.min_bitrate_mbps;
        let target_budget_mbps = video_share_mbps
            .max(self.min_bitrate_mbps)
            .min(self.max_bitrate_mbps)
            .max(0.0);
        let budget_mbps = self.budget_mbps.map_or(target_budget_mbps, |budget_mbps| {
            smoothed_budget_mbps(budget_mbps, target_budget_mbps, api.dt() as f32)
        });
        self.budget_mbps = Some(budget_mbps);

        *self.worker.shared.settings.lock().unwrap() = EncodeSettings {
            crop: Crop {
                max_range_m: self.max_range_m,
                min_height_m: self.min_height_m,
                max_height_m: self.max_height_m,
            },
            min_voxel_size_m: self.min_voxel_size_m,
            min_fps: if self.is_at_min_bitrate {
                self.min_fps.min(MIN_BITRATE_MIN_FPS)
            } else {
                self.min_fps
            },
            budget_bytes_per_s: budget_mbps * BYTES_PER_MBIT,
            rig_from_lidar: self.rig.rig_from_lidar(api),
        };
    }

    fn gui(&mut self, api: &GuiParams) {
        if let Some(budget_mbps) = self.budget_mbps {
            let video_state = format!(", video at {:.0}%", self.video_fraction * PERCENT);
            let floor_state = if self.is_at_min_bitrate {
                ", at Min Bitrate"
            } else {
                ""
            };
            api.label(&format!(
                "Budget: {:.0} kbit/s{video_state}{floor_state}",
                budget_mbps * KBIT_PER_MBIT
            ));
        }

        let stats: Vec<(usize, SendStats)> = self
            .worker
            .shared
            .sources
            .lock()
            .unwrap()
            .iter()
            .enumerate()
            .filter(|(_, source)| source.current_wish().is_some())
            .map(|(index, source)| (index, source.stats))
            .collect();
        for (source, stats) in &stats {
            api.label(&format!(
                "Source {source}: {:.0} kbit/s, {:.1} fps, {} points, \
                 {} bytes in {} chunks, voxel {:.3} m",
                stats.kbit_per_s,
                stats.fps,
                stats.points,
                stats.bytes,
                stats.chunks,
                stats.voxel_size
            ));
        }
    }
}

#[derive(Default)]
struct Rig {
    parent: Option<String>,
    stream: Option<i32>,
    probed_at: Option<Instant>,
}

impl Rig {
    fn rig_from_lidar(&mut self, api: &UpdateParams) -> Mat4 {
        let is_probe_due = self
            .probed_at
            .is_none_or(|probed_at| probed_at.elapsed() >= RIG_PROBE_INTERVAL);
        if is_probe_due {
            self.probe(api);
        }

        let lidar_world = Mat4::from(api.world_matrix()) * ODEN_FROM_ROS;
        let Some(parent) = self.parent.as_deref() else {
            return lidar_world;
        };

        let calibration = self
            .stream
            .and_then(|stream| api.camera_calibration(parent, stream));
        if calibration.is_none() {
            self.stream = None;
        }
        let global = calibration.map_or(Mat4::IDENTITY, |calibration| {
            let [w, x, y, z] = calibration.globalRotation;
            Mat4::from_rotation_translation(
                Quat::from_xyzw(x, y, z, w),
                Vec3::from_array(calibration.globalPosition),
            )
        });
        let parent_world = api
            .entity_world_matrix(parent)
            .map_or(Mat4::IDENTITY, Mat4::from);

        global * parent_world.inverse() * lidar_world
    }

    fn probe(&mut self, api: &UpdateParams) {
        let is_first_probe = self.probed_at.is_none();
        self.probed_at = Some(Instant::now());

        let parent = api
            .instance_uuid()
            .and_then(|uuid| api.parent_uuid(&uuid.to_string()))
            .map(|uuid| uuid.to_string());
        if parent != self.parent {
            self.parent = parent;
            self.stream = None;
        }

        let Some(parent) = self.parent.as_deref() else {
            return;
        };
        if self.stream.is_none() {
            self.stream = (0..=MAX_PROBED_CAMERA_STREAM)
                .find(|stream| api.camera_calibration(parent, *stream).is_some());
        }
        if self.stream.is_none() && is_first_probe {
            log::warn!(
                "No camera stream 0-{MAX_PROBED_CAMERA_STREAM} in the parent, \
                 no rig calibration"
            );
        }
    }
}

fn video_fraction(api: &UpdateParams) -> f32 {
    if unsafe { (*api.inner).getStreamerBitrateMbps }.is_none() {
        return 0.0;
    }
    let sent_mbps = api.streamer_bitrate_mbps().filter(|mbps| *mbps > 0.0);
    let configured_mbps = api.target_bitrate().ok().filter(|mbps| *mbps > 0.0);

    sent_mbps
        .zip(configured_mbps)
        .map_or(0.0, |(sent, configured)| (sent / configured).min(1.0))
}

/// Keeps the plugin's own traffic and the regulator from pushing each other around.
fn smoothed_budget_mbps(budget_mbps: f32, target_budget_mbps: f32, dt_s: f32) -> f32 {
    if target_budget_mbps <= 0.0 {
        return 0.0;
    }

    let time_constant_s = if target_budget_mbps < budget_mbps {
        BUDGET_DECREASE_TIME_CONSTANT_S
    } else {
        BUDGET_INCREASE_TIME_CONSTANT_S
    };

    budget_mbps + (target_budget_mbps - budget_mbps) * (1.0 - (-dt_s / time_constant_s).exp())
}

fn expected_frame_interval(
    input_period: Duration,
    credit_bytes: f32,
    budget_bytes_per_s: f32,
) -> Duration {
    let input_period_s = input_period.as_secs_f32();
    let refill_s = (-credit_bytes / budget_bytes_per_s).max(0.0);
    let whole_input_periods = (refill_s / input_period_s).ceil().max(1.0);

    Duration::try_from_secs_f32(whole_input_periods * input_period_s)
        .unwrap_or(MAX_ANNOUNCED_FRAME_INTERVAL)
        .min(MAX_ANNOUNCED_FRAME_INTERVAL)
}

#[derive(Clone, Copy, Default)]
struct EncodeSettings {
    crop: Crop,
    min_voxel_size_m: f32,
    min_fps: f32,
    budget_bytes_per_s: f32,
    rig_from_lidar: Mat4,
}

#[derive(Clone, Copy, Default)]
struct SendStats {
    kbit_per_s: f32,
    fps: f32,
    points: i32,
    bytes: i32,
    chunks: i32,
    voxel_size: f32,
}

pub struct InputFrame {
    pub points: Vec<Point>,
    pub input_period: Duration,
}

#[derive(Clone, Copy)]
struct Wish {
    mbps: f32,
    made_at: Instant,
}

#[derive(Default)]
struct Source {
    newest_frame: Option<InputFrame>,
    wish: Option<Wish>,
    stats: SendStats,
}

impl Source {
    fn current_wish(&self) -> Option<Wish> {
        self.wish
            .filter(|wish| wish.made_at.elapsed() < WISH_LIFETIME)
    }

    fn current_wish_mbps(&self) -> f32 {
        self.current_wish().map_or(0.0, |wish| wish.mbps)
    }
}

#[derive(Default)]
pub struct Shared {
    pub run: AtomicBool,
    sources: Mutex<Vec<Source>>,
    new_frame: Condvar,
    settings: Mutex<EncodeSettings>,
    encode_threads: Mutex<Vec<JoinHandle<()>>>,
}

impl Shared {
    fn settings_with_share(&self, source: usize) -> EncodeSettings {
        let settings = *self.settings.lock().unwrap();
        let sources = self.sources.lock().unwrap();
        let total_wish_mbps: f32 = sources.iter().map(Source::current_wish_mbps).sum();
        let share = if total_wish_mbps > 0.0 {
            sources[source].current_wish_mbps() / total_wish_mbps
        } else {
            0.0
        };

        EncodeSettings {
            budget_bytes_per_s: settings.budget_bytes_per_s * share,
            ..settings
        }
    }

    pub fn put_frame(self: &Arc<Self>, source: u8, frame: InputFrame) {
        let mut sources = self.sources.lock().unwrap();
        for new_source in (0..=source).skip(sources.len()) {
            sources.push(Source::default());
            let shared = self.clone();
            self.encode_threads
                .lock()
                .unwrap()
                .push(thread::spawn(move || encode_thread(&shared, new_source)));
        }

        sources[usize::from(source)].newest_frame = Some(frame);
        self.new_frame.notify_all();
    }

    pub fn sleep_while_running(&self, duration: Duration) -> bool {
        let guard = self.sources.lock().unwrap();
        let _guard = self
            .new_frame
            .wait_timeout_while(guard, duration, |_| self.run.load(Ordering::Relaxed))
            .unwrap();
        self.run.load(Ordering::Relaxed)
    }

    fn wait_for_frame(&self, source: usize) -> Option<InputFrame> {
        let mut sources = self.sources.lock().unwrap();

        loop {
            if !self.run.load(Ordering::Relaxed) {
                return None;
            }
            if let Some(input_frame) = sources[source].newest_frame.take() {
                return Some(input_frame);
            }
            sources = self.new_frame.wait(sources).unwrap();
        }
    }
}

struct Worker {
    shared: Arc<Shared>,
    bridge: Option<JoinHandle<()>>,
}

impl Worker {
    fn start() -> Self {
        let shared = Arc::new(Shared {
            run: AtomicBool::new(true),
            ..Default::default()
        });

        let bridge = thread::spawn({
            let shared = shared.clone();
            move || bridge_thread(&shared)
        });

        Self {
            shared,
            bridge: Some(bridge),
        }
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        {
            let _guard = self.shared.sources.lock().unwrap();
            self.shared.run.store(false, Ordering::Relaxed);
        }
        self.shared.new_frame.notify_all();

        let mut has_panicked = self
            .bridge
            .take()
            .is_some_and(|bridge| bridge.join().is_err());
        let encode_threads = std::mem::take(&mut *self.shared.encode_threads.lock().unwrap());
        for thread in encode_threads {
            has_panicked |= thread.join().is_err();
        }
        if has_panicked {
            log::error!("A point cloud Streamer thread panicked");
        }
    }
}

fn send_all(messages: &[Vec<u8>], message_ids: &[String]) {
    for (message, message_id) in messages.iter().zip(message_ids) {
        async_com_channel_add_message_to_send_queue_raw(message_id, message);
    }
    async_com_channel_flush_message_queue();
}

fn encode_thread(shared: &Shared, source: u8) {
    let message_ids = chunk_message_ids();
    let mut sender = RateLimitedSender::new(source);

    while let Some(frame) = shared.wait_for_frame(usize::from(source)) {
        sender.publish_wish(&frame, shared);
        let settings = shared.settings_with_share(usize::from(source));
        sender.process(&frame, &settings, &message_ids, shared);
    }
}

struct RateLimitedSender {
    source: u8,
    has_encode_failed: bool,
    voxel_size: f32,
    bytes_per_point: f32,
    credit_bytes: f32,
    frame: u8,
    last_frame: Instant,
    last_voxel_change: Instant,
    window_start: Instant,
    window_bytes: i32,
    window_frames: i32,
    last_sent: SendStats,
}

impl RateLimitedSender {
    fn new(source: u8) -> Self {
        Self {
            source,
            has_encode_failed: false,
            voxel_size: 0.0,
            bytes_per_point: INITIAL_BYTES_PER_POINT,
            credit_bytes: 0.0,
            frame: 0,
            last_frame: Instant::now(),
            last_voxel_change: Instant::now(),
            window_start: Instant::now(),
            window_bytes: 0,
            window_frames: 0,
            last_sent: SendStats::default(),
        }
    }

    fn publish_wish(&self, frame: &InputFrame, shared: &Shared) {
        let full_rate_bytes_per_s =
            frame.points.len() as f32 * self.bytes_per_point / frame.input_period.as_secs_f32();
        shared.sources.lock().unwrap()[usize::from(self.source)].wish = Some(Wish {
            mbps: full_rate_bytes_per_s / BYTES_PER_MBIT,
            made_at: Instant::now(),
        });
    }

    fn process(
        &mut self,
        frame: &InputFrame,
        settings: &EncodeSettings,
        message_ids: &[String],
        shared: &Shared,
    ) {
        let since_last_frame = self.last_frame.elapsed().min(MAX_FRAME_INTERVAL);
        self.last_frame = Instant::now();

        let accrued_bytes = settings.budget_bytes_per_s * since_last_frame.as_secs_f32();
        self.credit_bytes = (self.credit_bytes + accrued_bytes).min(accrued_bytes);
        if settings.budget_bytes_per_s > 0.0 && self.credit_bytes >= 0.0 {
            self.send(frame, settings, message_ids);
        }

        self.publish_stats(shared);
    }

    fn send(&mut self, frame: &InputFrame, settings: &EncodeSettings, message_ids: &[String]) {
        self.voxel_size = self
            .voxel_size
            .min(MAX_VOXEL_SIZE_M)
            .max(settings.min_voxel_size_m)
            .max(settings.crop.smallest_voxel_size_m());

        let (sectors, lidar_points) =
            cloud::voxel_centers_by_azimuth(&frame.points, self.voxel_size, &settings.crop);
        let rig_points: Vec<Point> = lidar_points
            .into_iter()
            .map(|point| {
                settings
                    .rig_from_lidar
                    .transform_point3(Vec3::from_array(point))
                    .to_array()
            })
            .collect();

        let target_chunk_bytes = (settings.budget_bytes_per_s * CHUNK_BUDGET_TIME_S)
            .clamp(MIN_TARGET_CHUNK_BYTES, MAX_TARGET_CHUNK_BYTES);
        let points_per_chunk = (target_chunk_bytes / self.bytes_per_point) as usize;
        let chunks =
            match cloud::encode_chunks(&sectors, &rig_points, self.voxel_size, points_per_chunk) {
                Ok(chunks) => chunks,
                Err(error) => {
                    if !self.has_encode_failed {
                        log::error!("Draco encode failed: {error}");
                    }
                    self.has_encode_failed = true;
                    return;
                }
            };
        self.has_encode_failed = false;

        let draco_bytes: usize = chunks.iter().map(|chunk| chunk.draco.len()).sum();
        let frame_bytes = draco_bytes as i32
            + chunks.len() as i32 * (HEADER_BYTES as i32 + COM_CHANNEL_MESSAGE_HEADER_BYTES);

        if !rig_points.is_empty() {
            self.bytes_per_point = draco_bytes as f32 / rig_points.len() as f32;
        }

        let voxel_size = self.voxel_size;
        let achievable_fps = settings.budget_bytes_per_s / frame_bytes.max(1) as f32;
        self.adapt_voxel_size(achievable_fps, chunks.len(), settings);

        let is_coarsest = voxel_size >= MAX_VOXEL_SIZE_M;
        if achievable_fps < settings.min_fps && !is_coarsest {
            return;
        }

        self.frame = self.frame.wrapping_add(1);
        let frame_interval = expected_frame_interval(
            frame.input_period,
            self.credit_bytes - frame_bytes as f32,
            settings.budget_bytes_per_s,
        );

        let messages: Vec<Vec<u8>> = chunks
            .iter()
            .map(|chunk| {
                ChunkHeader {
                    source: self.source,
                    frame: self.frame,
                    first_sector: chunk.first_sector,
                    last_sector: chunk.last_sector,
                    frame_interval,
                }
                .message(&chunk.draco)
            })
            .collect();

        send_all(&messages, message_ids);
        let sent_bytes: i32 = messages
            .iter()
            .map(|message| message.len() as i32 + COM_CHANNEL_MESSAGE_HEADER_BYTES)
            .sum();
        self.credit_bytes -= sent_bytes as f32;

        self.window_bytes += sent_bytes;
        self.window_frames += 1;
        self.last_sent = SendStats {
            points: rig_points.len() as i32,
            bytes: sent_bytes,
            chunks: messages.len() as i32,
            voxel_size,
            ..Default::default()
        };
    }

    fn adapt_voxel_size(&mut self, achievable_fps: f32, chunks: usize, settings: &EncodeSettings) {
        if achievable_fps < settings.min_fps || chunks >= MAX_CHUNKS {
            let coarsening = (settings.min_fps / achievable_fps).sqrt().max(VOXEL_STEP);
            self.voxel_size = (self.voxel_size * coarsening).min(MAX_VOXEL_SIZE_M);
            self.last_voxel_change = Instant::now();
        } else if achievable_fps > settings.min_fps * REFINE_HEADROOM
            && (chunks as f32) * REFINE_HEADROOM < MAX_CHUNKS as f32
            && self.last_voxel_change.elapsed() >= REFINE_INTERVAL
        {
            self.voxel_size /= VOXEL_STEP;
            self.last_voxel_change = Instant::now();
        }
    }

    fn publish_stats(&mut self, shared: &Shared) {
        let window = self.window_start.elapsed();
        if window < STATS_INTERVAL {
            return;
        }

        shared.sources.lock().unwrap()[usize::from(self.source)].stats = SendStats {
            kbit_per_s: self.window_bytes as f32 * KBIT_PER_BYTE / window.as_secs_f32(),
            fps: self.window_frames as f32 / window.as_secs_f32(),
            ..self.last_sent
        };

        self.window_start = Instant::now();
        self.window_bytes = 0;
        self.window_frames = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expected_frame_interval_is_whole_input_periods_up_to_the_cap() {
        let input = Duration::from_millis(125);

        assert_eq!(expected_frame_interval(input, 50_000.0, 100_000.0), input);
        assert_eq!(
            expected_frame_interval(input, -25_000.0, 100_000.0),
            Duration::from_millis(250)
        );
        assert_eq!(
            expected_frame_interval(input, -30_000.0, 100_000.0),
            Duration::from_millis(375)
        );
        assert_eq!(
            expected_frame_interval(input, -25_000.0, 0.0),
            MAX_ANNOUNCED_FRAME_INTERVAL
        );
        assert_eq!(expected_frame_interval(input, 0.0, 0.0), input);
        assert_eq!(
            expected_frame_interval(Duration::from_secs(30), -1.0, 100_000.0),
            MAX_ANNOUNCED_FRAME_INTERVAL
        );
    }
}
