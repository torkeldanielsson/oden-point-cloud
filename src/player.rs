use crate::{
    chunk_message_ids,
    cloud::{self, Point},
    ChunkHeader, MAX_MESSAGE_BYTES,
};
use oden_plugin_rs::{
    async_com_channel_api::async_com_channel_receive_var_size,
    glam::{Mat4, Vec3},
    log, math, AsyncComChannelReceiveError, GuiParams, InitParams, OdenPlugin, ShutdownParams,
    UpdateParams,
};
use oden_plugin_rs_derive::Properties;
use std::{
    panic,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

const RECEIVE_TIMEOUT: Duration = Duration::from_millis(50);
const FRAME_INTERVALS_KEPT: u32 = 2;
const MIN_CHUNK_LIFETIME: Duration = Duration::from_secs(1);
const MAX_CHUNK_LIFETIME: Duration = Duration::from_secs(5);
const MAX_LATE_FRAMES: i8 = 2;
const MIN_DEPTH_M: f32 = 0.05;
const MIN_HEIGHT_M: f32 = -2.0;
const MAX_HEIGHT_M: f32 = 3.0;
const MAX_HUE: f32 = 0.8;
const SATURATION: f32 = 0.8;
const BRIGHTNESS: f32 = 0.8;

#[derive(Properties)]
pub struct PlayerState {
    #[property(default = 3.0)]
    point_size_px: f32,

    receiver: ChunkReceiver,
    points: Vec<Point>,
    colors: Vec<math::Vec4>,
    triangle_positions: Vec<math::Vec3>,
    triangle_colors: Vec<math::Vec4>,
}

impl OdenPlugin for PlayerState {
    fn init(_api: &InitParams) -> Self {
        PlayerState {
            point_size_px: Default::default(),
            receiver: ChunkReceiver::start(),
            points: Vec::new(),
            colors: Vec::new(),
            triangle_positions: Vec::new(),
            triangle_colors: Vec::new(),
        }
    }

    fn shutdown(self, _api: &ShutdownParams) {}

    fn update(&mut self, api: &UpdateParams) {
        self.receiver
            .collect_points(&mut self.points, &mut self.colors);
        if self.points.is_empty() {
            return;
        }

        let world = api.world_matrix();
        let local_from_world = Mat4::from(world).inverse();
        let camera = api.camera_forward_ray();
        let eye = local_from_world.transform_point3(camera.origin.into());
        let forward = local_from_world
            .transform_vector3(camera.direction.into())
            .normalize_or_zero();
        let right = forward.cross(Vec3::Y).try_normalize().unwrap_or(Vec3::X);
        let up = right.cross(forward);
        let viewport = api.viewport_info();
        let half_fov = api.fps_camera_state().fov.to_radians() / 2.0;
        let radius_per_depth = half_fov.tan() * self.point_size_px * viewport.content_scale.x()
            / viewport.width_px.max(1) as f32;

        self.triangle_positions.clear();
        self.triangle_colors.clear();

        for (point, color) in self.points.iter().zip(&self.colors) {
            let point = Vec3::from_array(*point);
            let depth = (point - eye).dot(forward);
            if depth < MIN_DEPTH_M {
                continue;
            }

            let up = up * depth * radius_per_depth;
            let right = right * depth * radius_per_depth;
            let [top_left, top_right, bottom_left, bottom_right] = [
                point + up - right,
                point + up + right,
                point - up - right,
                point - up + right,
            ];
            self.triangle_positions.extend(
                [
                    top_left,
                    bottom_left,
                    top_right,
                    top_right,
                    bottom_left,
                    bottom_right,
                ]
                .map(math::Vec3::from),
            );
            self.triangle_colors.extend([*color; 6]);
        }

        api.im_draw_push_matrix(world);
        api.im_draw_set_use_z_buffer(false);
        api.im_draw_add_triangles(&mut self.triangle_positions, &mut self.triangle_colors);
        api.im_draw_set_use_z_buffer(true);
        api.im_draw_pop_matrix();
    }

    fn gui(&mut self, api: &GuiParams) {
        api.label(&format!("Points: {}", self.points.len()));
    }
}

fn height_color(height: f32) -> math::Vec4 {
    let height_fraction = ((height - MIN_HEIGHT_M) / (MAX_HEIGHT_M - MIN_HEIGHT_M)).clamp(0.0, 1.0);
    let rgb = rgb_from_hue(smoothstep(height_fraction) * MAX_HUE);

    (Vec3::ONE.lerp(rgb, SATURATION) * BRIGHTNESS)
        .extend(1.0)
        .into()
}

fn rgb_from_hue(hue: f32) -> Vec3 {
    let rgb = (((Vec3::new(0.0, 4.0, 2.0) + hue * 6.0) % 6.0 - 3.0).abs() - 1.0)
        .clamp(Vec3::ZERO, Vec3::ONE);
    Vec3::from_array(rgb.to_array().map(smoothstep))
}

fn smoothstep(x: f32) -> f32 {
    x * x * (3.0 - 2.0 * x)
}

struct ShownChunk {
    header: ChunkHeader,
    expires: Instant,
    points: Vec<Point>,
    colors: Vec<math::Vec4>,
}

impl ShownChunk {
    fn overlaps(&self, header: &ChunkHeader) -> bool {
        self.header.source == header.source
            && self.header.first_sector <= header.last_sector
            && header.first_sector <= self.header.last_sector
    }

    fn frames_ahead_of(&self, header: &ChunkHeader) -> i8 {
        self.header.frame.wrapping_sub(header.frame) as i8
    }
}

fn keep_newest_per_sector(chunks: &mut Vec<ShownChunk>, chunk: ShownChunk) {
    let header = chunk.header;
    let is_same_source = |shown: &ShownChunk| shown.header.source == header.source;

    let has_source_restarted = chunks
        .iter()
        .any(|shown| is_same_source(shown) && shown.frames_ahead_of(&header) > MAX_LATE_FRAMES);
    if has_source_restarted {
        chunks.retain(|shown| !is_same_source(shown));
    }

    let is_outdated = chunks.iter().any(|shown| {
        let frames_ahead = shown.frames_ahead_of(&header);
        is_same_source(shown) && (frames_ahead > 1 || (shown.overlaps(&header) && frames_ahead > 0))
    });
    if is_outdated {
        return;
    }

    chunks.retain(|shown| {
        !is_same_source(shown)
            || (shown.frames_ahead_of(&header) >= -1
                && (!shown.overlaps(&header) || shown.header.frame == header.frame))
    });
    chunks.push(chunk);
}

struct ChunkReceiver {
    run: Arc<AtomicBool>,
    chunks: Arc<Mutex<Vec<ShownChunk>>>,
    thread: Option<JoinHandle<()>>,
}

impl ChunkReceiver {
    fn start() -> Self {
        let run = Arc::new(AtomicBool::new(true));
        let chunks = Arc::new(Mutex::new(Vec::new()));

        let thread = Some(thread::spawn({
            let run = run.clone();
            let chunks = chunks.clone();
            move || receive_thread(&run, &chunks)
        }));

        Self {
            run,
            chunks,
            thread,
        }
    }

    fn collect_points(&self, points: &mut Vec<Point>, colors: &mut Vec<math::Vec4>) {
        let now = Instant::now();
        let mut chunks = self.chunks.lock().unwrap();
        chunks.retain(|chunk| chunk.expires > now);

        points.clear();
        colors.clear();
        for chunk in chunks.iter() {
            points.extend_from_slice(&chunk.points);
            colors.extend_from_slice(&chunk.colors);
        }
    }
}

impl Drop for ChunkReceiver {
    fn drop(&mut self) {
        self.run.store(false, Ordering::Relaxed);

        if let Some(thread) = self.thread.take() {
            if thread.join().is_err() {
                log::error!("The point cloud Player thread panicked");
            }
        }
    }
}

fn receive_thread(run: &AtomicBool, chunks: &Mutex<Vec<ShownChunk>>) {
    let message_ids = chunk_message_ids();
    let message_ids: Vec<&str> = message_ids.iter().map(String::as_str).collect();
    let mut buffer = [0; MAX_MESSAGE_BYTES];

    while run.load(Ordering::Relaxed) {
        match async_com_channel_receive_var_size(&message_ids, &mut buffer, RECEIVE_TIMEOUT) {
            Ok((_, size)) => {
                if let Some(chunk) = decode_chunk(&buffer[..size]) {
                    keep_newest_per_sector(&mut chunks.lock().unwrap(), chunk);
                }
            }
            Err(AsyncComChannelReceiveError::OdenAsyncComChannelReceiveErrorUnknown) => {
                thread::sleep(RECEIVE_TIMEOUT);
            }
            Err(_) => {}
        }
    }
}

fn decode_chunk(message: &[u8]) -> Option<ShownChunk> {
    let (header, draco) = ChunkHeader::parse(message)?;

    match panic::catch_unwind(|| cloud::decode(draco)) {
        Ok(Ok(points)) => Some(ShownChunk {
            header,
            expires: Instant::now()
                + (header.frame_interval * FRAME_INTERVALS_KEPT)
                    .clamp(MIN_CHUNK_LIFETIME, MAX_CHUNK_LIFETIME),
            colors: points.iter().map(|[_, y, _]| height_color(*y)).collect(),
            points,
        }),
        Ok(Err(_)) => None,
        Err(_) => {
            log::error!("Point cloud chunk decoder panicked");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chunk(source: u8, frame: u8, first_sector: u16, last_sector: u16) -> ShownChunk {
        ShownChunk {
            header: ChunkHeader {
                source,
                frame,
                first_sector,
                last_sector,
                frame_interval: Duration::ZERO,
            },
            expires: Instant::now(),
            points: Vec::new(),
            colors: Vec::new(),
        }
    }

    fn shown(chunks: &[ShownChunk]) -> Vec<(u8, u8, u16)> {
        chunks
            .iter()
            .map(|chunk| {
                (
                    chunk.header.source,
                    chunk.header.frame,
                    chunk.header.first_sector,
                )
            })
            .collect()
    }

    #[test]
    fn newer_frames_replace_overlapping_sectors_drop_older_frames_and_follow_restarts() {
        let mut chunks = Vec::new();
        keep_newest_per_sector(&mut chunks, chunk(0, 255, 0, 10));
        keep_newest_per_sector(&mut chunks, chunk(0, 255, 10, 20));
        keep_newest_per_sector(&mut chunks, chunk(0, 255, 21, 30));
        keep_newest_per_sector(&mut chunks, chunk(1, 7, 0, 30));
        assert_eq!(
            shown(&chunks),
            [(0, 255, 0), (0, 255, 10), (0, 255, 21), (1, 7, 0)]
        );

        keep_newest_per_sector(&mut chunks, chunk(0, 0, 5, 15));
        assert_eq!(shown(&chunks), [(0, 255, 21), (1, 7, 0), (0, 0, 5)]);

        keep_newest_per_sector(&mut chunks, chunk(0, 255, 0, 4));
        assert_eq!(
            shown(&chunks),
            [(0, 255, 21), (1, 7, 0), (0, 0, 5), (0, 255, 0)]
        );

        keep_newest_per_sector(&mut chunks, chunk(0, 255, 14, 25));
        assert_eq!(
            shown(&chunks),
            [(0, 255, 21), (1, 7, 0), (0, 0, 5), (0, 255, 0)]
        );

        keep_newest_per_sector(&mut chunks, chunk(0, 1, 0, 3));
        assert_eq!(shown(&chunks), [(1, 7, 0), (0, 0, 5), (0, 1, 0)]);

        keep_newest_per_sector(&mut chunks, chunk(0, 255, 40, 50));
        assert_eq!(shown(&chunks), [(1, 7, 0), (0, 0, 5), (0, 1, 0)]);

        keep_newest_per_sector(&mut chunks, chunk(1, 2, 20, 30));
        assert_eq!(shown(&chunks), [(0, 0, 5), (0, 1, 0), (1, 2, 20)]);
    }
}
