use oden_plugin_rs::{register, RegisterType};
use player::PlayerState;
use std::time::Duration;
use streamer::StreamerState;

mod bridge;
mod cloud;
mod player;
mod streamer;

const MAX_CHUNKS: usize = 64;
const MAX_MESSAGE_BYTES: usize = 1300;
const HEADER_BYTES: usize = 8;

fn chunk_message_ids() -> Vec<String> {
    (0..MAX_CHUNKS)
        .map(|chunk| format!("point_cloud_{chunk}"))
        .collect()
}

#[derive(Clone, Copy)]
struct ChunkHeader {
    source: u8,
    frame: u8,
    first_sector: u16,
    last_sector: u16,
    frame_interval: Duration,
}

impl ChunkHeader {
    fn message(&self, draco: &[u8]) -> Vec<u8> {
        let frame_interval_ms = u16::try_from(self.frame_interval.as_millis()).unwrap_or(u16::MAX);

        let mut message = Vec::with_capacity(HEADER_BYTES + draco.len());
        message.push(self.source);
        message.push(self.frame);
        message.extend_from_slice(&self.first_sector.to_le_bytes());
        message.extend_from_slice(&self.last_sector.to_le_bytes());
        message.extend_from_slice(&frame_interval_ms.to_le_bytes());
        message.extend_from_slice(draco);
        message
    }

    fn parse(message: &[u8]) -> Option<(Self, &[u8])> {
        let (header, draco) = message.split_first_chunk::<HEADER_BYTES>()?;
        let [source, frame, first_0, first_1, last_0, last_1, interval_0, interval_1] = *header;

        let header = Self {
            source,
            frame,
            first_sector: u16::from_le_bytes([first_0, first_1]),
            last_sector: u16::from_le_bytes([last_0, last_1]),
            frame_interval: Duration::from_millis(
                u16::from_le_bytes([interval_0, interval_1]).into(),
            ),
        };

        Some((header, draco))
    }
}

register! {
    {
        "Point Cloud Player",
        "8c5dfd89-7513-4edb-bd7f-97587e265f79",
        PlayerState,
        RegisterType::Entity,
    },
    {
        "Point Cloud Streamer",
        "c24b0e07-5107-4970-9931-21c9c9581df5",
        StreamerState,
        RegisterType::Entity,
    }
}
