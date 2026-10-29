use crate::{HEADER_BYTES, MAX_CHUNKS, MAX_MESSAGE_BYTES};
use draco_core::{
    DataType, DecodeLimits, DecoderBuffer, DracoError, EncoderBuffer, EncoderOptions, ErrorKind,
    GeometryAttributeType, PointAttribute, PointCloud, PointCloudDecoder, PointCloudEncoder,
};
use std::f32::consts::TAU;

const MAX_DRACO_BYTES: usize = MAX_MESSAGE_BYTES - HEADER_BYTES;
const CELL_OFFSET: i32 = 1 << 15;
const AZIMUTH_SECTORS: f32 = 4096.0;
const MAX_QUANTIZATION_BITS: i32 = 30;
const ENCODING_SPEED: i32 = 7;
const MAX_CHUNK_POINTS: usize = 4096;
const MAX_DECODED_POINTS: u64 = 2 * MAX_CHUNK_POINTS as u64;
const MAX_DECODED_BYTES: u64 = 1 << 20;
const HEADER_FLAGS_OFFSET: usize = 9;
const METADATA_FLAG: u16 = 0x8000;

pub type Point = [f32; 3];

#[derive(Clone, Copy, Default)]
pub struct Crop {
    pub max_range_m: f32,
    pub min_height_m: f32,
    pub max_height_m: f32,
}

impl Crop {
    pub fn smallest_voxel_size_m(&self) -> f32 {
        2.0 * self.max_range_m / f32::from(u16::MAX)
    }

    fn keeps(&self, [x, y, z]: &Point) -> bool {
        let squared_range = x * x + y * y + z * z;
        squared_range > 0.0
            && squared_range <= self.max_range_m * self.max_range_m
            && (self.min_height_m..=self.max_height_m).contains(z)
    }
}

pub fn voxel_centers_by_azimuth(
    points: &[Point],
    voxel_size: f32,
    crop: &Crop,
) -> (Vec<u16>, Vec<Point>) {
    let mut keys: Vec<u64> = points
        .iter()
        .filter(|point| crop.keeps(point))
        .filter_map(|point| voxel_key(point, voxel_size))
        .collect();
    keys.sort_unstable();
    keys.dedup();

    keys.into_iter()
        .map(|key| {
            let center = [
                cell_center(key >> 32, voxel_size),
                cell_center(key >> 16, voxel_size),
                cell_center(key, voxel_size),
            ];
            ((key >> 48) as u16, center)
        })
        .unzip()
}

fn voxel_key(point: &Point, voxel_size: f32) -> Option<u64> {
    let cell = |value: f32| {
        u16::try_from(((value / voxel_size).floor() as i32).saturating_add(CELL_OFFSET)).ok()
    };
    let [x, y, z] = [cell(point[0])?, cell(point[1])?, cell(point[2])?].map(u64::from);

    let azimuth = cell_center(y, voxel_size).atan2(cell_center(x, voxel_size));
    let sector = ((azimuth / TAU + 0.5) * AZIMUTH_SECTORS) as u64;

    Some((sector << 48) | (x << 32) | (y << 16) | z)
}

fn cell_center(key: u64, voxel_size: f32) -> f32 {
    let cell = i32::from(key as u16) - CELL_OFFSET;
    (cell as f32 + 0.5) * voxel_size
}

pub struct EncodedChunk {
    pub first_sector: u16,
    pub last_sector: u16,
    pub draco: Vec<u8>,
}

pub fn encode_chunks(
    sectors: &[u16],
    points: &[Point],
    voxel_size: f32,
    points_per_chunk: usize,
) -> Result<Vec<EncodedChunk>, DracoError> {
    let points_per_chunk = points_per_chunk.clamp(1, MAX_CHUNK_POINTS);
    let mut chunks = Vec::new();

    for start in (0..points.len()).step_by(points_per_chunk) {
        let end = (start + points_per_chunk).min(points.len());
        encode_fitting(
            &sectors[start..end],
            &points[start..end],
            voxel_size,
            &mut chunks,
        )?;
    }

    Ok(chunks)
}

fn encode_fitting(
    sectors: &[u16],
    points: &[Point],
    voxel_size: f32,
    chunks: &mut Vec<EncodedChunk>,
) -> Result<(), DracoError> {
    if chunks.len() >= MAX_CHUNKS {
        return Ok(());
    }

    let draco = encode(points, voxel_size)?;
    if draco.len() <= MAX_DRACO_BYTES || points.len() == 1 {
        chunks.push(EncodedChunk {
            first_sector: sectors[0],
            last_sector: sectors[sectors.len() - 1],
            draco,
        });
        Ok(())
    } else {
        let half = points.len() / 2;
        encode_fitting(&sectors[..half], &points[..half], voxel_size, chunks)?;
        encode_fitting(&sectors[half..], &points[half..], voxel_size, chunks)
    }
}

fn encode(points: &[Point], voxel_size: f32) -> Result<Vec<u8>, DracoError> {
    let mut position = PointAttribute::new();
    position.init(
        GeometryAttributeType::Position,
        3,
        DataType::Float32,
        false,
        points.len(),
    );
    position
        .buffer_mut()
        .update_f32s_le(0, points.as_flattened());

    let mut cloud = PointCloud::new();
    cloud.add_attribute(position);

    let mut options = EncoderOptions::new();
    options.set_attribute_int(
        0,
        "quantization_bits",
        quantization_bits(points, voxel_size),
    );
    options.set_global_int("encoding_speed", ENCODING_SPEED);

    let mut encoder = PointCloudEncoder::new();
    encoder.set_point_cloud(cloud);

    let mut buffer = EncoderBuffer::new();
    encoder.encode(&options, &mut buffer)?;

    Ok(buffer.data().to_vec())
}

fn quantization_bits(points: &[Point], voxel_size: f32) -> i32 {
    let extent = (0..3)
        .map(|axis| {
            let values = points.iter().map(|point| point[axis]);
            values.clone().fold(f32::MIN, f32::max) - values.fold(f32::MAX, f32::min)
        })
        .fold(0.0, f32::max);

    let steps = extent / voxel_size + 1.0;

    (steps.log2().ceil() as i32).clamp(1, MAX_QUANTIZATION_BITS)
}

/// Metadata, which the encoder never writes, can recurse deep enough to overflow a thread's stack.
pub fn decode(draco: &[u8]) -> Result<Vec<Point>, DracoError> {
    if draco.len() > MAX_DRACO_BYTES || has_metadata(draco) {
        return Err(DracoError::new(
            ErrorKind::UnsupportedFeature,
            "not a point cloud chunk",
        ));
    }

    let limits = DecodeLimits::default()
        .with_max_points(MAX_DECODED_POINTS)
        .with_max_decoded_bytes(MAX_DECODED_BYTES);
    let mut buffer = DecoderBuffer::new(draco).with_limits(limits);
    let mut cloud = PointCloud::new();
    PointCloudDecoder::new().decode(&mut buffer, &mut cloud)?;

    let positions = cloud
        .named_attribute(GeometryAttributeType::Position)
        .map(|position| position.read_f32s(cloud.num_points(), 3))
        .unwrap_or_default();

    Ok(positions.as_chunks().0.to_vec())
}

fn has_metadata(draco: &[u8]) -> bool {
    draco
        .get(HEADER_FLAGS_OFFSET..)
        .and_then(|flags| flags.first_chunk())
        .is_some_and(|flags| u16::from_le_bytes(*flags) & METADATA_FLAG != 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Decodes to 134M points and 3.9 GB with Draco's default limits.
    const HUGE_POINT_COUNT: [u8; 82] = [
        0x44, 0x52, 0x41, 0x43, 0x4f, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x08,
        0x01, 0x02, 0x00, 0x09, 0x03, 0x00, 0x00, 0x04, 0x02, 0x01, 0x00, 0x01, 0x02, 0x01, 0x00,
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x40, 0x01, 0x00, 0x61, 0x00, 0x00,
        0x00, 0x61, 0x00, 0x00, 0x00, 0xef, 0x57, 0xff, 0x3f, 0x2b, 0xbd, 0xe3, 0x3f, 0x15, 0xb8,
        0xcd, 0x3e, 0x00, 0x00, 0x80, 0x3f, 0x0b,
    ];

    #[test]
    fn decode_rejects_malformed_chunks() {
        assert!(decode(&HUGE_POINT_COUNT).is_err());

        let points: Vec<Point> = (0..100).map(|i| [i as f32 * 0.1, 0.0, 1.0]).collect();
        let mut chunk = encode(&points, 0.1).unwrap();
        assert_eq!(decode(&chunk).unwrap().len(), points.len());

        chunk[HEADER_FLAGS_OFFSET + 1] |= 0x80;
        assert!(decode(&chunk).is_err());
    }

    #[test]
    fn voxels_skip_no_returns_and_cropped_points() {
        let points = [
            [0.0, 0.0, 0.0],
            [f32::NAN, 1.0, 1.0],
            [f32::INFINITY, 0.0, 0.0],
            [60.0, 0.0, 0.0],
            [1.0, 2.0, -0.5],
            [1.0, 2.0, 2.5],
            [1.0, 2.0, 0.5],
        ];
        let crop = Crop {
            max_range_m: 50.0,
            min_height_m: 0.0,
            max_height_m: 2.0,
        };

        let (_, centers) = voxel_centers_by_azimuth(&points, 1.0, &crop);

        assert_eq!(centers, vec![[1.5, 2.5, 0.5]]);
    }
}
