use std::{io::{self, BufReader, Read}, mem::{size_of, transmute}, net::UdpSocket};

use nalgebra::{UnitQuaternion, Vector2, Vector3};

pub type Vec2 = Vector2<f32>;
pub type Vec3 = Vector3<f32>;
pub type Quat = UnitQuaternion<f32>;

pub struct OSFPacket {
    /// The time this tracking data was captured at.
    pub time: f64,
    /// This is the id of the tracked face. When tracking multiple faces, they might get reordered due to faces coming and going, but as long as tracking is not lost on a face, its id should stay the same. Face ids depend only on the order of first detection and locations of the faces.
    pub id: i32,
    
    /// This field gives the resolution of the camera or video being tracked.
    pub camera_resolution: Vec2,
    
    /// This field tells you how likely it is that the right eye is open.
    pub right_eye_open: f32,
    /// This field tells you how likely it is that the left eye is open.
    pub left_eye_open: f32,
    
    /// This field tells you if 3D points have been successfully estimated from the 2D points. If this is false, do not rely on pose or 3D data.
    pub got_3d_points: bool,
    /// This field contains the error for fitting the original 3D points. It shouldn't matter much, but it it is very high, something is probably wrong
    pub fit_3d_error: f32,
    
    /// This is the raw rotation quaternion calculated from the OpenCV rotation matrix.
    pub quaternion: Quat,
    /// This is the raw rotation euler angles calculated by OpenCV from the rotation matrix.
    pub rotation: Vec3,
    /// This is the translation vector for the 3D points to turn into the estimated face pose.
    pub translation: Vec3,

    /// This field tells you how certain the tracker is.
    pub confidence: [f32; 68],
    /// These are the detected face landmarks in image coordinates. There are 68 points. The last too points are pupil points from the gaze tracker.
    pub points: [Vec2; 68],
    /// These are 3D points estimated from the 2D points. The should be rotation and translation compensated. There are 70 points with guesses for the eyeball center positions being added at the end of the 68 2D points.
    pub points_3d: [Vec3; 70],

    /// This field contains a number of action unit like features.
    pub features: OSFFeatures
}

impl OSFPacket {
    pub fn read(mut source: impl Read) -> io::Result<OSFPacket> {
        const POINTS_FLOATS: usize = 2 * 68;
        const POINTS3D_FLOATS: usize = 3 * 70;
        unsafe {
            Ok(OSFPacket {
                time: f64::from_le_bytes(read(&mut source)?),
                id: i32::from_be_bytes(read(&mut source)?),
    
                camera_resolution: transmute(f32_arr_from_le::<2>(&mut source)?),
    
                right_eye_open: f32::from_le_bytes(read(&mut source)?),
                left_eye_open: f32::from_le_bytes(read(&mut source)?),
    
                got_3d_points: read::<1>(&mut source)?[0] > 0,
                fit_3d_error: f32::from_le_bytes(read(&mut source)?),

                quaternion: transmute(f32_arr_from_le::<4>(&mut source)?),
                rotation: transmute(f32_arr_from_le::<3>(&mut source)?),
                translation: transmute(f32_arr_from_le::<3>(&mut source)?),

                confidence: f32_arr_from_le(&mut source)?,
                points: transmute(f32_arr_from_le::<POINTS_FLOATS>(&mut source)?),
                points_3d: transmute(f32_arr_from_le::<POINTS3D_FLOATS>(&mut source)?),

                features: OSFFeatures::read(&mut source)?
            })
        }
    }
}

pub struct OSFFeatures {
    /// This field indicates whether the left eye is opened(0) or closed (-1). A value of 1 means open wider than normal.
    pub eye_left: f32,
    /// This field indicates whether the right eye is opened(0) or closed (-1). A value of 1 means open wider than normal.
    pub eye_right: f32,
    
    /// This field indicates how steep the left eyebrow is, compared to the median steepness.
    pub eyebrow_steepness_left: f32,
    /// This field indicates how far up or down the left eyebrow is, compared to its median position.
    pub eyebrow_up_down_left: f32,
    /// This field indicates how quirked the left eyebrow is, compared to its median quirk.
    pub eyebrow_quirk_left: f32,

    /// This field indicates how steep the right eyebrow is, compared to the median steepness.
    pub eyebrow_steepness_right: f32,
    /// This field indicates how far up or down the right eyebrow is, compared to its median position.
    pub eyebrow_up_down_right: f32,
    /// This field indicates how quirked the right eyebrow is, compared to its median quirk.
    pub eyebrow_quirk_right: f32,

    /// This field indicates how far up or down the left mouth corner is, compared to its median position.
    pub mouth_corner_up_down_left: f32,
    /// This field indicates how far in or out the left mouth corner is, compared to its median position.
    pub mouth_corner_in_out_left: f32,

    /// This field indicates how far up or down the right mouth corner is, compared to its median position.
    pub mouth_corner_up_down_right: f32,
    /// This field indicates how far in or out the right mouth corner is, compared to its median position.
    pub mouth_corner_in_out_right: f32,

    /// This field indicates how open or closed the mouth is, compared to its median pose.
    pub mouth_open: f32,
    /// This field indicates how wide the mouth is, compared to its median pose.
    pub mouth_wide: f32
}

impl OSFFeatures {
    pub fn read(source: impl Read) -> io::Result<OSFFeatures> {
        unsafe {
            Ok(transmute(f32_arr_from_le::<14>(source)?))
        }
    }
}

fn read<const N: usize>(mut source: impl Read) -> io::Result<[u8; N]> {
    let mut buf = [0u8; N];
    source.read_exact(&mut buf)?;
    Ok(buf)
}

fn f32_arr_from_le<const N: usize>(mut input: impl Read) -> io::Result<[f32; N]> {
    let mut floats = [0f32; N];
    for i in 0..N {
        floats[i] = f32::from_le_bytes(read(&mut input)?);
    }
    Ok(floats)
}

pub fn wait_until_packet(socket: &UdpSocket) -> io::Result<OSFPacket> {
    const PACKET_SIZE: usize = size_of::<OSFPacket>();
    
    let mut packet_data = [0u8; PACKET_SIZE];
    let mut current_len = 0;
    loop {
        if let Ok(len) = socket.recv(&mut packet_data[current_len..]) {
            current_len += len;
            if current_len == PACKET_SIZE {
                let packet: OSFPacket;
                packet = OSFPacket::read(BufReader::new(&packet_data[..]))?;
                return Ok(packet);
            }
        }
    }
}