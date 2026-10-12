module audio

import os
import encoding.binary

// WavWriter collects interleaved float samples and writes a 16-bit PCM WAV.
pub struct WavWriter {
pub:
	sample_rate int
	channels    int
mut:
	samples []i16
}

pub fn new_wav_writer(sample_rate int, channels int) WavWriter {
	return WavWriter{
		sample_rate: sample_rate
		channels:    channels
	}
}

// append adds `frames` interleaved frames, clipping to [-1, 1].
pub fn (mut w WavWriter) append(src []f32, frames int) {
	for i in 0 .. frames * w.channels {
		mut x := src[i]
		if x > 1.0 {
			x = 1.0
		} else if x < -1.0 {
			x = -1.0
		}
		w.samples << i16(x * 32767.0)
	}
}

pub fn (w &WavWriter) frames() int {
	return w.samples.len / w.channels
}

// save writes the file.
pub fn (w &WavWriter) save(path string) ! {
	data_bytes := u32(w.samples.len * 2)
	mut buf := []u8{len: 44 + int(data_bytes)}
	copy(mut buf[0..4], 'RIFF'.bytes())
	binary.little_endian_put_u32(mut buf[4..8], 36 + data_bytes)
	copy(mut buf[8..12], 'WAVE'.bytes())
	copy(mut buf[12..16], 'fmt '.bytes())
	binary.little_endian_put_u32(mut buf[16..20], 16)
	binary.little_endian_put_u16(mut buf[20..22], 1) // PCM
	binary.little_endian_put_u16(mut buf[22..24], u16(w.channels))
	binary.little_endian_put_u32(mut buf[24..28], u32(w.sample_rate))
	binary.little_endian_put_u32(mut buf[28..32], u32(w.sample_rate * w.channels * 2))
	binary.little_endian_put_u16(mut buf[32..34], u16(w.channels * 2))
	binary.little_endian_put_u16(mut buf[34..36], 16)
	copy(mut buf[36..40], 'data'.bytes())
	binary.little_endian_put_u32(mut buf[40..44], data_bytes)
	for i, s in w.samples {
		binary.little_endian_put_u16(mut buf[44 + 2 * i..46 + 2 * i], u16(s))
	}
	os.write_file_array(path, buf)!
}
