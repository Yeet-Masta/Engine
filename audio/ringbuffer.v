// Module audio provides the lock-free ring buffer between the simulation
// thread and the audio callback, the sokol.audio output, and a WAV writer
// for offline renders.
module audio

import sync.stdatomic

// RingBuffer is a single-producer / single-consumer queue of interleaved
// f32 frames. The producer (simulation thread) only advances `write_pos`, the
// consumer (audio callback) only advances `read_pos`. Both are monotonically
// increasing 64-bit counters, so `write_pos - read_pos` is always the fill
// level and there is no ABA or wrap ambiguity. Neither side ever blocks or
// allocates.
@[heap]
pub struct RingBuffer {
mut:
	data      []f32
	mask      u64 // capacity_frames - 1 (capacity is a power of two)
	channels  int
	write_pos u64 // frames written (producer-owned)
	read_pos  u64 // frames read (consumer-owned)
	underruns u64 // frames the consumer had to synthesise (consumer-owned)
	last      []f32
	fade      f32 = 1.0
}

// new_ring_buffer creates a buffer holding at least `min_frames` frames.
pub fn new_ring_buffer(min_frames int, channels int) &RingBuffer {
	mut cap := 1
	for cap < min_frames {
		cap <<= 1
	}
	return &RingBuffer{
		data:     []f32{len: cap * channels}
		mask:     u64(cap - 1)
		channels: channels
		last:     []f32{len: channels}
	}
}

// capacity returns the capacity in frames.
pub fn (r &RingBuffer) capacity() int {
	return int(r.mask + 1)
}

// fill returns the number of frames available to read. Safe from either side.
pub fn (r &RingBuffer) fill() int {
	w := stdatomic.load_u64(&r.write_pos)
	rd := stdatomic.load_u64(&r.read_pos)
	return int(w - rd)
}

// free_frames returns how many frames the producer may write.
pub fn (r &RingBuffer) free_frames() int {
	return r.capacity() - r.fill()
}

// underrun_frames returns the total number of frames that had to be faked.
pub fn (r &RingBuffer) underrun_frames() u64 {
	return stdatomic.load_u64(&r.underruns)
}

// write copies up to `frames` interleaved frames from `src` (producer side).
// Returns the number of frames written.
@[direct_array_access]
pub fn (mut r RingBuffer) write(src []f32, frames int) int {
	w := stdatomic.load_u64(&r.write_pos)
	rd := stdatomic.load_u64(&r.read_pos)
	space := int(r.mask + 1 - (w - rd))
	n := if frames < space { frames } else { space }
	ch := r.channels
	for i in 0 .. n {
		idx := int((w + u64(i)) & r.mask) * ch
		for c in 0 .. ch {
			r.data[idx + c] = src[i * ch + c]
		}
	}
	// Publish after the data is in place (seq-cst store orders the copies).
	stdatomic.store_u64(&r.write_pos, w + u64(n))
	return n
}

// read fills `frames` frames into `dst` (consumer side, called from the audio
// callback). If not enough data is available the remainder is filled with the
// last sample fading to silence, so an underrun is a dip, never a click.
@[direct_array_access]
pub fn (mut r RingBuffer) read(dst &f32, frames int, dst_channels int) {
	w := stdatomic.load_u64(&r.write_pos)
	rd := stdatomic.load_u64(&r.read_pos)
	avail := int(w - rd)
	n := if frames < avail { frames } else { avail }
	ch := r.channels
	for i in 0 .. n {
		idx := int((rd + u64(i)) & r.mask) * ch
		for c in 0 .. dst_channels {
			src_c := if c < ch { c } else { ch - 1 }
			unsafe {
				dst[i * dst_channels + c] = r.data[idx + src_c]
			}
		}
	}
	if n > 0 {
		last_idx := int((rd + u64(n - 1)) & r.mask) * ch
		for c in 0 .. ch {
			r.last[c] = r.data[last_idx + c]
		}
		r.fade = 1.0
	}
	stdatomic.store_u64(&r.read_pos, rd + u64(n))
	if n < frames {
		for i in n .. frames {
			r.fade *= 0.995
			for c in 0 .. dst_channels {
				src_c := if c < ch { c } else { ch - 1 }
				unsafe {
					dst[i * dst_channels + c] = r.last[src_c] * r.fade
				}
			}
		}
		stdatomic.store_u64(&r.underruns, stdatomic.load_u64(&r.underruns) + u64(frames - n))
	}
}

// read_into is a safe wrapper of `read` for tests and offline use.
pub fn (mut r RingBuffer) read_into(mut dst []f32, frames int) {
	if dst.len < frames * r.channels {
		return
	}
	r.read(unsafe { &f32(dst.data) }, frames, r.channels)
}
