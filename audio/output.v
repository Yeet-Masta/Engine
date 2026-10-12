module audio

import sokol.audio as saudio

// Output owns the audio device. The device callback runs on sokol's audio
// thread and only reads from the ring buffer.
pub struct Output {
pub:
	sample_rate   int
	channels      int
	buffer_frames int
	valid         bool
}

// audio_callback is invoked by sokol on its own thread. It must never block:
// it copies from the ring buffer (lock-free) and returns.
fn audio_callback(buffer &f32, num_frames i32, num_channels i32, user_data voidptr) {
	mut rb := unsafe { &RingBuffer(user_data) }
	rb.read(buffer, int(num_frames), int(num_channels))
}

// open_output starts the audio device and connects it to `ring`.
pub fn open_output(ring &RingBuffer, sample_rate int, buffer_frames int) Output {
	saudio.setup(
		sample_rate:        sample_rate
		num_channels:       ring.channels
		buffer_frames:      buffer_frames
		stream_userdata_cb: audio_callback
		user_data:          voidptr(ring)
	)
	if !saudio.is_valid() {
		return Output{
			sample_rate:   sample_rate
			channels:      ring.channels
			buffer_frames: buffer_frames
			valid:         false
		}
	}
	return Output{
		sample_rate:   saudio.sample_rate()
		channels:      saudio.channels()
		buffer_frames: saudio.buffer_frames()
		valid:         true
	}
}

// close stops the device.
pub fn (o &Output) close() {
	if o.valid {
		saudio.shutdown()
	}
}
