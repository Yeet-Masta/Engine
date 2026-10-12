module audio

import time

fn test_capacity_is_power_of_two() {
	r := new_ring_buffer(1000, 2)
	assert r.capacity() == 1024
	assert r.fill() == 0
	assert r.free_frames() == 1024
}

fn test_write_read_wraps_correctly() {
	mut r := new_ring_buffer(8, 1)
	mut out := []f32{len: 8}
	mut next := f32(0)
	mut expect := f32(0)
	for _ in 0 .. 50 {
		mut chunk := []f32{len: 5}
		for i in 0 .. 5 {
			chunk[i] = next
			next += 1
		}
		assert r.write(chunk, 5) == 5
		r.read_into(mut out, 5)
		for i in 0 .. 5 {
			assert out[i] == expect
			expect += 1
		}
	}
	assert r.underrun_frames() == 0
}

fn test_write_never_overwrites_unread() {
	mut r := new_ring_buffer(4, 1)
	assert r.write([f32(1), 2, 3, 4, 5, 6], 6) == 4
	assert r.fill() == 4
	assert r.write([f32(9)], 1) == 0
}

fn test_underrun_fades_instead_of_clicking() {
	mut r := new_ring_buffer(16, 1)
	r.write([f32(0.5), 0.5], 2)
	mut out := []f32{len: 10}
	r.read_into(mut out, 10)
	assert out[0] == 0.5 && out[1] == 0.5
	// Faked samples continue from the last value and decay monotonically.
	assert out[2] > 0.49 && out[2] < 0.5
	for i in 3 .. 10 {
		assert out[i] < out[i - 1]
	}
	assert r.underrun_frames() == 8
}

fn producer(mut r RingBuffer, total int) {
	mut sent := 0
	mut chunk := []f32{len: 64 * 2}
	for sent < total {
		n := if total - sent < 64 { total - sent } else { 64 }
		for i in 0 .. n {
			chunk[2 * i] = f32(sent + i)
			chunk[2 * i + 1] = -f32(sent + i)
		}
		mut done := 0
		for done < n {
			done += r.write(chunk[2 * done..], n - done)
			if done < n {
				time.sleep(10 * time.microsecond)
			}
		}
		sent += n
	}
}

fn test_threaded_spsc_preserves_order() {
	total := 200_000
	mut r := new_ring_buffer(512, 2)
	t := spawn producer(mut r, total)
	mut got := 0
	mut out := []f32{len: 100 * 2}
	for got < total {
		avail := r.fill()
		if avail == 0 {
			time.sleep(5 * time.microsecond)
			continue
		}
		n := if avail < 100 { avail } else { 100 }
		r.read_into(mut out, n)
		for i in 0 .. n {
			assert out[2 * i] == f32(got + i)
			assert out[2 * i + 1] == -f32(got + i)
		}
		got += n
	}
	t.wait()
	assert r.underrun_frames() == 0
}
