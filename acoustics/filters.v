module acoustics

import math
import util

// OnePole is a DC-unity one-pole lowpass: y += a*(x - y).
pub struct OnePole {
pub mut:
	a f64 = 1.0
	y f64
}

pub fn new_lowpass1(fc f64, fs f64) OnePole {
	return OnePole{
		a: util.one_pole_coef(fc, fs)
	}
}

pub fn (mut f OnePole) set_cutoff(fc f64, fs f64) {
	f.a = util.one_pole_coef(fc, fs)
}

@[inline]
pub fn (mut f OnePole) process(x f64) f64 {
	f.y += f.a * (x - f.y)
	return f.y
}

// DcBlocker removes DC: y = x - x1 + r*y1.
pub struct DcBlocker {
pub mut:
	r  f64 = 0.999
	x1 f64
	y1 f64
}

pub fn new_dc_blocker(fc f64, fs f64) DcBlocker {
	return DcBlocker{
		r: math.exp(-2.0 * math.pi * fc / fs)
	}
}

@[inline]
pub fn (mut d DcBlocker) process(x f64) f64 {
	y := x - d.x1 + d.r * d.y1
	d.x1 = x
	d.y1 = y
	return y
}

pub enum BiquadKind {
	lowpass
	highpass
	bandpass
	peaking
}

// Biquad is a transposed direct form II biquad (RBJ cookbook designs).
pub struct Biquad {
pub mut:
	b0 f64 = 1.0
	b1 f64
	b2 f64
	a1 f64
	a2 f64
	z1 f64
	z2 f64
}

// new_biquad designs a filter. `gain_db` is only used by `.peaking`.
pub fn new_biquad(kind BiquadKind, fc f64, q f64, gain_db f64, fs f64) Biquad {
	mut f := Biquad{}
	f.design(kind, fc, q, gain_db, fs)
	return f
}

pub fn (mut f Biquad) design(kind BiquadKind, fc_ f64, q f64, gain_db f64, fs f64) {
	fc := util.clamp(fc_, 1.0, fs * 0.49)
	w0 := 2.0 * math.pi * fc / fs
	cw := math.cos(w0)
	alpha := math.sin(w0) / (2.0 * q)
	mut b0 := 0.0
	mut b1 := 0.0
	mut b2 := 0.0
	mut a0 := 1.0
	mut a1 := 0.0
	mut a2 := 0.0
	match kind {
		.lowpass {
			b0 = (1.0 - cw) * 0.5
			b1 = 1.0 - cw
			b2 = b0
			a0 = 1.0 + alpha
			a1 = -2.0 * cw
			a2 = 1.0 - alpha
		}
		.highpass {
			b0 = (1.0 + cw) * 0.5
			b1 = -(1.0 + cw)
			b2 = b0
			a0 = 1.0 + alpha
			a1 = -2.0 * cw
			a2 = 1.0 - alpha
		}
		.bandpass {
			// constant 0 dB peak gain
			b0 = alpha
			b1 = 0
			b2 = -alpha
			a0 = 1.0 + alpha
			a1 = -2.0 * cw
			a2 = 1.0 - alpha
		}
		.peaking {
			amp := math.pow(10.0, gain_db / 40.0)
			b0 = 1.0 + alpha * amp
			b1 = -2.0 * cw
			b2 = 1.0 - alpha * amp
			a0 = 1.0 + alpha / amp
			a1 = -2.0 * cw
			a2 = 1.0 - alpha / amp
		}
	}
	f.b0 = b0 / a0
	f.b1 = b1 / a0
	f.b2 = b2 / a0
	f.a1 = a1 / a0
	f.a2 = a2 / a0
}

@[inline]
pub fn (mut f Biquad) process(x f64) f64 {
	y := f.b0 * x + f.z1
	f.z1 = f.b1 * x - f.a1 * y + f.z2
	f.z2 = f.b2 * x - f.a2 * y
	return y
}

pub fn (mut f Biquad) reset() {
	f.z1 = 0
	f.z2 = 0
}

// FirDecimator low-passes a signal running at `factor` times the output rate
// and returns one output sample per `factor` inputs. Windowed-sinc design.
pub struct FirDecimator {
pub mut:
	factor int
	taps   []f64
	hist   []f64 // doubled history so the convolution never wraps
	pos    int
	count  int
}

// new_fir_decimator designs a decimator with cutoff at `cutoff` x output
// Nyquist (e.g. 0.9). factor == 1 gives a pass-through.
pub fn new_fir_decimator(factor int, taps_per_phase int, cutoff f64) FirDecimator {
	if factor <= 1 {
		return FirDecimator{
			factor: 1
			taps:   [1.0]
			hist:   []f64{len: 2}
		}
	}
	n := factor * taps_per_phase + 1
	mut taps := []f64{len: n}
	fc := cutoff * 0.5 / f64(factor) // cycles per input sample
	mid := f64(n - 1) * 0.5
	mut sum := 0.0
	for i in 0 .. n {
		x := f64(i) - mid
		sinc := if math.abs(x) < 1e-9 {
			2.0 * fc
		} else {
			math.sin(2.0 * math.pi * fc * x) / (math.pi * x)
		}
		// Blackman window
		w := 0.42 - 0.5 * math.cos(2.0 * math.pi * f64(i) / f64(n - 1)) +
			0.08 * math.cos(4.0 * math.pi * f64(i) / f64(n - 1))
		taps[i] = sinc * w
		sum += taps[i]
	}
	for i in 0 .. n {
		taps[i] /= sum
	}
	return FirDecimator{
		factor: factor
		taps:   taps
		hist:   []f64{len: 2 * n}
	}
}

// push adds one input sample. Returns true when an output is ready, which
// can then be read with `output()`.
@[direct_array_access; inline]
pub fn (mut d FirDecimator) push(x f64) bool {
	n := d.taps.len
	d.hist[d.pos] = x
	d.hist[d.pos + n] = x
	d.pos++
	if d.pos >= n {
		d.pos = 0
	}
	d.count++
	if d.count >= d.factor {
		d.count = 0
		return true
	}
	return false
}

// output convolves the current history with the filter taps.
@[direct_array_access]
pub fn (d &FirDecimator) output() f64 {
	n := d.taps.len
	mut acc := 0.0
	// hist[pos .. pos+n) holds the last n samples, oldest first.
	base := d.pos
	for i in 0 .. n {
		acc += d.taps[i] * d.hist[base + i]
	}
	return acc
}
