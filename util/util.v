// Module util holds small allocation-free helpers shared by the physics and
// audio modules: a deterministic PRNG and a few scalar math functions.
module util

import math

// Rng is a xorshift64* generator. It is deterministic for a given seed so
// offline renders and tests are reproducible, and it never allocates.
pub struct Rng {
mut:
	state u64 = 0x9E3779B97F4A7C15
}

// new_rng returns a generator seeded with `seed` (0 is remapped).
pub fn new_rng(seed u64) Rng {
	return Rng{
		state: if seed == 0 { u64(0x9E3779B97F4A7C15) } else { seed }
	}
}

// u64 returns the next raw 64-bit value.
@[inline]
pub fn (mut r Rng) u64() u64 {
	mut x := r.state
	x ^= x >> 12
	x ^= x << 25
	x ^= x >> 27
	r.state = x
	return x * u64(0x2545F4914F6CDD1D)
}

// uniform returns a value in [0, 1).
@[inline]
pub fn (mut r Rng) uniform() f64 {
	return f64(r.u64() >> 11) * (1.0 / 9007199254740992.0)
}

// bipolar returns a value in [-1, 1).
@[inline]
pub fn (mut r Rng) bipolar() f64 {
	return 2.0 * r.uniform() - 1.0
}

// gauss returns an approximately normal value (mean 0, stddev 1) using the
// sum of four uniforms (Irwin-Hall), which is cheap and bounded.
@[inline]
pub fn (mut r Rng) gauss() f64 {
	s := r.uniform() + r.uniform() + r.uniform() + r.uniform()
	return (s - 2.0) * 1.7320508075688772
}

@[inline]
pub fn clamp(x f64, lo f64, hi f64) f64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}

@[inline]
pub fn lerp(a f64, b f64, t f64) f64 {
	return a + (b - a) * t
}

// smoothstep is the cubic Hermite step between edges e0 and e1.
@[inline]
pub fn smoothstep(e0 f64, e1 f64, x f64) f64 {
	t := clamp((x - e0) / (e1 - e0), 0.0, 1.0)
	return t * t * (3.0 - 2.0 * t)
}

// wrap returns x wrapped into [0, period).
@[inline]
pub fn wrap(x f64, period f64) f64 {
	mut y := math.fmod(x, period)
	if y < 0 {
		y += period
	}
	return y
}

@[inline]
pub fn deg2rad(d f64) f64 {
	return d * math.pi / 180.0
}

@[inline]
pub fn rad2deg(r f64) f64 {
	return r * 180.0 / math.pi
}

@[inline]
pub fn rpm2rads(rpm f64) f64 {
	return rpm * math.pi / 30.0
}

@[inline]
pub fn rads2rpm(w f64) f64 {
	return w * 30.0 / math.pi
}

// one_pole_coef returns the coefficient `a` for y += a*(x - y) giving a
// -3 dB cutoff of `fc` Hz at sample rate `fs` (exact for the impulse-invariant
// one-pole).
pub fn one_pole_coef(fc f64, fs f64) f64 {
	if fc <= 0 {
		return 0.0
	}
	if fc >= fs * 0.5 {
		return 1.0
	}
	return 1.0 - math.exp(-2.0 * math.pi * fc / fs)
}
