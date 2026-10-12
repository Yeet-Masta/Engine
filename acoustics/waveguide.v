module acoustics

import math

// DelayLine is a circular buffer read at a fractional delay (in samples,
// >= 1) with linear interpolation. `write` must be called once per step after
// all reads of that step.
pub struct DelayLine {
mut:
	buf  []f64
	mask int
	pos  int // next write index
}

pub fn new_delay_line(max_delay f64) DelayLine {
	mut n := 4
	for n < int(max_delay) + 4 {
		n <<= 1
	}
	return DelayLine{
		buf:  []f64{len: n}
		mask: n - 1
	}
}

pub fn (mut d DelayLine) fill(v f64) {
	for i in 0 .. d.buf.len {
		d.buf[i] = v
	}
}

@[direct_array_access; inline]
pub fn (d &DelayLine) read(delay f64) f64 {
	di := int(delay)
	frac := delay - f64(di)
	i0 := (d.pos - di) & d.mask
	i1 := (i0 - 1) & d.mask
	return d.buf[i0] + (d.buf[i1] - d.buf[i0]) * frac
}

@[direct_array_access; inline]
pub fn (mut d DelayLine) write(x f64) {
	d.buf[d.pos] = x
	d.pos = (d.pos + 1) & d.mask
}

// Pipe is a 1-D digital waveguide: two delay lines carrying the right- and
// left-going pressure waves (gauge pressure, Pa). End A is at `from`, end B
// at `to`. Waves are low-passed on arrival to model wall losses; the filter
// has unity DC gain so mean mass flow is conserved.
pub struct Pipe {
pub mut:
	fwd   DelayLine // travelling A -> B
	bwd   DelayLine // travelling B -> A
	delay f64       // samples
	area  f64
	c     f64 // speed of sound (m/s)
	temp  f64 // gas temperature (K)
	y     f64 // mass-flow admittance A/c (kg/(s*Pa))
	burned f64 // burned fraction of gas carried (1 exhaust, 0 intake)
	lp_a  f64 // arrival filter coefficient
	// Lowest allowed travelling-wave value (gauge Pa). Linear acoustics
	// lets a rarefaction reflected off a closed end reach negative absolute
	// pressure; a real gas cannot, so arriving waves saturate here.
	floor f64 = -1e9
	st_a  f64 // filter state for waves arriving at A
	st_b  f64 // filter state for waves arriving at B
	// Per-step values
	arr_a f64 // wave arriving at A (from bwd)
	arr_b f64 // wave arriving at B (from fwd)
	out_a f64 // wave leaving A (into fwd)
	out_b f64 // wave leaving B (into bwd)
}

// new_pipe creates a waveguide. `cutoff` is the -3 dB frequency of the
// per-traversal loss filter (Hz); `fs` is the simulation rate.
pub fn new_pipe(length f64, diameter f64, temp f64, gamma f64, cutoff f64, fs f64) Pipe {
	c := math.sqrt(gamma * 287.0 * temp)
	area := math.pi * diameter * diameter * 0.25
	mut delay := length / c * fs
	if delay < 1.0 {
		delay = 1.0
	}
	coef := if cutoff >= fs * 0.45 { 1.0 } else { 1.0 - math.exp(-2.0 * math.pi * cutoff / fs) }
	return Pipe{
		fwd:   new_delay_line(delay)
		bwd:   new_delay_line(delay)
		delay: delay
		area:  area
		c:     c
		temp:  temp
		y:     area / c
		lp_a:  coef
	}
}

// read computes the waves arriving at both ends for this step.
@[inline]
pub fn (mut p Pipe) read() {
	xb := p.fwd.read(p.delay)
	xa := p.bwd.read(p.delay)
	p.st_b += p.lp_a * (xb - p.st_b)
	p.st_a += p.lp_a * (xa - p.st_a)
	p.arr_b = if p.st_b < p.floor { p.floor } else { p.st_b }
	p.arr_a = if p.st_a < p.floor { p.floor } else { p.st_a }
}

// write injects the outgoing waves computed by the end nodes.
@[inline]
pub fn (mut p Pipe) write() {
	p.fwd.write(p.out_a)
	p.bwd.write(p.out_b)
}

// arriving returns the wave arriving at `side` (0 = A, 1 = B).
@[inline]
pub fn (p &Pipe) arriving(side int) f64 {
	return if side == 0 { p.arr_a } else { p.arr_b }
}

@[inline]
pub fn (mut p Pipe) set_outgoing(side int, v f64) {
	if side == 0 {
		p.out_a = v
	} else {
		p.out_b = v
	}
}

// preload fills the pipe with a uniform gauge pressure (no flow).
pub fn (mut p Pipe) preload(gauge f64) {
	p.fwd.fill(gauge * 0.5)
	p.bwd.fill(gauge * 0.5)
	p.st_a = gauge * 0.5
	p.st_b = gauge * 0.5
}

// wall_loss_cutoff estimates a cutoff frequency for a pipe's loss filter from
// its geometry. Viscothermal loss scales with sqrt(f)/radius; real exhaust
// pipes (rough, hot, with mean flow) lose several times more than the
// textbook value, hence `roughness`.
pub fn wall_loss_cutoff(length f64, diameter f64, damping f64) f64 {
	a := diameter * 0.5
	// alpha(f) = k*sqrt(f)/a  (Np/m); -3 dB when alpha*L = 0.35
	k := 3.0e-5 * 6.0 // textbook constant x roughness factor
	s := 0.35 * a / (k * (if length < 0.05 { 0.05 } else { length }))
	mut fc := s * s
	// packing/absorption: pull the cutoff down strongly
	if damping > 0 {
		fc *= math.pow(1.0 - damping * 0.97, 2.0)
	}
	if fc < 300 {
		fc = 300
	}
	return fc
}
