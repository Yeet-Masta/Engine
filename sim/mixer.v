module sim

import math
import acoustics
import config
import util

// Pressure (Pa) that maps to digital full scale.
const full_scale_pa = 2.0
const rho_air = 1.2

// MicState holds the smoothed per-source gains of the active listener.
struct MicState {
mut:
	exhaust    f64
	intake     f64
	mechanical f64
	road       f64
	wind       f64
	width      f64
	boom       f64
}

// Mixer turns simulation signals into the listener's stereo signal. It runs
// at the simulation rate, then decimates to the audio rate.
pub struct Mixer {
pub mut:
	rng    util.Rng
	target MicState
	cur    MicState
	// listener lowpasses (two cascaded one-poles = 12 dB/oct)
	lp_ex_l  [2]acoustics.OnePole
	lp_ex_r  [2]acoustics.OnePole
	lp_in    [2]acoustics.OnePole
	lp_mech  [2]acoustics.OnePole
	boom_l   acoustics.Biquad
	boom_r   acoustics.Biquad
	// source shaping
	noise_hp_l acoustics.Biquad
	noise_hp_r acoustics.Biquad
	hiss_bp    acoustics.Biquad
	block_hp   acoustics.Biquad
	block_bp1  acoustics.Biquad
	block_bp2  acoustics.Biquad
	click_bp   acoustics.Biquad
	road_lp    acoustics.Biquad
	wind_lp    acoustics.Biquad
	squeal_bp  acoustics.Biquad
	grind_bp   acoustics.Biquad
	starter_ph f64
	// distance scaling (1/r) for exhaust per mic
	ex_dist f64 = 1.5
	// output stage (audio rate)
	dec_l   acoustics.FirDecimator
	dec_r   acoustics.FirDecimator
	dc_l    acoustics.DcBlocker
	dc_r    acoustics.DcBlocker
	env     f64
	rms     f64
	fs      f64
	fs_out  f64
	acfg    config.AudioConfig
	straight_cut bool
}

pub fn new_mixer(cfg config.Config, fs f64, fs_out f64, substeps int) Mixer {
	a := cfg.audio
	mut m := Mixer{
		rng:          util.new_rng(cfg.simulation.seed ^ 0xA5A5)
		noise_hp_l:   acoustics.new_biquad(.bandpass, 1800, 0.5, 0, fs)
		noise_hp_r:   acoustics.new_biquad(.bandpass, 1800, 0.5, 0, fs)
		hiss_bp:      acoustics.new_biquad(.bandpass, 2600, 0.6, 0, fs)
		block_hp:     acoustics.new_biquad(.highpass, 500, 0.7, 0, fs)
		block_bp1:    acoustics.new_biquad(.bandpass, 1250, 3.0, 0, fs)
		block_bp2:    acoustics.new_biquad(.bandpass, 2900, 4.0, 0, fs)
		click_bp:     acoustics.new_biquad(.bandpass, 3300, 4.0, 0, fs)
		road_lp:      acoustics.new_biquad(.lowpass, 450, 0.7, 0, fs)
		wind_lp:      acoustics.new_biquad(.lowpass, 320, 0.6, 0, fs)
		squeal_bp:    acoustics.new_biquad(.bandpass, 1050, 18.0, 0, fs)
		grind_bp:     acoustics.new_biquad(.bandpass, 3400, 2.0, 0, fs)
		dec_l:        acoustics.new_fir_decimator(substeps, 24, 0.92)
		dec_r:        acoustics.new_fir_decimator(substeps, 24, 0.92)
		dc_l:         acoustics.new_dc_blocker(12, fs_out)
		dc_r:         acoustics.new_dc_blocker(12, fs_out)
		fs:           fs
		fs_out:       fs_out
		acfg:         a
		straight_cut: cfg.drivetrain.straight_cut
	}
	m.select_mic(a.mic_index(a.listener), true)
	return m
}

// select_mic switches the listener; gains glide to the new values.
pub fn (mut m Mixer) select_mic(idx int, immediate bool) {
	if idx < 0 || idx >= m.acfg.mics.len {
		return
	}
	mic := m.acfg.mics[idx]
	m.target = MicState{
		exhaust:    mic.exhaust
		intake:     mic.intake
		mechanical: mic.mechanical
		road:       mic.road
		wind:       mic.wind
		width:      mic.width
		boom:       mic.boom_gain
	}
	m.ex_dist = if mic.distance < 0.3 { 0.3 } else { mic.distance }
	for k in 0 .. 2 {
		m.lp_ex_l[k].set_cutoff(mic.exhaust_lowpass, m.fs)
		m.lp_ex_r[k].set_cutoff(mic.exhaust_lowpass, m.fs)
		m.lp_in[k].set_cutoff(mic.intake_lowpass, m.fs)
		m.lp_mech[k].set_cutoff(mic.mechanical_lowpass, m.fs)
	}
	gain_db := if mic.boom_gain > 0 { 6.0 * mic.boom_gain } else { 0.0 }
	m.boom_l.design(.peaking, mic.boom_freq, 1.4, gain_db, m.fs)
	m.boom_r.design(.peaking, mic.boom_freq, 1.4, gain_db, m.fs)
	if immediate {
		m.cur = m.target
	}
}

@[inline]
fn glide(cur f64, target f64, a f64) f64 {
	return cur + (target - cur) * a
}

// mix_step produces one stereo sample at the simulation rate from the
// current simulator state and pushes it into the decimators.
@[direct_array_access]
fn (mut s Simulator) mix_step() bool {
	mut m := &s.mix
	a := m.acfg
	ga := 20.0 / m.fs // ~50 ms glide
	m.cur.exhaust = glide(m.cur.exhaust, m.target.exhaust, ga)
	m.cur.intake = glide(m.cur.intake, m.target.intake, ga)
	m.cur.mechanical = glide(m.cur.mechanical, m.target.mechanical, ga)
	m.cur.road = glide(m.cur.road, m.target.road, ga)
	m.cur.wind = glide(m.cur.wind, m.target.wind, ga)
	m.cur.width = glide(m.cur.width, m.target.width, ga)
	m.cur.boom = glide(m.cur.boom, m.target.boom, ga)
	// --- exhaust outlets: monopole radiation + turbulent flow noise ---
	ex_scale := rho_air / (4.0 * math.pi * m.ex_dist) / full_scale_pa
	mut ex_l := 0.0
	mut ex_r := 0.0
	mut nz_l := 0.0
	mut nz_r := 0.0
	for o in s.exhaust.outlets {
		sig := o.radiated * o.gain * ex_scale
		pan := util.clamp(o.pan * m.cur.width, -1.0, 1.0)
		th := (pan + 1.0) * math.pi * 0.25
		gl := math.cos(th) * math.sqrt2
		gr := math.sin(th) * math.sqrt2
		ex_l += sig * gl
		ex_r += sig * gr
		f := math.abs(o.flow)
		nz := m.rng.bipolar() * f * math.sqrt(f) * a.flow_noise * 1.0 * o.gain / m.ex_dist
		nz_l += nz * gl
		nz_r += nz * gr
	}
	ex_l += m.noise_hp_l.process(nz_l)
	ex_r += m.noise_hp_r.process(nz_r)
	// --- intake mouth + throttle hiss ---
	mut intake := 0.0
	for o in s.intake.net.outlets {
		intake += o.radiated * rho_air / (4.0 * math.pi * 1.0) / full_scale_pa
	}
	intake += m.hiss_bp.process(m.rng.bipolar() * s.intake.hiss * a.flow_noise * 0.6)
	// --- mechanical: combustion/block noise, valvetrain, gears, starter ---
	blk := m.block_hp.process(s.eng.block_signal * 3e-6 * a.combustion_noise)
	mut mech := m.block_bp1.process(blk) + 0.7 * m.block_bp2.process(blk)
	mech += m.click_bp.process(s.eng.valve_click * 0.2 * a.valvetrain_noise +
		m.rng.bipolar() * 0.00002 * math.abs(s.eng.omega) / 300.0 * a.valvetrain_noise)
	whine_amp := (0.0006 + s.dtr.mesh_load / 400.0 * 0.004) * (if m.straight_cut { 5.0 } else { 1.0 })
	if s.dtr.in_gear() {
		mech += math.sin(s.dtr.gear_phase) * whine_amp * a.gear_whine
	}
	if s.eng.starter && s.eng.t_starter > 0 {
		m.starter_ph += s.eng.omega * 11.0 * s.dt
		mech += (0.04 * math.sin(m.starter_ph) + 0.02 * math.sin(3.0 * m.starter_ph) +
			0.01 * m.rng.bipolar()) * a.starter_noise
	}
	if s.dtr.grind > 0 {
		mech += m.grind_bp.process(m.rng.bipolar() * util.clamp(s.dtr.grind / 100.0, 0.0, 1.0) * 0.3)
	}
	// --- road and wind ---
	v := math.abs(s.veh.v)
	mut road := m.road_lp.process(m.rng.bipolar()) * v / 30.0 * 0.15
	slip := math.abs(s.veh.slip_speed)
	if slip > 0.8 {
		road += m.squeal_bp.process(m.rng.bipolar()) * util.clamp((slip - 0.8) / 3.0, 0.0, 1.0) * 0.6
	}
	wind := m.wind_lp.process(m.rng.bipolar()) * (v / 40.0) * (v / 40.0) * 0.3
	// --- listener ---
	exl := m.lp_ex_l[1].process(m.lp_ex_l[0].process(ex_l)) * m.cur.exhaust
	exr := m.lp_ex_r[1].process(m.lp_ex_r[0].process(ex_r)) * m.cur.exhaust
	inm := m.lp_in[1].process(m.lp_in[0].process(intake)) * m.cur.intake
	mem := m.lp_mech[1].process(m.lp_mech[0].process(mech)) * m.cur.mechanical
	common := inm + mem + road * m.cur.road + wind * m.cur.wind
	mut l := exl + common
	mut r := exr + common
	if m.cur.boom > 0.01 {
		l = m.boom_l.process(l)
		r = m.boom_r.process(r)
	}
	m.dec_l.push(l)
	return m.dec_r.push(r)
}

// finish_sample runs the audio-rate output stage: decimation, DC removal,
// a gentle compressor, master gain and a soft limiter.
fn (mut s Simulator) finish_sample(volume f64) (f32, f32) {
	mut m := &s.mix
	mut l := m.dc_l.process(m.dec_l.output())
	mut r := m.dc_r.process(m.dec_r.output())
	// RMS-ish envelope for a 2:1 compressor above -24 dBFS
	pk := math.max(math.abs(l), math.abs(r))
	att := if pk > m.env { 0.01 } else { 0.0002 }
	m.env += (pk - m.env) * att
	thresh := 0.063
	mut gain := m.acfg.master_gain * volume
	if m.env > thresh {
		gain *= math.sqrt(thresh / m.env)
	}
	l *= gain
	r *= gain
	m.rms += (0.5 * (l * l + r * r) - m.rms) * 0.0005
	return f32(soft_clip(l)), f32(soft_clip(r))
}

@[inline]
fn soft_clip(x f64) f64 {
	if x > 0.8 {
		return 0.8 + 0.2 * math.tanh((x - 0.8) / 0.2)
	}
	if x < -0.8 {
		return -0.8 + 0.2 * math.tanh((x + 0.8) / 0.2)
	}
	return x
}
