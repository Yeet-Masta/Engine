// Module sim ties the physics modules together: it owns the engine, intake
// and exhaust networks, drivetrain and vehicle, advances them with a fixed
// timestep, mixes the listener's audio and publishes telemetry. It has no
// knowledge of threads, windows or audio devices.
module sim

import math
import config
import engine
import acoustics
import drivetrain
import vehicle
import input
import util

pub struct Simulator {
pub mut:
	cfg      config.Config
	fs_out   f64
	fs       f64
	dt       f64
	substeps int
	eng      engine.Engine
	intake   acoustics.IntakeSystem
	exhaust  acoustics.Network
	dtr      drivetrain.Drivetrain
	veh      vehicle.Vehicle
	mix      Mixer
	tel      Telemetry
	time     f64
	steps    u64
	// controls
	ctl         input.Snapshot
	pedal       f64 // ramped pedal position
	clutch_down f64 // ramped manual clutch pedal (1 = disengaged)
	auto_engage f64
	blip        f64
	launch      bool
	mic         int
	seen_up     u64
	seen_down   u64
	seen_neut   u64
	seen_mic    u64
	ecu_acc     f64
	frozen      bool // hold crank speed (priming and dyno runs)
	hold_omega  f64
}

// new_simulator builds a simulator for `cfg` producing audio at `sample_rate`.
pub fn new_simulator(cfg config.Config, sample_rate int) !&Simulator {
	sub := cfg.simulation.substeps
	fs := f64(sample_rate * sub)
	mut s := &Simulator{
		cfg:      cfg
		fs_out:   f64(sample_rate)
		fs:       fs
		dt:       1.0 / fs
		substeps: sub
		eng:      engine.new_engine(cfg)!
		intake:   acoustics.new_intake(cfg, fs)!
		exhaust:  acoustics.new_exhaust(cfg, fs)!
		dtr:      drivetrain.new_drivetrain(cfg.drivetrain)
		veh:      vehicle.new_vehicle(cfg.vehicle)
		mix:      new_mixer(cfg, fs, f64(sample_rate), sub)
		mic:      cfg.audio.mic_index(cfg.audio.listener)
	}
	s.ctl = input.Snapshot{
		ignition:    cfg.simulation.start_running
		limiter:     cfg.engine.limiter.enabled
		auto_clutch: cfg.drivetrain.auto_clutch
		rev_match:   cfg.drivetrain.rev_match
	}
	s.auto_engage = 1.0
	if cfg.simulation.start_running {
		s.eng.omega = util.rpm2rads(cfg.engine.idle_rpm)
		s.prime(0.3)
		s.settle(2.5)
		s.time = 0
	}
	return s
}

// prime runs the gas dynamics for `seconds` with the crank speed held, so
// the manifolds and pipes reach a periodic state before audio starts.
pub fn (mut s Simulator) prime(seconds f64) {
	s.frozen = true
	s.hold_omega = s.eng.omega
	n := int(seconds * s.fs)
	for _ in 0 .. n {
		s.step()
	}
	s.frozen = false
}

// settle lets the engine run freely (no audio) so the idle controller and
// manifold pressure reach their steady state before the first sample.
pub fn (mut s Simulator) settle(seconds f64) {
	n := int(seconds * s.fs)
	for _ in 0 .. n {
		s.step()
	}
}

// apply_controls takes a new control snapshot (once per block).
pub fn (mut s Simulator) apply_controls(c input.Snapshot) {
	if c.gear_up != s.seen_up {
		for _ in s.seen_up .. c.gear_up {
			s.dtr.shift_up()
		}
		s.seen_up = c.gear_up
	}
	if c.gear_down != s.seen_down {
		for _ in s.seen_down .. c.gear_down {
			if s.dtr.gear == 1 || (s.dtr.gear == 0 && math.abs(s.veh.v) < 1.0)
				|| s.dtr.gear > 1 {
				s.dtr.shift_down()
			}
		}
		s.seen_down = c.gear_down
	}
	if c.neutral != s.seen_neut {
		s.dtr.select(0)
		s.seen_neut = c.neutral
	}
	if c.mic != s.seen_mic {
		s.mic = (s.mic + int(c.mic - s.seen_mic)) % s.cfg.audio.mics.len
		s.mix.select_mic(s.mic, false)
		s.seen_mic = c.mic
	}
	s.ctl = c
	s.veh.brake = c.brake
}

// sync_counters makes the simulator ignore events already counted in `c`
// (used when a controls object outlives a reloaded simulator).
pub fn (mut s Simulator) sync_counters(c input.Snapshot) {
	s.seen_up = c.gear_up
	s.seen_down = c.gear_down
	s.seen_neut = c.neutral
	s.seen_mic = c.mic
}

// request_gear selects gear `g` directly (scripts).
pub fn (mut s Simulator) request_gear(g int) {
	s.dtr.select(g)
}

// render produces `frames` interleaved stereo frames into `out`.
@[direct_array_access]
pub fn (mut s Simulator) render(mut out []f32, frames int) {
	vol := s.ctl.volume
	for f in 0 .. frames {
		for _ in 0 .. s.substeps {
			s.step()
			s.mix_step()
		}
		l, r := s.finish_sample(vol)
		out[2 * f] = l
		out[2 * f + 1] = r
	}
	s.update_telemetry()
}

// step advances the whole physical system by one simulation step.
@[direct_array_access]
fn (mut s Simulator) step() {
	dt := s.dt
	s.time += dt
	s.steps++
	s.update_driver(dt)
	// ECU at 1/8 of the simulation rate
	s.ecu_acc += dt
	if s.steps % 8 == 0 {
		map_ratio := s.intake.plenum_pressure() / s.cfg.simulation.ambient_pressure
		ecu_in := engine.EcuInputs{
			pedal:           s.pedal
			ignition:        s.ctl.ignition
			limiter_enabled: s.ctl.limiter
			launch_active:   s.launch
			blip:            s.blip
		}
		s.eng.ecu.update(s.ecu_acc, s.eng.rpm(), map_ratio, ecu_in)
		s.ecu_acc = 0
	}
	s.eng.step_gas(dt, mut s.intake.net.ports, mut s.exhaust.ports)
	s.intake.step(dt, s.eng.ecu.plate, s.eng.ecu.bypass)
	s.exhaust.step()
	s.eng.mech_torques(s.ctl.starter, s.cfg.engine.starter)
	if s.frozen {
		s.eng.integrate(dt, s.eng.net_torque())
		s.eng.omega = s.hold_omega
		return
	}
	tc := s.dtr.step(dt, s.eng.omega, s.eng.net_torque(), s.eng.i_eff, mut s.veh)
	s.eng.integrate(dt, tc)
}

// update_driver ramps the pedals and runs the automatic clutch, rev-matching
// and launch control logic.
fn (mut s Simulator) update_driver(dt f64) {
	// pedal dynamics: ~80 ms to floor it, ~100 ms to lift
	target := util.clamp(s.ctl.throttle, 0.0, 1.0)
	rate := if target > s.pedal { dt / 0.08 } else { dt / 0.10 }
	s.pedal += util.clamp(target - s.pedal, -rate, rate)
	ctarget := util.clamp(s.ctl.clutch, 0.0, 1.0)
	crate := dt / 0.12
	s.clutch_down += util.clamp(ctarget - s.clutch_down, -crate, crate)
	manual := 1.0 - s.clutch_down
	idle := s.cfg.engine.idle_rpm
	rpm := s.eng.rpm()
	shifting := s.dtr.state == .shifting || s.dtr.state == .syncing
	mut engage := manual
	if s.ctl.auto_clutch {
		mut auto_target := 1.0
		if shifting {
			auto_target = 0.0
		} else if s.dtr.gear != 0 {
			matched := util.rads2rpm(math.abs(s.dtr.w_whl * s.dtr.total_ratio()))
			if matched < idle * 1.2 {
				// launching / crawling: centrifugal-style engagement by rpm
				launch_rpm := if s.launch { s.eng.ecu.limit_rpm * 0.8 } else { idle + 900.0 }
				auto_target = util.smoothstep(idle + 80.0, launch_rpm, rpm)
				if s.pedal < 0.03 && matched < idle * 0.9 {
					auto_target = 0.0
				}
			}
		}
		up := dt / 0.25
		down := dt / 0.08
		s.auto_engage += util.clamp(auto_target - s.auto_engage, -down, up)
		engage = math.min(engage, s.auto_engage)
	}
	s.dtr.engage = engage
	// rev matching: blip the throttle toward the new gear's speed while the
	// clutch is open during a shift
	s.blip = 0
	if s.ctl.rev_match && shifting && s.dtr.gear != 0 && engage < 0.5 {
		want := util.rads2rpm(math.abs(s.dtr.matched_engine_speed(s.dtr.gear)))
		if want > rpm + 150 {
			s.blip = util.clamp((want - rpm) / 2500.0, 0.0, 0.9)
		}
	}
	// auto-clutch shifts lift the driver's throttle like a robotised gearbox
	if s.ctl.auto_clutch && shifting && engage < 0.5 {
		s.pedal = math.min(s.pedal, 0.0)
	}
	// launch control: armed, (nearly) stationary, clutch open or slipping
	s.launch = s.ctl.launch && math.abs(s.veh.v) < 2.0 && engage < 0.95
}

// update_telemetry refreshes the snapshot (once per block).
pub fn (mut s Simulator) update_telemetry() {
	mut t := &s.tel
	e := &s.eng
	t.time = s.time
	t.rpm = e.rpm()
	t.crank_deg = util.rad2deg(e.theta)
	t.pedal = s.pedal
	t.plate = e.ecu.plate
	t.clutch = s.dtr.engage
	t.gear = s.dtr.gear
	t.shift_state = match s.dtr.state {
		.engaged { 0 }
		.shifting { 1 }
		.syncing { 2 }
		.neutral { 3 }
	}
	t.speed_kmh = s.veh.speed_kmh()
	t.torque = e.torque_avg
	t.indicated = e.ind_avg
	t.power_kw = e.torque_avg * e.omega / 1000.0
	t.map_kpa = s.intake.plenum_pressure() / 1000.0
	t.afr = e.ecu.afr
	t.spark = e.ecu.spark
	t.ve = e.volumetric_efficiency()
	t.air_gs += (s.intake.throttle_flow * 1000.0 - t.air_gs) * 0.2
	t.idle_valve = e.ecu.bypass
	t.clutch_slip = util.rads2rpm(s.dtr.clutch_slip)
	t.wheel_slip = s.veh.slip_speed
	t.limiter_cut = e.ecu.limiter_cut
	t.fuel_cut = e.ecu.fuel_cut
	t.ignition = s.ctl.ignition
	t.starter = s.ctl.starter
	t.limiter_on = s.ctl.limiter
	t.launch = s.launch
	t.auto_clutch = s.ctl.auto_clutch
	t.rev_match = s.ctl.rev_match
	t.mic = s.mic
	t.level_db = 10.0 * math.log10(s.mix.rms + 1e-12)
	n := if e.cyl.len > max_cyl { max_cyl } else { e.cyl.len }
	t.ncyl = n
	for i in 0 .. n {
		c := e.cyl[i]
		t.cyl_bar[i] = c.gas.pressure() / 1e5
		t.cyl_temp[i] = c.gas.temperature()
		t.cyl_phi[i] = c.phi
		t.cyl_in_lift[i] = c.in_lift * 1000.0
		t.cyl_ex_lift[i] = c.ex_lift * 1000.0
		t.cyl_burning[i] = c.burning
	}
	t.exhaust_flow = s.exhaust.total_outlet_flow()
	t.fires = e.fires
	t.misfires = e.misfires
}

// transfer_from carries the driving state of a previous simulator over
// (hot reload): crank speed and phase, gear, vehicle speed and controls.
pub fn (mut s Simulator) transfer_from(old &Simulator) {
	s.eng.omega = old.eng.omega
	s.eng.theta = old.eng.theta
	s.eng.sync_phase()
	s.eng.reset_gas()
	if old.dtr.gear <= s.dtr.gear_count() {
		s.dtr.gear = old.dtr.gear
		s.dtr.state = old.dtr.state
	}
	s.dtr.w_in = old.dtr.w_in
	s.dtr.w_whl = old.dtr.w_whl
	s.veh.v = old.veh.v
	s.veh.x = old.veh.x
	s.ctl = old.ctl
	s.pedal = old.pedal
	s.clutch_down = old.clutch_down
	s.auto_engage = old.auto_engage
	s.seen_up = old.seen_up
	s.seen_down = old.seen_down
	s.seen_neut = old.seen_neut
	s.seen_mic = old.seen_mic
	if old.mic < s.cfg.audio.mics.len {
		s.mic = old.mic
		s.mix.select_mic(s.mic, true)
	}
	// settle the gas dynamics at the current speed
	w := s.eng.omega
	s.frozen_prime(0.25, w)
}

fn (mut s Simulator) frozen_prime(seconds f64, w f64) {
	n := int(seconds * s.fs)
	for _ in 0 .. n {
		s.eng.step_gas(s.dt, mut s.intake.net.ports, mut s.exhaust.ports)
		s.intake.step(s.dt, s.eng.ecu.plate, s.eng.ecu.bypass)
		s.exhaust.step()
		s.eng.integrate(s.dt, s.eng.net_torque())
		s.eng.omega = w
	}
}

pub fn (s &Simulator) sim_rate() f64 {
	return s.fs
}

// DynoPoint is one steady-state measurement at a held engine speed.
pub struct DynoPoint {
pub mut:
	rpm       f64
	torque    f64 // brake torque (N*m)
	indicated f64
	power_kw  f64
	map_kpa   f64
	ve        f64
	peak_bar  f64
	air_gs    f64
}

// dyno holds the crank at `rpm` with the given pedal, lets it settle, then
// averages torque over `measure` seconds. Audio is not produced.
pub fn (mut s Simulator) dyno(rpm f64, pedal f64, settle f64, measure f64) DynoPoint {
	s.frozen = true
	s.hold_omega = util.rpm2rads(rpm)
	s.eng.omega = s.hold_omega
	s.ctl.throttle = pedal
	s.pedal = pedal
	mut n := int(settle * s.fs)
	for _ in 0 .. n {
		s.step()
	}
	n = int(measure * s.fs)
	mut tq := 0.0
	mut ind := 0.0
	mut mp := 0.0
	mut peak := 0.0
	mut air := 0.0
	for _ in 0 .. n {
		s.step()
		tq += s.eng.t_gas - s.eng.t_fric
		ind += s.eng.t_gas
		mp += s.intake.plenum_pressure()
		air += s.intake.throttle_flow
		for c in s.eng.cyl {
			p := c.gas.pressure()
			if p > peak {
				peak = p
			}
		}
	}
	s.frozen = false
	fnn := f64(n)
	return DynoPoint{
		rpm:       rpm
		torque:    tq / fnn
		indicated: ind / fnn
		power_kw:  tq / fnn * s.hold_omega / 1000.0
		map_kpa:   mp / fnn / 1000.0
		ve:        s.eng.volumetric_efficiency()
		peak_bar:  peak / 1e5
		air_gs:    air / fnn * 1000.0
	}
}

// trace_cycle holds the crank at `rpm` and `pedal`, settles, then records one
// 720-degree cycle of cylinder `cyl` as CSV text (for p-V diagrams).
pub fn (mut s Simulator) trace_cycle(rpm f64, pedal f64, cyl int) string {
	s.dyno(rpm, pedal, 0.4, 0.01)
	s.frozen = true
	s.hold_omega = util.rpm2rads(rpm)
	mut out := []string{}
	out << 'phi_deg,volume_cc,p_bar,T_K,mass_mg,burned,in_lift_mm,ex_lift_mm,iport_bar,eport_bar,plenum_bar,xb'
	steps := int(4.0 * math.pi / (s.hold_omega * s.dt))
	every := if steps > 1440 { steps / 1440 } else { 1 }
	for k in 0 .. steps {
		s.step()
		if k % every != 0 {
			continue
		}
		c := s.eng.cyl[cyl]
		out << '${c.phi:.2f},${c.gas.v * 1e6:.3f},${c.gas.pressure() / 1e5:.4f},${c.gas.temperature():.1f},${c.gas.m * 1e6:.3f},${c.gas.burned_fraction():.3f},${c.in_lift * 1000:.3f},${c.ex_lift * 1000:.3f},${s.intake.net.ports[cyl].pressure() / 1e5:.4f},${s.exhaust.ports[cyl].pressure() / 1e5:.4f},${s.intake.plenum_pressure() / 1e5:.4f},${c.xb:.4f}'
	}
	s.frozen = false
	return out.join('\n')
}
