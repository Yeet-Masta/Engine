// Module drivetrain connects the crankshaft to the vehicle: clutch, gearbox
// with synchronisers and shift timing, final drive, a compliant driveline and
// the driven wheels with tyre friction.
//
// All friction couplings (clutch, synchro, tyre, brakes) use the
// discrete-time friction method: compute the torque that would make both
// sides move together at the end of the step and clamp it to the coupling's
// capacity. That resolves slip, lock-up and break-away exactly with no
// chatter at any timestep.
module drivetrain

import math
import config
import vehicle
import util

pub enum ShiftState {
	engaged
	shifting
	syncing
	neutral
}

pub struct Drivetrain {
pub mut:
	cfg   config.DrivetrainConfig
	gear  int // -1 reverse, 0 neutral, 1..n
	state ShiftState = .neutral
	timer f64
	w_in  f64 // gearbox input shaft (rad/s)
	w_whl f64 // driven wheels (rad/s)
	twist f64 // driveline twist referred to the wheels (rad)
	// clutch engagement 0 (pedal down) .. 1 (pedal up), set by the caller
	engage f64 = 1.0
	// outputs
	t_clutch    f64 // torque the clutch takes from the engine (N*m)
	t_shaft     f64 // driveline torque at the wheels (N*m)
	t_sync      f64
	clutch_slip f64 // rad/s
	gear_phase  f64 // input gear mesh phase, for whine
	mesh_load   f64 // |input torque| for whine level
	grind       f64 // synchro working against a turning engine
}

pub fn new_drivetrain(c config.DrivetrainConfig) Drivetrain {
	return Drivetrain{
		cfg: c
	}
}

// gear_ratio returns the gearbox ratio for gear `g` (0 for neutral).
pub fn (d &Drivetrain) gear_ratio(g int) f64 {
	if g == 0 {
		return 0.0
	}
	if g < 0 {
		return -d.cfg.reverse_ratio
	}
	return d.cfg.gear_ratios[g - 1]
}

// total_ratio returns gearbox x final drive for the selected gear.
pub fn (d &Drivetrain) total_ratio() f64 {
	return d.gear_ratio(d.gear) * d.cfg.final_drive
}

pub fn (d &Drivetrain) gear_count() int {
	return d.cfg.gear_ratios.len
}

// select starts a shift to gear `g`.
pub fn (mut d Drivetrain) select(g int) {
	target := util.clamp(f64(g), -1.0, f64(d.gear_count()))
	tg := int(target)
	if tg == d.gear && d.state != .neutral {
		return
	}
	d.gear = tg
	d.state = .shifting
	d.timer = d.cfg.shift_time
	d.twist = 0
}

pub fn (mut d Drivetrain) shift_up() {
	if d.gear < d.gear_count() {
		d.select(d.gear + 1)
	}
}

pub fn (mut d Drivetrain) shift_down() {
	if d.gear > -1 {
		d.select(d.gear - 1)
	}
}

pub fn (d &Drivetrain) in_gear() bool {
	return d.state == .engaged && d.gear != 0
}

// clutch_capacity returns the torque capacity for the current engagement.
pub fn (d &Drivetrain) clutch_capacity() f64 {
	x := util.clamp((d.engage - 0.08) / 0.84, 0.0, 1.0)
	return d.cfg.clutch_max_torque * x * math.sqrt(x)
}

// friction returns the torque body 1 passes to body 2 so that both reach the
// same speed after dt, limited to +-cap.
@[inline]
pub fn friction(w1 f64, w2 f64, t1 f64, t2 f64, i1 f64, i2 f64, cap f64, dt f64) f64 {
	if cap <= 0 {
		return 0.0
	}
	t := (w1 - w2 + dt * (t1 / i1 - t2 / i2)) / (dt * (1.0 / i1 + 1.0 / i2))
	return util.clamp(t, -cap, cap)
}

// step advances the drivetrain and vehicle by dt. `w_e`, `t_e`, `i_e` are the
// engine speed, net torque (without clutch) and effective inertia. Returns the
// clutch torque to subtract from the engine.
pub fn (mut d Drivetrain) step(dt f64, w_e f64, t_e f64, i_e f64, mut veh vehicle.Vehicle) f64 {
	c := d.cfg
	vc := veh.cfg
	r := vc.wheel_radius
	i_in := c.input_inertia
	i_w := vc.wheel_inertia
	m := vc.mass
	// --- shift sequencing ---
	match d.state {
		.shifting {
			d.timer -= dt
			if d.timer <= 0 {
				d.state = if d.gear == 0 { ShiftState.neutral } else { ShiftState.syncing }
			}
		}
		.syncing {
			ratio := d.total_ratio()
			if math.abs(d.w_in - d.w_whl * ratio) < 1.5 {
				d.state = .engaged
				d.twist = 0
			}
		}
		else {}
	}
	ratio := d.total_ratio()
	engaged := d.state == .engaged && d.gear != 0
	// --- driveline spring torque (at the wheels) ---
	mut t_s := 0.0
	if engaged {
		t_s = c.shaft_stiffness * d.twist + c.shaft_damping * (d.w_in / ratio - d.w_whl)
	} else {
		d.twist *= 1.0 - dt * 20.0
	}
	t_in_spring := if engaged { -t_s / ratio } else { 0.0 }
	// gearbox losses and input drag
	mut drag := 0.3 * math.tanh(d.w_in / 5.0) + 0.002 * d.w_in
	if engaged {
		drag += (1.0 - c.efficiency) * math.abs(t_s / ratio) * math.tanh(d.w_in / 5.0)
	}
	// --- clutch ---
	tc := friction(w_e, d.w_in, t_e, t_in_spring - drag, i_e, i_in, d.clutch_capacity(), dt)
	d.t_clutch = tc
	d.clutch_slip = w_e - d.w_in
	// --- synchroniser: drags the input shaft to the new gear's speed ---
	mut t_sync := 0.0
	d.grind = 0
	if d.state == .syncing {
		target := d.w_whl * ratio
		others := tc - drag
		need := (target - d.w_in) * i_in / dt - others
		t_sync = util.clamp(need, -c.synchro_torque, c.synchro_torque)
		if d.clutch_capacity() > 1.0 {
			d.grind = math.abs(target - d.w_in) * d.clutch_capacity() / c.clutch_max_torque
		}
	}
	d.t_sync = t_sync
	// --- tyre: wheel surface vs vehicle body ---
	t_w_other := t_s - t_sync * ratio
	f_res := veh.resistance()
	slip := d.w_whl * r - veh.v
	cap_t := veh.tyre_capacity(slip)
	f_req := (slip + dt * (t_w_other * r / i_w + f_res / m)) / (dt * (r * r / i_w + 1.0 / m))
	f_t := util.clamp(f_req, -cap_t, cap_t)
	// --- brakes: body vs ground ---
	v_pred := veh.v + dt * (f_t - f_res) / m
	f_b := util.clamp(v_pred * m / dt, -veh.brake_force_capacity(), veh.brake_force_capacity())
	// --- integrate ---
	d.w_in += dt * (tc + t_in_spring + t_sync - drag) / i_in
	d.w_whl += dt * (t_w_other - f_t * r) / i_w
	acc := (f_t - f_res - f_b) / m
	veh.v += dt * acc
	veh.x += dt * veh.v
	veh.a += (acc - veh.a) * dt * 20.0
	veh.tire_force = f_t
	veh.slip_speed = slip
	if engaged {
		d.twist += dt * (d.w_in / ratio - d.w_whl)
	}
	d.t_shaft = t_s
	d.gear_phase += d.w_in * f64(c.gear_teeth) * dt
	if d.gear_phase > 2.0 * math.pi * 1000.0 {
		d.gear_phase -= 2.0 * math.pi * 1000.0
	}
	d.mesh_load = if engaged { math.abs(t_s / ratio) } else { 0.0 }
	return tc
}

// wheel_rpm_to_engine returns the engine speed (rad/s) that matches the
// current wheel speed in gear `g`.
pub fn (d &Drivetrain) matched_engine_speed(g int) f64 {
	return d.w_whl * d.gear_ratio(g) * d.cfg.final_drive
}
