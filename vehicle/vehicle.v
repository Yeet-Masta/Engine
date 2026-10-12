// Module vehicle models the car body: mass, rolling resistance, aerodynamic
// drag, brakes, axle loads and tyre friction limits. The drivetrain couples
// the driven wheels to it.
module vehicle

import math
import config

pub const g = 9.81
pub const rho_air = 1.2

pub struct Vehicle {
pub mut:
	cfg config.VehicleConfig
	v   f64 // forward speed (m/s)
	x   f64 // distance travelled (m)
	a   f64 // last acceleration (m/s^2), used for weight transfer
	// last tyre force and slip, for telemetry and tyre noise
	tire_force f64
	slip_speed f64
	brake      f64 // 0..1
}

pub fn new_vehicle(c config.VehicleConfig) Vehicle {
	return Vehicle{
		cfg: c
	}
}

// resistance returns rolling + aero drag (N), opposing motion.
pub fn (v &Vehicle) resistance() f64 {
	c := v.cfg
	roll := c.rolling_resistance * c.mass * g * math.tanh(v.v / 0.3)
	aero := 0.5 * rho_air * c.drag_coefficient * c.frontal_area * v.v * math.abs(v.v)
	return roll + aero
}

// driven_load returns the normal force on the driven axle (N), including
// longitudinal weight transfer.
pub fn (v &Vehicle) driven_load() f64 {
	c := v.cfg
	w := c.mass * g
	transfer := c.mass * v.a * c.cg_height / c.wheelbase
	return match c.drive {
		'rwd' { math.max(0.1 * w, c.driven_axle_load * w + transfer) }
		'fwd' { math.max(0.1 * w, c.driven_axle_load * w - transfer) }
		else { w }
	}
}

// tyre_capacity returns the maximum tyre force (N) for slip speed `slip`
// (m/s): peak friction near zero slip falling to sliding friction.
pub fn (v &Vehicle) tyre_capacity(slip f64) f64 {
	c := v.cfg
	s := math.abs(slip)
	mu := c.tire_mu_slide + (c.tire_mu_peak - c.tire_mu_slide) * math.exp(-s / 1.5)
	return mu * v.driven_load()
}

// brake_force_capacity is the maximum braking force on the body (N).
pub fn (v &Vehicle) brake_force_capacity() f64 {
	return v.brake * v.cfg.brake_decel * v.cfg.mass
}

pub fn (v &Vehicle) speed_kmh() f64 {
	return v.v * 3.6
}
