module engine

import math
import config
import util

// EcuInputs are the driver/vehicle-side inputs the control unit sees.
pub struct EcuInputs {
pub mut:
	pedal           f64 // 0..1
	ignition        bool
	limiter_enabled bool = true
	launch_active   bool // launch control engaged (stationary, clutch in, armed)
	blip            f64  // throttle added by rev-matching, 0..1
}

// Ecu is a simple engine control unit: throttle by wire, idle air control,
// spark map, fuelling, rev limiter, launch control and overrun fuel cut.
pub struct Ecu {
pub mut:
	cfg config.EngineConfig
	// outputs
	plate        f64 // throttle plate opening 0..1
	bypass       f64 // idle valve opening 0..1
	spark        f64 // advance, deg BTDC
	afr          f64
	limiter_cut  bool
	fuel_cut     bool // overrun / limiter fuel cut
	spark_cut    bool
	retard_crack bool // overrun with afterfire tune: fuel on, spark very late
	limit_rpm    f64
	// state
	integral f64
	running  bool
	overrun  bool
	rpm_avg  f64 // crank speed averaged over ~one revolution
	dash     f64 // current dashpot opening
	rpm_rate f64 // its rate of change (rpm/s)
}

pub fn new_ecu(cfg config.EngineConfig) Ecu {
	return Ecu{
		cfg:    cfg
		spark:  cfg.combustion.spark_idle
		afr:    cfg.combustion.afr
		bypass: cfg.idle.base
	}
}

// update runs the control laws. `map_ratio` is manifold pressure / ambient.
pub fn (mut e Ecu) update(dt f64, rpm f64, map_ratio f64, inp EcuInputs) {
	c := e.cfg
	idle := c.idle_rpm
	pedal := util.clamp(inp.pedal + inp.blip, 0.0, 1.0)
	e.plate = pedal
	e.running = rpm > idle * 0.5
	// --- idle air control ---
	// PI(D) on the normalised rpm error in the idle band; above it (overrun or
	// on the pedal) the valve holds its learned position plus a dashpot so
	// the manifold doesn't go to hard vacuum and the revs fall back gently.
	// The crank speed ripples with every firing; the controller works on a
	// speed averaged over roughly one revolution.
	tau := util.clamp(60.0 / math.max(rpm, 300.0), 0.02, 0.15)
	prev := e.rpm_avg
	e.rpm_avg += (rpm - e.rpm_avg) * util.clamp(dt / tau, 0.0, 1.0)
	rate := if dt > 0 { (e.rpm_avg - prev) / dt } else { 0.0 }
	e.rpm_rate += (rate - e.rpm_rate) * util.clamp(dt / 0.05, 0.0, 1.0)
	err := util.clamp((idle - e.rpm_avg) / idle, -0.3, 0.5)
	off_pedal := pedal < 0.02 && inp.ignition
	in_idle := off_pedal && e.rpm_avg < idle + 500.0
	// dashpot: extra air while on the pedal, bled off ~0.8 s after lifting
	if off_pedal {
		e.dash -= e.dash * util.clamp(dt / 0.8, 0.0, 1.0)
	} else {
		e.dash = c.idle.dashpot
	}
	if off_pedal {
		// The integrator only learns inside the idle band (no wind-up on
		// overrun); the proportional and damping terms always act, so a
		// too-generous base opening can't hang the idle high.
		if in_idle {
			e.integral += err * dt * c.idle.ki
			e.integral = util.clamp(e.integral, -c.idle.base * 0.95, 1.0 - c.idle.base)
		}
		damp := -c.idle.kd * e.rpm_rate / idle
		e.bypass = util.clamp(c.idle.base + c.idle.kp * err + e.integral + damp + e.dash, 0.02,
			1.0)
	} else {
		e.bypass = util.clamp(c.idle.base + e.integral + e.dash, 0.02, 1.0)
	}
	// --- spark advance ---
	load := util.clamp(map_ratio, 0.0, 1.2)
	mut adv := 0.0
	if c.combustion.spark_mode == 'map' {
		t := util.clamp((rpm - idle) / (c.combustion.spark_full_rpm - idle), 0.0, 1.0)
		adv = c.combustion.spark_idle + (c.combustion.spark_max - c.combustion.spark_idle) * t
		adv -= c.combustion.spark_load_retard * util.clamp((load - 0.4) / 0.6, 0.0, 1.0)
	} else {
		adv = e.mbt_advance(rpm, load)
	}
	if in_idle {
		// Idle runs retarded from best torque, keeping a torque reserve the
		// spark can use instantly: advance when slow, retard when fast.
		reserve := c.idle.spark_reserve
		fast := c.idle.spark_gain * (idle - e.rpm_avg) / 100.0 - 0.05 * c.idle.spark_gain * e.rpm_rate / 100.0
		adv += -reserve + util.clamp(fast, -reserve - 6.0, reserve)
	}
	e.spark = adv
	// --- fuelling ---
	wot := util.clamp((pedal - 0.75) / 0.2, 0.0, 1.0)
	e.afr = c.combustion.afr + (c.combustion.afr_wot - c.combustion.afr) * wot
	// --- rev limiter (with launch control) ---
	e.limit_rpm = if inp.launch_active { c.limiter.launch_rpm } else { c.limiter.rpm }
	if inp.limiter_enabled {
		if rpm > e.limit_rpm {
			e.limiter_cut = true
		} else if rpm < e.limit_rpm - c.limiter.hysteresis {
			e.limiter_cut = false
		}
	} else {
		e.limiter_cut = false
	}
	// --- overrun: fuel cut, or crackle map when an afterfire tune is set ---
	overrun_on := pedal < 0.01 && rpm > idle + 1000
	overrun_off := pedal >= 0.01 || rpm < idle + 400
	if overrun_on {
		e.overrun = true
	} else if overrun_off {
		e.overrun = false
	}
	e.retard_crack = e.overrun && c.combustion.afterfire > 0
	overrun_cut := e.overrun && c.combustion.overrun_fuel_cut && !e.retard_crack
	limiter_fuel := e.limiter_cut && c.limiter.mode == 'fuel_cut'
	e.fuel_cut = !inp.ignition || overrun_cut || limiter_fuel
	e.spark_cut = !inp.ignition || (e.limiter_cut && c.limiter.mode == 'spark_cut')
	if e.retard_crack {
		e.spark = -25.0 // fire well after TDC so the burn spills into the exhaust
	}
}

// mbt_advance returns the spark advance (deg BTDC) that puts 50% mass
// burned at the target angle after TDC, using the same burn-rate model as
// the cylinders (minimum advance for best torque), minus a knock-limit
// retard at high load and low speed.
pub fn (e &Ecu) mbt_advance(rpm f64, load f64) f64 {
	cc := e.cfg.combustion
	b_est := util.clamp(0.28 - 0.24 * load, 0.04, 0.3)
	dur := cc.burn_duration * (1.0 + 1.6 * b_est) * (1.0 + 0.2 * util.clamp(rpm / 6000.0, 0.0, 1.5))
	delay := 2.0 + 0.0021 * math.abs(rpm)
	u50 := math.pow(math.ln2 / cc.wiebe_a, 1.0 / (cc.wiebe_m + 1.0))
	ca50 := util.lerp(cc.ca50_light, cc.ca50_full, util.clamp((load - 0.3) / 0.7, 0.0, 1.0))
	knock := cc.spark_load_retard * util.clamp((load - 0.7) / 0.3, 0.0, 1.0) * util.clamp((4000.0 - rpm) / 3000.0, 0.0, 1.0)
	return delay + u50 * dur - ca50 - knock
}
