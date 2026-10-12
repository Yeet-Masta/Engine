module engine

import math
import config
import gas
import util

pub const max_cylinders = 16
const wiebe_len = 512
const four_pi = 4.0 * math.pi
const rad2deg = 180.0 / math.pi

// Cylinder is the thermodynamic and combustion state of one cylinder.
pub struct Cylinder {
pub mut:
	gas  gas.Volume
	fire f64 // cycle angle of firing TDC (rad)
	phi  f64 // current cycle angle (deg, 0 = firing TDC)
	// combustion
	armed       bool // spark armed for this cycle
	spark_at    f64  // cycle angle of the last spark (deg)
	burning     bool
	q_total     f64 // heat to release this cycle (J)
	m_burn      f64 // mixture mass converted to products this cycle (kg)
	burn_delay  f64 // deg after spark
	burn_dur    f64 // deg
	xb          f64 // Wiebe mass fraction burned
	unburned    f64 // fuel (kg) left unburned in the charge -> afterfire
	crackle     bool
	pop_timer   f64
	pop_rate    f64
	misfired    bool
	ex_lift_was bool
	in_lift_was bool
	h_wall      f64
	trapped     f64 // mass at the last intake closing (kg)
	residual    f64 // burned fraction at the last spark
	// outputs
	torque  f64
	p_prev  f64
	in_lift f64
	ex_lift f64
}

// Engine owns cylinders, crank and ECU. It advances gas exchange and
// combustion; the crank speed is integrated by `integrate` once the
// drivetrain has computed the clutch torque.
pub struct Engine {
pub mut:
	cfg      config.EngineConfig
	geom     Geometry
	vt       Valvetrain
	ecu      Ecu
	cyl      []Cylinder
	table    gas.FlowTable
	wiebe    []f64
	rng      util.Rng
	p_amb    f64
	t_amb    f64
	theta    f64 // crank angle in the 720-degree cycle (rad)
	omega    f64 // rad/s
	inertia  f64
	m_rec    f64
	v_disp   f64
	fire_deg []f64
	// per-step results
	t_gas     f64 // gas torque (N*m)
	rec_coef  f64 // sum m*s'*s''  (torque = -rec_coef * omega^2)
	i_eff     f64 // effective inertia incl. reciprocating mass
	t_fric    f64
	t_starter f64
	starter   bool
	// audio side-signals (per step)
	block_signal f64 // summed cylinder pressure change (Pa)
	valve_click  f64
	pop_signal   f64
	// telemetry averages
	torque_avg f64 // brake torque, cycle averaged (N*m)
	ind_avg    f64 // indicated torque, cycle averaged (N*m)
	ve         f64 // volumetric efficiency of the last cycles
	fires      u64
	misfires   u64
	steps      u64
}

// new_engine builds an engine from a config.
pub fn new_engine(c config.Config) !Engine {
	e := c.engine
	fire := firing_angles(e)!
	geom := new_geometry(e.bore, e.stroke, e.rod_length, e.compression_ratio)
	mut eng := Engine{
		cfg:      e
		geom:     geom
		vt:       new_valvetrain(e)
		ecu:      new_ecu(e)
		table:    gas.new_flow_table()
		rng:      util.new_rng(c.simulation.seed)
		p_amb:    c.simulation.ambient_pressure
		t_amb:    c.simulation.ambient_temperature
		inertia:  e.inertia
		m_rec:    e.reciprocating_mass
		v_disp:   e.displacement()
		fire_deg: fire
	}
	eng.wiebe = []f64{len: wiebe_len + 1}
	for i in 0 .. wiebe_len + 1 {
		u := f64(i) / f64(wiebe_len)
		eng.wiebe[i] = 1.0 - math.exp(-e.combustion.wiebe_a * math.pow(u, e.combustion.wiebe_m + 1.0))
	}
	// normalise so the burn completes exactly
	last := eng.wiebe[wiebe_len]
	for i in 0 .. wiebe_len + 1 {
		eng.wiebe[i] /= last
	}
	for i in 0 .. e.cylinders.len {
		eng.cyl << Cylinder{
			fire: fire[i] * math.pi / 180.0
		}
	}
	eng.reset_gas()
	return eng
}

// reset_gas fills every cylinder with ambient air at its current volume.
pub fn (mut e Engine) reset_gas() {
	for mut c in e.cyl {
		phi := util.wrap(e.theta - c.fire, four_pi) * rad2deg
		alpha := util.wrap(phi * math.pi / 180.0, 2.0 * math.pi)
		v := e.geom.volume(alpha)
		c.gas = gas.new_volume(v, e.p_amb, e.t_amb, 0.05)
		c.phi = phi
		c.armed = phi >= 360.0 && phi < 700.0
		c.p_prev = e.p_amb
		c.burning = false
	}
}

@[direct_array_access; inline]
fn (e &Engine) wiebe_at(u f64) f64 {
	if u <= 0 {
		return 0.0
	}
	if u >= 1 {
		return 1.0
	}
	x := u * f64(wiebe_len)
	i := int(x)
	f := x - f64(i)
	return e.wiebe[i] + (e.wiebe[i + 1] - e.wiebe[i]) * f
}

// rpm returns the crank speed in rpm.
pub fn (e &Engine) rpm() f64 {
	return e.omega * 30.0 / math.pi
}

// step_gas advances gas exchange and combustion for every cylinder by `dt`
// and computes the gas and inertia torques. Valve flows are written into the
// port volumes' accumulators; the networks consume them afterwards.
@[direct_array_access]
pub fn (mut e Engine) step_gas(dt f64, mut iports []gas.Volume, mut eports []gas.Volume) {
	omega := e.omega
	dphi := omega * dt * rad2deg
	rpm := omega * 30.0 / math.pi
	cc := e.cfg.combustion
	spark_phi := 720.0 - e.ecu.spark
	update_wall := e.steps % 4 == 0
	e.steps++
	mut t_gas := 0.0
	mut rec := 0.0
	mut i_extra := 0.0
	mut block := 0.0
	mut click := 0.0
	mut pop := 0.0
	geom := e.geom
	sp := 2.0 * geom.stroke * math.abs(rpm) / 60.0 // mean piston speed
	for i in 0 .. e.cyl.len {
		mut c := &e.cyl[i]
		phi0 := c.phi
		mut phi1 := phi0 + dphi
		if phi1 >= 720.0 {
			phi1 -= 720.0
		}
		c.phi = phi1
		alpha1 := (if phi1 >= 360.0 { phi1 - 360.0 } else { phi1 }) * math.pi / 180.0
		s, s1, s2 := geom.kinematics(alpha1)
		v1 := geom.v_clear + geom.area * s
		// --- valves ---
		a_in := lookup(e.vt.in_area, phi0)
		a_ex := lookup(e.vt.ex_area, phi0)
		c.in_lift = lookup(e.vt.in_lift, phi0)
		c.ex_lift = lookup(e.vt.ex_lift, phi0)
		in_open := c.in_lift > 0
		ex_open := c.ex_lift > 0
		if c.in_lift_was && !in_open {
			click += math.abs(omega) / 300.0
			c.trapped = c.gas.m
		}
		if c.ex_lift_was && !ex_open {
			click += math.abs(omega) / 300.0
		}
		if ex_open && !c.ex_lift_was {
			// exhaust valve just opened: unburned charge may light in the port
			if c.unburned > 0 && cc.afterfire > 0 {
				chance := if c.crackle { 0.45 } else { 0.7 }
				if e.rng.uniform() < cc.afterfire * chance {
					energy := c.unburned * cc.fuel_lhv * (0.15 + 0.25 * e.rng.uniform())
					c.pop_timer = 0.0015
					c.pop_rate = energy / c.pop_timer
				}
			}
			c.unburned = 0
			c.crackle = false
		}
		c.in_lift_was = in_open
		c.ex_lift_was = ex_open
		if in_open {
			e.table.transfer(a_in, dt, mut c.gas, mut iports[i])
		}
		if ex_open {
			e.table.transfer(a_ex, dt, mut c.gas, mut eports[i])
		}
		if c.pop_timer > 0 {
			eports[i].acc_e += c.pop_rate * dt
			pop += c.pop_rate * dt
			c.pop_timer -= dt
		}
		// --- spark: armed during the intake stroke, fires once when the
		// (moving) spark angle has been passed ---
		if phi1 >= 360.0 && phi1 < 540.0 {
			c.armed = true
		}
		if c.armed && util.wrap(spark_phi - phi1, 720.0) > 360.0 {
			c.armed = false
			c.spark_at = phi1
			e.ignite(mut c, rpm)
		}
		// --- combustion heat release (Wiebe), timed from the latched spark ---
		mut dq := 0.0
		if c.burning {
			since := util.wrap(phi1 - c.spark_at, 720.0)
			u := (since - c.burn_delay) / c.burn_dur
			if u > 0 {
				xb := e.wiebe_at(u)
				dx := xb - c.xb
				c.xb = xb
				dq = c.q_total * dx
				c.gas.mb += c.m_burn * dx
				if u >= 1.0 {
					c.burning = false
				}
			}
		}
		// --- wall heat transfer (Woschni-style, coefficient refreshed every 4 steps) ---
		t_gas_k := c.gas.temperature()
		if update_wall {
			p_kpa := c.gas.pressure() * 1e-3
			w := 2.28 * (if sp < 0.5 { 0.5 } else { sp }) * (if c.burning { 1.5 } else { 1.0 })
			c.h_wall = e.cfg.heat_transfer * 3.26 * math.pow(geom.bore, -0.2) * math.pow(p_kpa, 0.8) * math.pow(t_gas_k, -0.55) * math.pow(w, 0.8)
		}
		area_w := 2.2 * geom.area + math.pi * geom.bore * s
		mut dq_w := c.h_wall * area_w * (t_gas_k - e.cfg.wall_temperature) * dt
		cap := 0.05 * c.gas.e
		if dq_w > cap {
			dq_w = cap
		} else if dq_w < -cap {
			dq_w = -cap
		}
		// --- energy: adiabatic work (2nd-order rational update) + heat + flows ---
		v0 := c.gas.v
		dv := v1 - v0
		k := c.gas.gamma() - 1.0
		x := dv / (0.5 * (v0 + v1))
		c.gas.e *= (1.0 - 0.5 * k * x) / (1.0 + 0.5 * k * x)
		c.gas.e += dq - dq_w
		c.gas.v = v1
		c.gas.commit()
		p := c.gas.pressure()
		block += p - c.p_prev
		c.p_prev = p
		// --- torque: T = (p - p_crankcase) * dV/dtheta ---
		tq := (p - e.p_amb) * geom.area * s1
		c.torque = tq
		t_gas += tq
		rec += e.m_rec * s1 * s2
		i_extra += e.m_rec * s1 * s1
	}
	e.t_gas = t_gas
	e.rec_coef = rec
	e.i_eff = e.inertia + i_extra
	e.block_signal = block
	e.valve_click = click
	e.pop_signal = pop
}

// ignite handles the spark event of a cylinder.
fn (mut e Engine) ignite(mut c Cylinder, rpm f64) {
	cc := e.cfg.combustion
	fresh := c.gas.m - c.gas.mb
	b_res := c.gas.burned_fraction()
	c.residual = b_res
	c.burning = false
	c.xb = 0
	c.misfired = false
	if fresh <= 0 || e.ecu.fuel_cut {
		c.unburned = 0
		return
	}
	afr := e.ecu.afr
	fuel := fresh / (1.0 + afr)
	air := fresh - fuel
	burnable := math.min(fuel, air / cc.afr_stoich)
	if e.ecu.spark_cut {
		c.unburned = fuel
		return
	}
	// Dilution by residual gas slows the flame and causes misfires at idle
	// with big overlap cams.
	p_misfire := util.clamp((b_res - 0.30) * 1.2, 0.0, 0.5)
	if e.rng.uniform() < p_misfire {
		c.misfired = true
		c.unburned = fuel
		e.misfires++
		return
	}
	var := cc.variation * (1.0 + 2.5 * b_res)
	mut q := burnable * cc.fuel_lhv * cc.efficiency * (1.0 + 0.4 * var * e.rng.gauss())
	if q < 0 {
		q = 0
	}
	mut dur := cc.burn_duration * (1.0 + 1.6 * b_res) * (1.0 + 0.2 * util.clamp(rpm / 6000.0, 0.0,
		1.5)) * (1.0 + var * e.rng.gauss())
	if e.ecu.retard_crack {
		dur *= 1.8
		c.crackle = true
		c.unburned = fuel * 0.3
	}
	if dur < 15 {
		dur = 15
	}
	c.q_total = q
	c.m_burn = burnable * (1.0 + cc.afr_stoich)
	if c.m_burn > fresh {
		c.m_burn = fresh
	}
	c.burn_delay = 2.0 + 0.0021 * math.abs(rpm)
	c.burn_dur = dur
	c.burning = true
	e.fires++
}

// mech_torques computes friction and starter torques for the current speed.
pub fn (mut e Engine) mech_torques(starter bool, starter_cfg config.StarterConfig) {
	f := e.cfg.friction
	rpm := e.omega * 30.0 / math.pi
	sp := 2.0 * e.geom.stroke * math.abs(rpm) / 60.0
	fmep := (f.a + f.b * sp + f.c * sp * sp) * 1000.0
	mag := fmep * e.v_disp / (4.0 * math.pi) + f.accessory
	e.t_fric = mag * math.tanh(e.omega / 3.0)
	e.starter = starter
	w_free := starter_cfg.free_rpm * math.pi / 30.0
	if starter && e.omega < w_free {
		e.t_starter = starter_cfg.torque * (1.0 - e.omega / w_free)
	} else {
		e.t_starter = 0
	}
}

// net_torque is the crank torque excluding the clutch.
@[inline]
pub fn (e &Engine) net_torque() f64 {
	return e.t_gas - e.rec_coef * e.omega * e.omega - e.t_fric + e.t_starter
}

// integrate advances the crank by `dt` given the clutch torque it delivers.
pub fn (mut e Engine) integrate(dt f64, t_clutch f64) {
	acc := (e.net_torque() - t_clutch) / e.i_eff
	e.omega += acc * dt
	if e.omega < -5.0 {
		e.omega = -5.0
	}
	e.theta += e.omega * dt
	if e.theta >= four_pi {
		e.theta -= four_pi
	} else if e.theta < 0 {
		e.theta += four_pi
	}
	// cycle-averaged torque for the readout (time constant ~ two cycles)
	rpm := math.abs(e.omega) * 30.0 / math.pi
	tau := math.max(0.06, 2.0 * 120.0 / math.max(rpm, 300.0))
	a := dt / tau
	brake := e.t_gas - e.t_fric
	e.ind_avg += a * (e.t_gas - e.ind_avg)
	e.torque_avg += a * (brake - e.torque_avg)
}

// sync_phase re-derives each cylinder's cycle angle from the crank angle
// (used after externally changing theta).
pub fn (mut e Engine) sync_phase() {
	for mut c in e.cyl {
		c.phi = util.wrap(e.theta - c.fire, four_pi) * rad2deg
	}
}

// volumetric_efficiency returns trapped mass / (ambient density x swept volume),
// averaged over cylinders.
pub fn (e &Engine) volumetric_efficiency() f64 {
	rho := e.p_amb / (gas.r_gas * e.t_amb)
	mut s := 0.0
	for c in e.cyl {
		s += c.trapped * (1.0 - c.residual)
	}
	return s / (f64(e.cyl.len) * rho * e.geom.v_swept)
}
