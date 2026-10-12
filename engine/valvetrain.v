module engine

import math
import config

// Resolution of the valve tables (entries per crank degree).
pub const valve_res = 4

const valve_table_len = 720 * valve_res + 1

// Cam describes one valve event in cycle coordinates (deg, 0 = firing TDC).
pub struct Cam {
pub:
	open_at  f64 // cycle angle where the valve leaves its seat
	close_at f64 // cycle angle where it closes (may exceed 720 only if open_at does)
	lift     f64
	shape    f64
}

// Valvetrain holds precomputed lift and effective flow area tables for the
// intake and exhaust valves of a cylinder (all cylinders share one cam
// design; they differ only by phase).
pub struct Valvetrain {
pub:
	intake  Cam
	exhaust Cam
pub mut:
	in_lift  []f64 // m
	ex_lift  []f64
	in_area  []f64 // effective area Cd*A (m^2), all valves of the cylinder
	ex_area  []f64
}

// intake_cam converts spec-sheet timing (open BTDC, close ABDC) to a Cam.
pub fn intake_cam(v config.ValveConfig) Cam {
	return Cam{
		open_at:  360.0 - v.open
		close_at: 540.0 + v.close
		lift:     v.lift
		shape:    v.profile
	}
}

// exhaust_cam converts spec-sheet timing (open BBDC, close ATDC) to a Cam.
pub fn exhaust_cam(v config.ValveConfig) Cam {
	return Cam{
		open_at:  180.0 - v.open
		close_at: 360.0 + v.close
		lift:     v.lift
		shape:    v.profile
	}
}

// lift_at returns the valve lift at cycle angle `phi` (deg). The profile is
// sin(pi*u)^shape over the open period: zero velocity at the seat, finite
// acceleration, and boxier for smaller `shape`.
pub fn (c &Cam) lift_at(phi f64) f64 {
	mut x := phi
	dur := c.close_at - c.open_at
	if x < c.open_at {
		x += 720.0
	}
	u := (x - c.open_at) / dur
	if u <= 0 || u >= 1 {
		return 0.0
	}
	return c.lift * math.pow(math.sin(math.pi * u), c.shape)
}

// curtain_area returns the effective flow area for lift `l`.
pub fn curtain_area(v config.ValveConfig, l f64) f64 {
	dp := if v.port_diameter > 0 { v.port_diameter } else { 0.88 * v.diameter }
	curtain := math.pi * v.diameter * l
	port := math.pi * dp * dp * 0.25
	return v.cd * f64(v.count) * (if curtain < port { curtain } else { port })
}

pub fn new_valvetrain(e config.EngineConfig) Valvetrain {
	ic := intake_cam(e.intake_valve)
	xc := exhaust_cam(e.exhaust_valve)
	mut vt := Valvetrain{
		intake:  ic
		exhaust: xc
		in_lift: []f64{len: valve_table_len}
		ex_lift: []f64{len: valve_table_len}
		in_area: []f64{len: valve_table_len}
		ex_area: []f64{len: valve_table_len}
	}
	for i in 0 .. valve_table_len {
		phi := f64(i) / f64(valve_res)
		li := ic.lift_at(phi)
		le := xc.lift_at(phi)
		vt.in_lift[i] = li
		vt.ex_lift[i] = le
		vt.in_area[i] = curtain_area(e.intake_valve, li)
		vt.ex_area[i] = curtain_area(e.exhaust_valve, le)
	}
	return vt
}

// lookup interpolates a table at cycle angle `phi` (deg, 0..720).
@[direct_array_access; inline]
pub fn lookup(t []f64, phi f64) f64 {
	x := phi * f64(valve_res)
	mut i := int(x)
	if i < 0 {
		i = 0
	}
	if i >= valve_table_len - 1 {
		return t[valve_table_len - 1]
	}
	f := x - f64(i)
	return t[i] + (t[i + 1] - t[i]) * f
}

// overlap returns the number of degrees both valves are open.
pub fn (vt &Valvetrain) overlap() f64 {
	mut n := 0
	for i in 0 .. valve_table_len - 1 {
		if vt.in_lift[i] > 0 && vt.ex_lift[i] > 0 {
			n++
		}
	}
	return f64(n) / f64(valve_res)
}
