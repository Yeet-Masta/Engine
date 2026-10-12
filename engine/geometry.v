// Module engine models the reciprocating engine: slider-crank kinematics,
// valve trains, the per-cylinder thermodynamic cycle with Wiebe combustion,
// crankshaft dynamics and the engine control unit.
module engine

import math
import config
import util

// Geometry holds the slider-crank dimensions of one cylinder.
pub struct Geometry {
pub:
	bore    f64
	stroke  f64
	rod     f64
	cr      f64
	crank_r f64
	area    f64 // piston area (m^2)
	v_swept f64 // per cylinder (m^3)
	v_clear f64 // clearance volume (m^3)
}

pub fn new_geometry(bore f64, stroke f64, rod f64, cr f64) Geometry {
	area := math.pi * bore * bore * 0.25
	vs := area * stroke
	return Geometry{
		bore:    bore
		stroke:  stroke
		rod:     rod
		cr:      cr
		crank_r: stroke * 0.5
		area:    area
		v_swept: vs
		v_clear: vs / (cr - 1.0)
	}
}

// kinematics returns the piston displacement from TDC `s` and its first and
// second derivatives with respect to the crank angle `alpha` (rad from TDC).
@[inline]
pub fn (g &Geometry) kinematics(alpha f64) (f64, f64, f64) {
	r := g.crank_r
	l := g.rod
	sa := math.sin(alpha)
	ca := math.cos(alpha)
	q := math.sqrt(l * l - r * r * sa * sa)
	s := r + l - (r * ca + q)
	s1 := r * sa + r * r * sa * ca / q
	s2 := r * ca + r * r * (ca * ca - sa * sa) / q + r * r * r * r * sa * sa * ca * ca / (q * q * q)
	return s, s1, s2
}

// volume returns the cylinder volume at crank angle `alpha` (rad from TDC).
@[inline]
pub fn (g &Geometry) volume(alpha f64) f64 {
	s, _, _ := g.kinematics(alpha)
	return g.v_clear + g.area * s
}

// tdc_angles returns, per cylinder, the crank angle (deg, 0..360) at which the
// piston is at top dead centre.
//
// Convention: a throw's angle is the crank rotation at which that crankpin
// passes the top (12 o'clock); a bank's angle is measured from vertical in
// the direction of rotation. The pin then lines up with the bank axis, i.e.
// the piston is at TDC, at crank angle `throw + bank`.
pub fn tdc_angles(e config.EngineConfig) []f64 {
	mut out := []f64{len: e.cylinders.len}
	for i, c in e.cylinders {
		out[i] = util.wrap(e.throws[c.throw] + e.banks[c.bank].angle, 360.0)
	}
	return out
}

// firing_angles returns, per cylinder (0-based), the crank angle in the
// 720-degree cycle at which that cylinder reaches its firing (compression)
// TDC.
//
// Each cylinder passes TDC twice per cycle; which one is the firing TDC is
// chosen so that cylinders fire in `firing_order` once per cycle, and among
// the consistent choices the most even spacing is picked (that is how real
// engines are designed; explicit `firing_angles` override it for oddballs).
pub fn firing_angles(e config.EngineConfig) ![]f64 {
	n := e.cylinders.len
	order := e.firing_order.map(it - 1)
	mut result := []f64{len: n}
	if e.firing_angles.len == n {
		for k, cyl in order {
			result[cyl] = util.wrap(e.firing_angles[k], 720.0)
		}
		return result
	}
	tdc := tdc_angles(e)
	if n == 1 {
		result[0] = tdc[0]
		return result
	}
	ideal := 720.0 / f64(n)
	mut best_score := math.inf(1)
	mut best_mask := -1
	combos := 1 << (n - 1)
	mut f := []f64{len: n}
	for mask in 0 .. combos {
		f[0] = tdc[order[0]]
		for k in 1 .. n {
			f[k] = tdc[order[k]] + (if (mask >> (k - 1)) & 1 == 1 { 360.0 } else { 0.0 })
		}
		mut total := 0.0
		mut score := 0.0
		mut ok := true
		for k in 0 .. n {
			gap := util.wrap(f[(k + 1) % n] - f[k], 720.0)
			if gap < 0.5 {
				ok = false
				break
			}
			total += gap
			score += (gap - ideal) * (gap - ideal)
		}
		if !ok || math.abs(total - 720.0) > 1e-6 {
			continue
		}
		if score < best_score - 1e-9 {
			best_score = score
			best_mask = mask
		}
	}
	if best_mask < 0 {
		return error('firing order ${e.firing_order} cannot be realised with these banks and throws (two cylinders would fire together); check engine.throws/banks or give engine.firing_angles')
	}
	for k in 0 .. n {
		extra := if k > 0 && (best_mask >> (k - 1)) & 1 == 1 { 360.0 } else { 0.0 }
		result[order[k]] = util.wrap(tdc[order[k]] + extra, 720.0)
	}
	return result
}

// firing_intervals returns the gaps (deg) between consecutive firings, in
// firing order, starting with the first cylinder of the order.
pub fn firing_intervals(e config.EngineConfig, angles []f64) []f64 {
	order := e.firing_order.map(it - 1)
	n := order.len
	mut out := []f64{len: n}
	for k in 0 .. n {
		out[k] = util.wrap(angles[order[(k + 1) % n]] - angles[order[k]], 720.0)
		if n == 1 {
			out[k] = 720.0
		}
	}
	return out
}
