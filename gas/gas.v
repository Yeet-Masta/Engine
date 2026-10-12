// Module gas implements lumped (0-D) ideal-gas control volumes and
// compressible orifice flow between them. Cylinders, ports, plenums and
// airboxes are all `Volume`s.
module gas

import math

pub const r_gas = 287.0 // J/(kg*K), air and burned gas are close enough
pub const gamma_fresh = 1.35 // air-fuel mixture
pub const gamma_burned = 1.27 // combustion products
pub const p_atm = 101325.0

// gamma_mix returns the ratio of specific heats for burned fraction `b`.
@[inline]
pub fn gamma_mix(b f64) f64 {
	return gamma_fresh + (gamma_burned - gamma_fresh) * b
}

// cp returns the specific heat at constant pressure (J/(kg*K)) for gamma `g`.
@[inline]
pub fn cp(g f64) f64 {
	return g / (g - 1.0) * r_gas
}

// Volume is a well-mixed control volume. State is mass, internal energy and
// burned mass, which keeps p = (gamma-1)*E/V free of transcendental calls.
// The `acc_*` fields collect mass/energy/burned-mass transfers from orifice
// flows during a step; whoever integrates the volume consumes them.
pub struct Volume {
pub mut:
	m  f64 // kg
	e  f64 // J
	mb f64 // kg of burned gas
	v  f64 // m^3
	// Accumulated transfers this step (kg, J, kg).
	acc_m  f64
	acc_e  f64
	acc_mb f64
}

// new_volume creates a volume of size `v` filled at pressure p, temperature t
// and burned fraction b.
pub fn new_volume(v f64, p f64, t f64, b f64) Volume {
	g := gamma_mix(b)
	m := p * v / (r_gas * t)
	return Volume{
		m:  m
		e:  m * r_gas * t / (g - 1.0)
		mb: m * b
		v:  v
	}
}

@[inline]
pub fn (vol &Volume) burned_fraction() f64 {
	if vol.m <= 0 {
		return 0.0
	}
	b := vol.mb / vol.m
	return if b < 0 {
		0.0
	} else if b > 1 {
		1.0
	} else {
		b
	}
}

@[inline]
pub fn (vol &Volume) gamma() f64 {
	return gamma_mix(vol.burned_fraction())
}

@[inline]
pub fn (vol &Volume) pressure() f64 {
	return (vol.gamma() - 1.0) * vol.e / vol.v
}

@[inline]
pub fn (vol &Volume) temperature() f64 {
	if vol.m <= 1e-12 {
		return 300.0
	}
	return (vol.gamma() - 1.0) * vol.e / (vol.m * r_gas)
}

@[inline]
pub fn (vol &Volume) density() f64 {
	return vol.m / vol.v
}

// specific_enthalpy returns h = cp*T = gamma * (E/m) (J/kg).
@[inline]
pub fn (vol &Volume) specific_enthalpy() f64 {
	return vol.gamma() * vol.e / vol.m
}

// stiffness returns dp/dm (Pa/kg) when mass at the volume's own temperature
// is added at constant volume: gamma*p/m.
@[inline]
pub fn (vol &Volume) stiffness() f64 {
	if vol.m <= 1e-12 {
		return 1e12
	}
	return vol.gamma() * vol.pressure() / vol.m
}

// commit applies the accumulated transfers and clears them.
@[inline]
pub fn (mut vol Volume) commit() {
	vol.m += vol.acc_m
	vol.e += vol.acc_e
	vol.mb += vol.acc_mb
	vol.acc_m = 0
	vol.acc_e = 0
	vol.acc_mb = 0
	vol.sanitize()
}

// sanitize keeps the state physical after large transfers.
@[inline]
pub fn (mut vol Volume) sanitize() {
	min_m := 1e-9 * vol.v
	if vol.m < min_m {
		vol.m = min_m
	}
	min_e := vol.m * r_gas * 150.0 / (gamma_fresh - 1.0)
	if vol.e < min_e {
		vol.e = min_e
	}
	if vol.mb < 0 {
		vol.mb = 0
	} else if vol.mb > vol.m {
		vol.mb = vol.m
	}
}

// --- orifice flow -----------------------------------------------------------

const table_size = 1024
const pr_linear = 0.985 // above this pressure ratio the flow is linearised

// FlowTable caches the dimensionless flow function psi(pr) for the fresh and
// burned gamma values, so valve flow needs no `pow`.
pub struct FlowTable {
mut:
	fresh  []f64
	burned []f64
}

// new_flow_table precomputes psi for both gammas.
pub fn new_flow_table() FlowTable {
	mut t := FlowTable{
		fresh:  []f64{len: table_size + 1}
		burned: []f64{len: table_size + 1}
	}
	for i in 0 .. table_size + 1 {
		pr := f64(i) / f64(table_size)
		t.fresh[i] = psi_exact(pr, gamma_fresh)
		t.burned[i] = psi_exact(pr, gamma_burned)
	}
	return t
}

// psi_exact is the isentropic nozzle flow function:
// mdot = A * p0 / sqrt(R*T0) * psi(p_down/p_up). It is linearised for
// pr > pr_linear to remove the infinite slope at pr = 1.
pub fn psi_exact(pr f64, g f64) f64 {
	if pr >= 1.0 {
		return 0.0
	}
	if pr > pr_linear {
		return psi_raw(pr_linear, g) * (1.0 - pr) / (1.0 - pr_linear)
	}
	return psi_raw(pr, g)
}

fn psi_raw(pr f64, g f64) f64 {
	crit := math.pow(2.0 / (g + 1.0), g / (g - 1.0))
	if pr <= crit {
		return math.sqrt(g) * math.pow(2.0 / (g + 1.0), (g + 1.0) / (2.0 * (g - 1.0)))
	}
	v := 2.0 * g / (g - 1.0) * (math.pow(pr, 2.0 / g) - math.pow(pr, (g + 1.0) / g))
	return if v > 0 { math.sqrt(v) } else { 0.0 }
}

// psi interpolates the table for pressure ratio pr and burned fraction b.
@[direct_array_access; inline]
pub fn (t &FlowTable) psi(pr f64, b f64) f64 {
	if pr >= 1.0 {
		return 0.0
	}
	x := (if pr < 0 { 0.0 } else { pr }) * f64(table_size)
	i := int(x)
	f := x - f64(i)
	j := if i >= table_size { table_size } else { i + 1 }
	pf := t.fresh[i] + (t.fresh[j] - t.fresh[i]) * f
	pb := t.burned[i] + (t.burned[j] - t.burned[i]) * f
	return pf + (pb - pf) * b
}

// mass_flow returns the signed mass flow (kg/s) from volume `a` to volume
// `b` through effective area `area` (Cd already applied).
@[inline]
pub fn (t &FlowTable) mass_flow(area f64, pa f64, ta f64, ba f64, pb f64, tb f64, bb f64) f64 {
	if area <= 0 {
		return 0.0
	}
	if pa >= pb {
		return area * pa / math.sqrt(r_gas * ta) * t.psi(pb / pa, ba)
	}
	return -area * pb / math.sqrt(r_gas * tb) * t.psi(pa / pb, bb)
}

// psi_slope returns psi(pr) and d(psi)/d(pr) for pressure ratio pr and
// burned fraction b (table interpolation).
@[direct_array_access; inline]
pub fn (t &FlowTable) psi_slope(pr f64, b f64) (f64, f64) {
	if pr >= 1.0 {
		return 0.0, 0.0
	}
	x := (if pr < 0 { 0.0 } else { pr }) * f64(table_size)
	i := int(x)
	f := x - f64(i)
	j := if i >= table_size { table_size } else { i + 1 }
	df := t.fresh[j] - t.fresh[i]
	db := t.burned[j] - t.burned[i]
	pf := t.fresh[i] + df * f
	pb := t.burned[i] + db * f
	return pf + (pb - pf) * b, (df + (db - df) * b) * f64(table_size)
}

// transfer moves gas from `a` to `b` through `area` for `dt`, writing the
// transfer into both volumes' accumulators, and returns the mass moved (kg,
// positive a -> b).
//
// The flow is integrated with a linearised backward-Euler step: the orifice
// equation is differentiated with respect to both pressures and the
// pressure change the transfer itself causes is taken into account. Small
// volumes behind large valves (a cylinder near TDC during overlap) make the
// explicit flow stiff; this keeps it stable without slowing real blowdown.
@[inline]
pub fn (t &FlowTable) transfer(area f64, dt f64, mut a Volume, mut b Volume) f64 {
	pa := a.pressure()
	pb := b.pressure()
	return t.transfer_with(area, dt, mut a, pa, mut b, pb)
}

// transfer_with is `transfer` with pre-computed pressures.
pub fn (t &FlowTable) transfer_with(area f64, dt f64, mut a Volume, pa f64, mut b Volume, pb f64) f64 {
	if area <= 0 || pa == pb {
		return 0.0
	}
	fa := a.burned_fraction()
	fb := b.burned_fraction()
	ka := a.stiffness()
	kb := b.stiffness()
	forward := pa > pb
	p_up := if forward { pa } else { pb }
	p_dn := if forward { pb } else { pa }
	t_up := if forward { a.temperature() } else { b.temperature() }
	k_up := if forward { ka } else { kb }
	k_dn := if forward { kb } else { ka }
	pr := p_dn / p_up
	psi, dpsi := t.psi_slope(pr, if forward { fa } else { fb })
	c := area / math.sqrt(r_gas * t_up)
	mdot := c * p_up * psi
	d_up := c * (psi - pr * dpsi) // d(mdot)/d(p_up) >= 0
	d_dn := c * dpsi // d(mdot)/d(p_dn) <= 0
	mut x := mdot * dt / (1.0 + dt * (k_up * d_up - k_dn * d_dn))
	// never move more than half of what would equalise the pressures
	limit := 0.5 * (p_up - p_dn) / (ka + kb)
	if x > limit {
		x = limit
	}
	if forward {
		h := a.specific_enthalpy()
		a.acc_m -= x
		a.acc_e -= x * h
		a.acc_mb -= x * fa
		b.acc_m += x
		b.acc_e += x * h
		b.acc_mb += x * fa
		return x
	}
	h := b.specific_enthalpy()
	b.acc_m -= x
	b.acc_e -= x * h
	b.acc_mb -= x * fb
	a.acc_m += x
	a.acc_e += x * h
	a.acc_mb += x * fb
	return -x
}
