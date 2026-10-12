module sim

import sync.stdatomic

pub const max_cyl = 16

// Telemetry is a plain-data snapshot of the simulation published once per
// audio block. It contains everything a text readout or a future graphical
// engine view needs, in fixed-size arrays so copying never allocates.
pub struct Telemetry {
pub mut:
	time         f64
	rpm          f64
	crank_deg    f64
	pedal        f64
	plate        f64
	clutch       f64 // engagement 0..1
	gear         int
	shift_state  int // 0 engaged, 1 shifting, 2 syncing, 3 neutral
	speed_kmh    f64
	torque       f64 // brake torque, cycle averaged (N*m)
	indicated    f64
	power_kw     f64
	map_kpa      f64
	afr          f64
	spark        f64
	ve           f64
	air_gs       f64 // intake air flow (g/s)
	idle_valve   f64
	clutch_slip  f64 // rpm
	wheel_slip   f64 // m/s
	limiter_cut  bool
	fuel_cut     bool
	ignition     bool
	starter      bool
	limiter_on   bool
	launch       bool
	auto_clutch  bool
	rev_match    bool
	mic          int
	level_db     f64
	ncyl         int
	cyl_bar      [max_cyl]f64
	cyl_temp     [max_cyl]f64
	cyl_phi      [max_cyl]f64
	cyl_in_lift  [max_cyl]f64 // mm
	cyl_ex_lift  [max_cyl]f64 // mm
	cyl_burning  [max_cyl]bool
	exhaust_flow f64 // kg/s out of the tailpipes
	fires        u64
	misfires     u64
	// filled by the host loop
	sim_load  f64 // simulation time / real time
	underruns u64
	buffered  int
}

// TelemetryBox publishes telemetry from the simulation thread to the UI with
// a sequence lock: the writer bumps `seq` to odd, copies, bumps to even; the
// reader retries if it saw an odd or changed sequence. Neither side blocks.
@[heap]
pub struct TelemetryBox {
mut:
	seq  u64
	data Telemetry
}

pub fn new_telemetry_box() &TelemetryBox {
	return &TelemetryBox{}
}

pub fn (mut b TelemetryBox) publish(t &Telemetry) {
	s := stdatomic.load_u64(&b.seq)
	stdatomic.store_u64(&b.seq, s + 1)
	b.data = *t
	stdatomic.store_u64(&b.seq, s + 2)
}

pub fn (b &TelemetryBox) read() Telemetry {
	for _ in 0 .. 100 {
		s1 := stdatomic.load_u64(&b.seq)
		if s1 & 1 == 1 {
			continue
		}
		t := b.data
		s2 := stdatomic.load_u64(&b.seq)
		if s1 == s2 {
			return t
		}
	}
	return b.data
}
