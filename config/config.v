// Module config defines the vehicle description format (JSON) and loads,
// defaults and validates it. Everything about the engine layout, intake,
// exhaust, drivetrain, vehicle and audio comes from here; nothing is
// hard-coded elsewhere.
//
// Units are SI throughout (metres, kilograms, seconds, pascals, kelvin)
// except angles (degrees) and engine speeds (rpm), which are what people
// read off spec sheets.
module config

import os
import x.json2

pub struct Config {
pub mut:
	name        string = 'Unnamed vehicle'
	description string
	simulation  SimulationConfig
	engine      EngineConfig
	intake      IntakeConfig
	exhaust     ExhaustConfig
	drivetrain  DrivetrainConfig
	vehicle     VehicleConfig
	audio       AudioConfig
}

pub struct SimulationConfig {
pub mut:
	// Physics steps per audio sample (1-4). 2 at 48 kHz = 96 kHz physics.
	substeps            int = 2
	ambient_pressure    f64 = 101325.0
	ambient_temperature f64 = 293.15
	// Start with the engine already idling instead of stopped.
	start_running bool = true
	seed          u64  = 12345
}

pub struct BankConfig {
pub mut:
	// Angle of the cylinder axis from vertical, in degrees, measured in the
	// direction of crank rotation (viewed from the front).
	angle f64
	name  string
}

pub struct CylinderConfig {
pub mut:
	bank  int // index into engine.banks
	throw int // index into engine.throws
}

pub struct ValveConfig {
pub mut:
	// Intake valve: `open` is degrees before TDC, `close` degrees after BDC.
	// Exhaust valve: `open` is degrees before BDC, `close` degrees after TDC.
	// These are "advertised" (seat-to-seat) timings.
	open  f64 = 20
	close f64 = 50
	// Peak lift (m), head diameter (m), valves per cylinder.
	lift     f64 = 0.0095
	diameter f64 = 0.033
	count    int = 2
	// Discharge coefficient of the curtain/port.
	cd f64 = 0.62
	// Port throat diameter (m); 0 means 0.88 x valve diameter.
	port_diameter f64
	// Lift profile exponent: 2 = sin^2 (gentle), lower = more aggressive.
	profile f64 = 1.6
}

pub struct CombustionConfig {
pub mut:
	afr           f64 = 14.7 // part-load air-fuel ratio (by mass)
	afr_wot       f64 = 12.8 // power enrichment at wide-open throttle
	afr_stoich    f64 = 14.7
	fuel_lhv      f64 = 44.0e6 // J/kg
	efficiency    f64 = 0.86   // combustion efficiency (lumps dissociation)
	burn_duration f64 = 55.0   // Wiebe 0-100% duration, crank degrees
	wiebe_a       f64 = 5.0
	wiebe_m       f64 = 2.0
	// Cycle-to-cycle variation (fractional std-dev of burn duration and heat).
	variation f64 = 0.05
	// Spark timing. `mbt` (default) times the spark so 50% of the charge has
	// burned `ca50_full` degrees after TDC at full load (`ca50_light` at light
	// load); `map` uses the simple rpm map below instead.
	spark_mode string = 'mbt'
	ca50_full  f64    = 8.0
	ca50_light f64    = 14.0
	// Map mode: advance (deg BTDC) at idle, maximum, and the rpm where the
	// maximum is reached.
	spark_idle     f64 = 14
	spark_max      f64 = 34
	spark_full_rpm f64 = 4500
	// Knock-limit retard at full load and low rpm (both modes, deg).
	spark_load_retard f64 = 6
	// Deceleration fuel cut-off on a closed throttle above idle.
	overrun_fuel_cut bool = true
	// 0..1: tendency for unburned charge to ignite in the exhaust (pops and
	// crackles on overrun and against the limiter).
	afterfire f64 = 0.0
}

pub struct FrictionConfig {
pub mut:
	// Mechanical friction mean effective pressure
	// FMEP = a + b*Sp + c*Sp^2 (kPa), Sp = mean piston speed (m/s).
	// Pumping losses are not included; they come out of the gas exchange.
	a f64 = 75.0
	b f64 = 4.0
	c f64 = 0.25
	// Constant accessory load: alternator, pumps (N*m).
	accessory f64 = 4.0
}

pub struct StarterConfig {
pub mut:
	torque   f64 = 70.0  // stall torque at the crank (N*m)
	free_rpm f64 = 320.0 // crank speed where starter torque falls to zero
}

pub struct IdleConfig {
pub mut:
	// Maximum idle-air bypass diameter (m) around the closed throttle.
	bypass_diameter f64 = 0.014
	// Open-loop bypass opening fraction at the idle point (the controller
	// learns the rest).
	base f64 = 0.1
	// Extra opening while on the pedal, bled off after lifting.
	dashpot f64 = 0.08
	// Extra opening during deceleration fuel cut.
	decel_air f64 = 0.5
	// PID gains on the normalised rpm error (kd acts on rpm rate).
	kp f64 = 0.3
	ki f64 = 0.8
	kd f64 = 0.15
	// Idle spark: retard from best torque (deg) kept as a fast torque
	// reserve, and how much of it is used per 100 rpm of error.
	spark_reserve f64 = 8.0
	spark_gain    f64 = 4.0
}

pub struct LimiterConfig {
pub mut:
	enabled    bool   = true
	rpm        f64    = 6800
	hysteresis f64    = 120
	mode       string = 'spark_cut' // spark_cut | fuel_cut
	launch_rpm f64    = 4000
}

pub struct EngineConfig {
pub mut:
	name              string
	bore              f64 = 0.086
	stroke            f64 = 0.086
	rod_length        f64 = 0.145
	compression_ratio f64 = 10.0
	banks             []BankConfig
	// Crank throw (crankpin) angles in degrees, measured in the direction of
	// rotation. Several cylinders may share a throw (V engines).
	throws    []f64
	cylinders []CylinderConfig
	// Firing order, 1-based cylinder numbers.
	firing_order []int
	// Optional explicit firing angles (degrees, one per entry of firing_order)
	// that override the angles derived from geometry.
	firing_angles []f64
	// Piston + small-end rod mass per cylinder (kg).
	reciprocating_mass f64 = 0.55
	// Crankshaft + flywheel + damper rotational inertia (kg*m^2).
	inertia             f64 = 0.15
	idle_rpm            f64 = 800
	redline_rpm         f64 = 6500
	intake_valve        ValveConfig
	exhaust_valve       ValveConfig = ValveConfig{
		open:     50
		close:    20
		diameter: 0.028
	}
	intake_port_volume  f64 = 70e-6
	exhaust_port_volume f64 = 45e-6
	wall_temperature    f64 = 430
	heat_transfer       f64 = 1.0 // Woschni multiplier
	combustion          CombustionConfig
	friction            FrictionConfig
	starter             StarterConfig
	idle                IdleConfig
	limiter             LimiterConfig
}

pub struct IntakeConfig {
pub mut:
	throttle_diameter     f64 = 0.060
	throttle_cd           f64 = 0.82
	throttle_closed_angle f64 = 7.0 // plate angle when fully closed (deg)
	// One plenum per bank (each with its own throttle) instead of one shared.
	plenum_per_bank bool
	plenum_volume   f64 = 0.004 // per plenum (m^3)
	airbox_volume   f64 = 0.010
	runner_length   f64 = 0.30
	// Optional per-cylinder runner lengths (overrides runner_length).
	runner_lengths   []f64
	runner_diameter  f64 = 0.040
	snorkel_length   f64 = 0.35
	snorkel_diameter f64 = 0.075
	temperature      f64 = 310
}

pub struct PipeConfig {
pub mut:
	from     string
	to       string
	length   f64
	diameter f64
	// pipe | absorptive | catalyst
	kind string = 'pipe'
	// Gas temperature in the pipe (K); 0 = interpolate header -> tailpipe.
	temperature f64
	// Extra high-frequency damping, 0..1 (absorptive packing, roughness).
	damping f64
}

pub struct ResonatorConfig {
pub mut:
	kind string = 'helmholtz' // helmholtz | quarter_wave
	at   string // node name it hangs off
	// Helmholtz
	volume        f64 = 0.003
	neck_length   f64 = 0.06
	neck_diameter f64 = 0.03
	damping       f64 = 0.1 // fraction of critical damping
	// Quarter-wave side branch
	length   f64 = 0.5
	diameter f64 = 0.04
}

pub struct VolumeConfig {
pub mut:
	node   string
	volume f64
}

pub struct RestrictionConfig {
pub mut:
	node string
	// Pressure drop (Pa) at the reference mass flow (kg/s).
	dp_ref   f64 = 5000
	flow_ref f64 = 0.1
}

pub struct OutletConfig {
pub mut:
	node string
	pan  f64 // -1 (left) .. 1 (right)
	gain f64 = 1.0
}

pub struct ExhaustConfig {
pub mut:
	header_temperature   f64 = 950
	tailpipe_temperature f64 = 520
	// Graph of pipes. Node names `cyl1`..`cylN` are the exhaust ports. Any
	// node listed in `outlets` is an open, radiating end. A node with one pipe
	// that is neither a port nor an outlet is a closed end.
	pipes        []PipeConfig
	resonators   []ResonatorConfig
	volumes      []VolumeConfig
	restrictions []RestrictionConfig
	outlets      []OutletConfig
}

pub struct DrivetrainConfig {
pub mut:
	gear_ratios       []f64 = [3.6, 2.1, 1.4, 1.0, 0.82]
	reverse_ratio     f64   = 3.4
	final_drive       f64   = 3.9
	efficiency        f64   = 0.93
	clutch_max_torque f64   = 420
	// Clutch disc + gearbox input shaft inertia (kg*m^2).
	input_inertia f64 = 0.015
	shift_time    f64 = 0.22
	// Synchroniser friction torque capacity (N*m at the input shaft).
	synchro_torque f64 = 90
	// Driveline torsional stiffness/damping, referred to the wheels.
	shaft_stiffness f64 = 9000
	shaft_damping   f64 = 220
	// Gear whine: teeth on the input gear and whether gears are straight-cut.
	gear_teeth   int = 23
	straight_cut bool
	// Start with automatic clutch / rev matching enabled.
	auto_clutch bool = true
	rev_match   bool = true
}

pub struct VehicleConfig {
pub mut:
	mass               f64 = 1350
	wheel_radius       f64 = 0.31
	wheel_inertia      f64 = 2.6 // all driven wheels (kg*m^2)
	drag_coefficient   f64 = 0.32
	frontal_area       f64 = 2.2
	rolling_resistance f64 = 0.012
	// Static share of weight on the driven axle and geometry for weight transfer.
	driven_axle_load f64    = 0.5
	cg_height        f64    = 0.52
	wheelbase        f64    = 2.65
	drive            string = 'rwd' // fwd | rwd | awd
	tire_mu_peak     f64    = 1.05
	tire_mu_slide    f64    = 0.85
	brake_decel      f64    = 9.0 // m/s^2 at full brake
}

pub struct MicConfig {
pub mut:
	name string
	// Source gains.
	exhaust    f64 = 1.0
	intake     f64 = 0.15
	mechanical f64 = 0.1
	road       f64
	wind       f64
	// Distance from the exhaust outlets (m): 1/r level and propagation delay.
	distance f64 = 1.5
	// Lowpass cutoffs (Hz) per source, modelling body panels / distance.
	exhaust_lowpass    f64 = 18000
	intake_lowpass     f64 = 18000
	mechanical_lowpass f64 = 18000
	// Cabin boom: resonant peak (Hz) and gain (0 = off).
	boom_freq f64 = 55
	boom_gain f64
	// Stereo width of the exhaust outlets (0 = mono, 1 = as configured).
	width f64 = 1.0
}

pub struct AudioConfig {
pub mut:
	sample_rate   int = 48000
	buffer_frames int = 512 // device buffer
	block_frames  int = 64  // simulation block
	// Target fill of the ring buffer between simulation and audio callback.
	target_latency_ms f64 = 10
	master_gain       f64 = 1.0
	// Turbulent flow noise at the outlets and throttle hiss.
	flow_noise f64 = 0.25
	// Mechanical sub-source levels.
	combustion_noise f64 = 1.0
	valvetrain_noise f64 = 1.0
	gear_whine       f64 = 1.0
	starter_noise    f64 = 1.0
	listener         string = 'exhaust'
	mics             []MicConfig
}

// load reads, defaults and validates a vehicle file.
pub fn load(path string) !Config {
	text := os.read_file(path) or { return error('cannot read config `${path}`: ${err}') }
	return parse(text) or { return error('${path}: ${err}') }
}

// parse decodes a vehicle description from JSON text.
pub fn parse(text string) !Config {
	mut c := json2.decode[Config](text) or { return error('invalid JSON: ${err}') }
	c.apply_defaults()
	c.validate()!
	return c
}

// cylinder_count returns the number of cylinders.
pub fn (c &Config) cylinder_count() int {
	return c.engine.cylinders.len
}

// displacement returns the total swept volume in m^3.
pub fn (e &EngineConfig) displacement() f64 {
	return 0.25 * 3.141592653589793 * e.bore * e.bore * e.stroke * f64(e.cylinders.len)
}

// mic_index returns the index of the mic called `name`, or -1.
pub fn (a &AudioConfig) mic_index(name string) int {
	for i, m in a.mics {
		if m.name == name {
			return i
		}
	}
	return -1
}

// apply_defaults fills in anything that can be derived when a section is
// left out, so small configs stay small.
pub fn (mut c Config) apply_defaults() {
	if c.engine.banks.len == 0 {
		c.engine.banks = [BankConfig{
			angle: 0
		}]
	}
	if c.engine.throws.len == 0 && c.engine.cylinders.len == 0 {
		c.engine.throws = [0.0]
	}
	if c.engine.cylinders.len == 0 {
		// One cylinder per throw on the first bank.
		for i in 0 .. c.engine.throws.len {
			c.engine.cylinders << CylinderConfig{
				bank:  0
				throw: i
			}
		}
	}
	if c.engine.firing_order.len == 0 && c.engine.cylinders.len == 1 {
		c.engine.firing_order = [1]
	}
	if c.exhaust.pipes.len == 0 {
		c.exhaust = default_exhaust(c.engine.cylinders.len, c.exhaust)
	}
	if c.audio.mics.len == 0 {
		c.audio.mics = default_mics()
	}
	if c.intake.runner_lengths.len == 0 {
		for _ in 0 .. c.engine.cylinders.len {
			c.intake.runner_lengths << c.intake.runner_length
		}
	}
}

// default_exhaust builds a simple N-into-1 system with a muffler, used when a
// config gives no exhaust graph.
pub fn default_exhaust(n int, base ExhaustConfig) ExhaustConfig {
	mut e := ExhaustConfig{
		header_temperature:   base.header_temperature
		tailpipe_temperature: base.tailpipe_temperature
	}
	for i in 1 .. n + 1 {
		e.pipes << PipeConfig{
			from:     'cyl${i}'
			to:       'collector'
			length:   0.6 + 0.02 * f64(i % 3)
			diameter: 0.040
		}
	}
	e.pipes << PipeConfig{
		from:     'collector'
		to:       'muffler_in'
		length:   1.6
		diameter: 0.055
	}
	e.pipes << PipeConfig{
		from:     'muffler_in'
		to:       'muffler_out'
		length:   0.45
		diameter: 0.16
		kind:     'absorptive'
		damping:  0.5
	}
	e.pipes << PipeConfig{
		from:     'muffler_out'
		to:       'tail'
		length:   0.6
		diameter: 0.055
	}
	e.outlets = [OutletConfig{
		node: 'tail'
	}]
	return e
}

// default_mics returns the three standard listener positions.
pub fn default_mics() []MicConfig {
	return [
		MicConfig{
			name:       'exhaust'
			exhaust:    1.0
			intake:     0.12
			mechanical: 0.08
			road:       0.05
			wind:       0.05
			distance:   1.5
		},
		MicConfig{
			name:               'engine_bay'
			exhaust:            0.6
			intake:             1.0
			mechanical:         0.8
			road:               0.1
			wind:               0.1
			distance:           3.5
			exhaust_lowpass:    3000
			intake_lowpass:     16000
			mechanical_lowpass: 16000
		},
		MicConfig{
			name:               'cabin'
			exhaust:            0.8
			intake:             0.4
			mechanical:         0.3
			road:               0.5
			wind:               0.4
			distance:           2.5
			exhaust_lowpass:    900
			intake_lowpass:     1200
			mechanical_lowpass: 1500
			boom_freq:          50
			boom_gain:          1.5
			width:              0.3
		},
	]
}

// validate checks the config for consistency and returns a readable error.
pub fn (c &Config) validate() ! {
	e := c.engine
	if c.simulation.substeps < 1 || c.simulation.substeps > 8 {
		return error('simulation.substeps must be between 1 and 8')
	}
	if e.bore <= 0 || e.stroke <= 0 {
		return error('engine.bore and engine.stroke must be > 0')
	}
	if e.rod_length <= e.stroke * 0.5 * 1.2 {
		return error('engine.rod_length must be comfortably longer than the crank radius (stroke/2)')
	}
	if e.compression_ratio <= 1.5 {
		return error('engine.compression_ratio must be > 1.5')
	}
	n := e.cylinders.len
	if n < 1 || n > 16 {
		return error('engine must have 1..16 cylinders (got ${n})')
	}
	for i, cyl in e.cylinders {
		if cyl.bank < 0 || cyl.bank >= e.banks.len {
			return error('engine.cylinders[${i}].bank = ${cyl.bank} is not a valid bank index')
		}
		if cyl.throw < 0 || cyl.throw >= e.throws.len {
			return error('engine.cylinders[${i}].throw = ${cyl.throw} is not a valid throw index')
		}
	}
	if e.firing_order.len != n {
		return error('engine.firing_order must list all ${n} cylinders (got ${e.firing_order.len})')
	}
	mut seen := []bool{len: n + 1}
	for f in e.firing_order {
		if f < 1 || f > n {
			return error('engine.firing_order contains ${f}, cylinders are numbered 1..${n}')
		}
		if seen[f] {
			return error('engine.firing_order lists cylinder ${f} twice')
		}
		seen[f] = true
	}
	if e.firing_angles.len != 0 && e.firing_angles.len != n {
		return error('engine.firing_angles must be empty or have ${n} entries')
	}
	if e.idle_rpm < 200 || e.redline_rpm <= e.idle_rpm {
		return error('engine.idle_rpm must be >= 200 and below redline_rpm')
	}
	if e.limiter.rpm <= e.idle_rpm {
		return error('engine.limiter.rpm must be above idle')
	}
	if e.combustion.spark_mode !in ['mbt', 'map'] {
		return error('engine.combustion.spark_mode must be `mbt` or `map`')
	}
	if e.limiter.mode !in ['spark_cut', 'fuel_cut'] {
		return error('engine.limiter.mode must be `spark_cut` or `fuel_cut`')
	}
	if e.inertia <= 0 {
		return error('engine.inertia must be > 0')
	}
	for name, v in {
		'intake_valve':  e.intake_valve
		'exhaust_valve': e.exhaust_valve
	} {
		if v.lift <= 0 || v.diameter <= 0 || v.count < 1 {
			return error('engine.${name}: lift, diameter and count must be positive')
		}
		if v.open + v.close + 180 >= 360 {
			return error('engine.${name}: duration (open + close + 180) must be under 360 degrees')
		}
	}
	if c.intake.runner_lengths.len != n {
		return error('intake.runner_lengths must have ${n} entries')
	}
	if c.intake.throttle_diameter <= 0 || c.intake.plenum_volume <= 0
		|| c.intake.airbox_volume <= 0 {
		return error('intake: throttle_diameter, plenum_volume and airbox_volume must be > 0')
	}
	c.validate_exhaust()!
	d := c.drivetrain
	if d.gear_ratios.len == 0 {
		return error('drivetrain.gear_ratios must not be empty')
	}
	for r in d.gear_ratios {
		if r <= 0 {
			return error('drivetrain.gear_ratios must all be > 0')
		}
	}
	if d.final_drive <= 0 || d.clutch_max_torque <= 0 || d.input_inertia <= 0 {
		return error('drivetrain: final_drive, clutch_max_torque and input_inertia must be > 0')
	}
	v := c.vehicle
	if v.mass <= 0 || v.wheel_radius <= 0 || v.wheel_inertia <= 0 {
		return error('vehicle: mass, wheel_radius and wheel_inertia must be > 0')
	}
	if v.drive !in ['fwd', 'rwd', 'awd'] {
		return error('vehicle.drive must be fwd, rwd or awd')
	}
	a := c.audio
	if a.sample_rate < 8000 || a.sample_rate > 192000 {
		return error('audio.sample_rate out of range')
	}
	if a.block_frames < 8 || a.block_frames > 4096 {
		return error('audio.block_frames must be 8..4096')
	}
	if a.mic_index(a.listener) < 0 {
		return error('audio.listener `${a.listener}` is not one of the configured mics')
	}
}

fn (c &Config) validate_exhaust() ! {
	x := c.exhaust
	n := c.engine.cylinders.len
	mut degree := map[string]int{}
	for i, p in x.pipes {
		if p.from == '' || p.to == '' || p.from == p.to {
			return error('exhaust.pipes[${i}] needs distinct `from` and `to` nodes')
		}
		if p.length <= 0 || p.diameter <= 0 {
			return error('exhaust.pipes[${i}] (${p.from} -> ${p.to}) needs positive length and diameter')
		}
		if p.kind !in ['pipe', 'absorptive', 'catalyst'] {
			return error('exhaust.pipes[${i}].kind must be pipe, absorptive or catalyst')
		}
		degree[p.from]++
		degree[p.to]++
	}
	for i in 1 .. n + 1 {
		name := 'cyl${i}'
		if degree[name] != 1 {
			return error('exhaust: port node `${name}` must be connected to exactly one pipe (found ${degree[name]})')
		}
	}
	if x.outlets.len == 0 {
		return error('exhaust.outlets must name at least one open end')
	}
	for o in x.outlets {
		if degree[o.node] != 1 {
			return error('exhaust outlet `${o.node}` must be the end of exactly one pipe')
		}
	}
	for r in x.resonators {
		if r.at !in degree {
			return error('exhaust resonator is attached to unknown node `${r.at}`')
		}
		if r.kind !in ['helmholtz', 'quarter_wave'] {
			return error('exhaust resonator kind must be helmholtz or quarter_wave')
		}
	}
	for v in x.volumes {
		if v.node !in degree {
			return error('exhaust volume at unknown node `${v.node}`')
		}
		if v.volume <= 0 {
			return error('exhaust volume at `${v.node}` must be > 0')
		}
	}
	for r in x.restrictions {
		if degree[r.node] != 2 {
			return error('exhaust restriction at `${r.node}` must sit between exactly two pipes')
		}
		if r.dp_ref <= 0 || r.flow_ref <= 0 {
			return error('exhaust restriction at `${r.node}` needs positive dp_ref and flow_ref')
		}
	}
}
