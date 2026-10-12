module acoustics

import math
import gas

// Choked mass flux coefficient sqrt(g)*(2/(g+1))^((g+1)/(2(g-1))) for g ~ 1.35.
const choke_coef = 0.676

pub enum NodeKind {
	junction    // lossless scattering junction (N pipes)
	volume      // lumped gas volume (plenum, airbox, muffler box)
	port        // cylinder port volume (fed by a valve)
	open        // open, radiating end
	closed      // closed end
	restriction // two pipes joined through a flow restriction
}

pub struct End {
pub:
	pipe int
	side int // 0 = pipe end A (`from`), 1 = end B (`to`)
}

pub struct Node {
pub mut:
	name       string
	kind       NodeKind
	ends       []End
	sum_y      f64
	vol        int = -1 // volumes[] index (volume) or ports[] index (port)
	outlet     int = -1
	resonators []int
	k_restrict f64 // restriction: dp = k*q*|q|
	p          f64 // gauge pressure after the last step (Pa)
	flow       f64 // restriction / outlet mass flow (kg/s)
}

// Helmholtz is a neck + cavity side branch hung on a junction.
pub struct Helmholtz {
pub mut:
	node int
	g    f64 // dt * A_neck / L_eff
	r    f64 // neck resistance (Pa per kg/s)
	k    f64 // dt * c^2 / V
	q    f64 // neck mass flow into the cavity (kg/s)
	pc   f64 // cavity gauge pressure (Pa)
	freq f64 // resonance (Hz), informational
}

// Outlet is an open end that radiates sound.
pub struct Outlet {
pub mut:
	node      int
	pan       f64
	gain      f64 = 1.0
	lp        OnePole // reflection low-pass (Levine-Schwinger approximation)
	flow      f64     // mass flow leaving the pipe (kg/s)
	prev_flow f64
	inv_rho   f64 // 1 / gas density at the outlet (m^3/kg)
	radiated  f64 // dU/dt (m^3/s^2): monopole source strength
	r_flow    f64 = 1.0 // reflection magnitude reduction from mean flow
	pipe      int
}

// PipeSpec describes one waveguide in a network.
pub struct PipeSpec {
pub:
	from     string
	to       string
	length   f64
	diameter f64
	temp     f64 // K
	cutoff   f64 // loss filter cutoff (Hz)
	alpha    f64 = 0.04 // broadband attenuation of oscillations (Np/m)
}

pub struct VolumeSpec {
pub:
	node   string
	volume f64
	temp   f64
}

pub struct HelmholtzSpec {
pub:
	node          string
	volume        f64
	neck_length   f64
	neck_diameter f64
	damping       f64
	temp          f64
}

pub struct OutletSpec {
pub:
	node string
	pan  f64
	gain f64 = 1.0
}

pub struct RestrictionSpec {
pub:
	node string
	k    f64
}

// NetworkSpec is the resolved description of a pipe network. Port nodes are
// named `<port_prefix>1` .. `<port_prefix>N`.
pub struct NetworkSpec {
pub:
	pipes        []PipeSpec
	volumes      []VolumeSpec
	helmholtz    []HelmholtzSpec
	outlets      []OutletSpec
	restrictions []RestrictionSpec
	port_prefix  string = 'cyl'
	port_count   int
	port_volume  f64
	port_temp    f64
	burned       f64 // composition of the gas the pipes carry (0..1)
	p_amb        f64 = 101325.0
}

// Network is a graph of waveguides joined at nodes. Each step all pipes read
// their arriving waves, every node computes its outgoing waves, then all
// pipes write.
pub struct Network {
pub mut:
	pipes     []Pipe
	nodes     []Node
	volumes   []gas.Volume
	ports     []gas.Volume // index = cylinder index
	helm      []Helmholtz
	outlets   []Outlet
	dt        f64
	fs        f64
	p_amb     f64
	burned    f64
	gamma     f64
	node_ids  map[string]int
	port_node []int
	count     u64
}

// new_network builds a network from a spec at simulation rate `fs`.
pub fn new_network(spec NetworkSpec, fs f64) !Network {
	mut n := Network{
		dt:     1.0 / fs
		fs:     fs
		p_amb:  spec.p_amb
		burned: spec.burned
		gamma:  gas.gamma_mix(spec.burned)
	}
	for ps in spec.pipes {
		a := n.node_id(ps.from)
		b := n.node_id(ps.to)
		pi := n.pipes.len
		n.pipes << new_pipe(ps.length, ps.diameter, ps.temp, n.gamma, ps.cutoff, fs)
		n.pipes[pi].set_static_loss(ps.alpha)
		n.pipes[pi].burned = spec.burned
		n.pipes[pi].floor = -(spec.p_amb - 2000.0) * 0.5
		n.nodes[a].ends << End{pi, 0}
		n.nodes[b].ends << End{pi, 1}
	}
	// Ports
	n.port_node = []int{len: spec.port_count, init: -1}
	for i in 0 .. spec.port_count {
		name := '${spec.port_prefix}${i + 1}'
		id := n.node_ids[name] or { return error('port node `${name}` is not connected') }
		n.nodes[id].kind = .port
		n.nodes[id].vol = i
		n.port_node[i] = id
		n.ports << gas.new_volume(spec.port_volume, spec.p_amb, spec.port_temp, spec.burned)
	}
	for vs in spec.volumes {
		id := n.node_ids[vs.node] or { return error('volume node `${vs.node}` is not connected') }
		n.nodes[id].kind = .volume
		n.nodes[id].vol = n.volumes.len
		n.volumes << gas.new_volume(vs.volume, spec.p_amb, vs.temp, spec.burned)
	}
	for os in spec.outlets {
		id := n.node_ids[os.node] or { return error('outlet node `${os.node}` is not connected') }
		if n.nodes[id].ends.len != 1 {
			return error('outlet `${os.node}` must terminate exactly one pipe')
		}
		n.nodes[id].kind = .open
		n.nodes[id].outlet = n.outlets.len
		pipe := n.pipes[n.nodes[id].ends[0].pipe]
		radius := math.sqrt(pipe.area / math.pi)
		// |R| = 0.5 near ka = 1 (Levine-Schwinger): one-pole with fc = c/(2*pi*a*1.73)
		fc := pipe.c / (2.0 * math.pi * radius * 1.73)
		n.outlets << Outlet{
			node:    id
			pan:     os.pan
			gain:    os.gain
			lp:      new_lowpass1(fc, fs)
			inv_rho: gas.r_gas * pipe.temp / spec.p_amb
			pipe:    n.nodes[id].ends[0].pipe
		}
	}
	for rs in spec.restrictions {
		id := n.node_ids[rs.node] or { return error('restriction node `${rs.node}` is not connected') }
		if n.nodes[id].ends.len != 2 {
			return error('restriction `${rs.node}` must join exactly two pipes')
		}
		n.nodes[id].kind = .restriction
		n.nodes[id].k_restrict = rs.k
	}
	for hs in spec.helmholtz {
		id := n.node_ids[hs.node] or { return error('resonator node `${hs.node}` is not connected') }
		if n.nodes[id].kind != .junction {
			return error('resonator at `${hs.node}` must hang off a plain junction')
		}
		c := math.sqrt(n.gamma * gas.r_gas * hs.temp)
		an := math.pi * hs.neck_diameter * hs.neck_diameter * 0.25
		leff := hs.neck_length + 0.85 * hs.neck_diameter // end corrections
		omega := c * math.sqrt(an / (leff * hs.volume))
		n.nodes[id].resonators << n.helm.len
		n.helm << Helmholtz{
			node: id
			g:    n.dt * an / leff
			r:    2.0 * hs.damping * omega * leff / an
			k:    n.dt * c * c / hs.volume
			freq: omega / (2.0 * math.pi)
		}
	}
	// Remaining single-ended plain nodes are closed ends.
	for mut node in n.nodes {
		if node.kind == .junction && node.ends.len == 1 && node.resonators.len == 0 {
			node.kind = .closed
		}
		if node.ends.len == 0 {
			return error('node `${node.name}` has no pipes')
		}
		mut sy := 0.0
		for e in node.ends {
			sy += n.pipes[e.pipe].y
		}
		node.sum_y = sy
	}
	return n
}

fn (mut n Network) node_id(name string) int {
	if id := n.node_ids[name] {
		return id
	}
	id := n.nodes.len
	n.nodes << Node{
		name: name
	}
	n.node_ids[name] = id
	return id
}

// step advances the whole network by one simulation step. Port and volume
// accumulators (filled by valve/throttle flows) are consumed here.
@[direct_array_access]
pub fn (mut n Network) step() {
	n.count++
	if n.count % 64 == 0 {
		for mut p in n.pipes {
			p.update_losses(n.p_amb)
		}
		for mut o in n.outlets {
			m := n.pipes[o.pipe].mean_mach(n.p_amb)
			mm := if m > 0.6 { 0.6 } else { m }
			o.r_flow = (1.0 - mm) / (1.0 + mm)
		}
	}
	for mut p in n.pipes {
		p.read()
	}
	for i in 0 .. n.nodes.len {
		match n.nodes[i].kind {
			.junction { n.step_junction(i) }
			.volume { n.step_volume(i, false) }
			.port { n.step_volume(i, true) }
			.open { n.step_open(i) }
			.closed { n.step_closed(i) }
			.restriction { n.step_restriction(i) }
		}
	}
	for mut p in n.pipes {
		p.write()
	}
}

@[direct_array_access]
fn (mut n Network) step_junction(i int) {
	mut node := &n.nodes[i]
	mut s := 0.0
	for e in node.ends {
		s += n.pipes[e.pipe].y * n.pipes[e.pipe].arriving(e.side)
	}
	sy := node.sum_y
	mut q_total := 0.0
	for r in node.resonators {
		q_total += n.helm[r].q
	}
	for r in node.resonators {
		mut h := &n.helm[r]
		q_other := q_total - h.q
		// Implicit neck update coupled to the junction pressure.
		drive := (2.0 * s - q_other) / sy
		q_new := (h.q + h.g * (drive - h.pc)) / (1.0 + h.g / sy + h.g * h.r)
		q_total = q_other + q_new
		h.q = q_new
		h.pc += h.k * q_new
	}
	p := (2.0 * s - q_total) / sy
	node.p = p
	for e in node.ends {
		n.pipes[e.pipe].set_outgoing(e.side, p - n.pipes[e.pipe].arriving(e.side))
	}
}

@[direct_array_access]
fn (mut n Network) step_volume(i int, is_port bool) {
	mut node := &n.nodes[i]
	mut vol := if is_port { &n.ports[node.vol] } else { &n.volumes[node.vol] }
	mut s := 0.0
	for e in node.ends {
		s += n.pipes[e.pipe].y * n.pipes[e.pipe].arriving(e.side)
	}
	p_old := vol.pressure() - n.p_amb
	k := vol.stiffness()
	dt := n.dt
	// Linearised backward Euler for the volume-pipe coupling.
	p_new := (p_old + k * vol.acc_m + 2.0 * k * dt * s) / (1.0 + k * dt * node.sum_y)
	node.p = p_new
	vol.commit()
	h_vol := vol.specific_enthalpy()
	b_vol := vol.burned_fraction()
	p_abs := vol.pressure()
	t_vol := vol.temperature()
	for e in node.ends {
		mut pipe := &n.pipes[e.pipe]
		a := pipe.arriving(e.side)
		mut mdot := (p_new - 2.0 * a) * pipe.y // into the pipe
		// Linear acoustics has no upper bound on the flow a large wave can
		// drive; real flow chokes. Bound it by sonic flow from the upstream
		// side so a strong rarefaction cannot empty a small port in one go.
		if mdot > 0 {
			cap := choke_coef * pipe.area * math.max(p_abs, 0.0) / math.sqrt(gas.r_gas * t_vol)
			if mdot > cap {
				mdot = cap
			}
		} else {
			p_up := math.max(n.p_amb + 2.0 * a, 0.0)
			cap := choke_coef * pipe.area * p_up / math.sqrt(gas.r_gas * pipe.temp)
			if -mdot > cap {
				mdot = -cap
			}
		}
		pipe.set_outgoing(e.side, a + mdot / pipe.y)
		dm := mdot * dt
		if dm >= 0 {
			vol.m -= dm
			vol.e -= dm * h_vol
			vol.mb -= dm * b_vol
		} else {
			x := -dm
			h_in := gas.cp(n.gamma) * pipe.temp
			vol.m += x
			vol.e += x * h_in
			vol.mb += x * pipe.burned
		}
	}
	vol.sanitize()
}

@[direct_array_access]
fn (mut n Network) step_open(i int) {
	mut node := &n.nodes[i]
	e := node.ends[0]
	mut pipe := &n.pipes[e.pipe]
	a := pipe.arriving(e.side)
	mut o := &n.outlets[node.outlet]
	// Mean flow leaving a pipe sheds vortices that absorb low-frequency
	// sound: |R| ~ (1 - M) / (1 + M).
	out := -o.lp.process(a) * o.r_flow
	pipe.set_outgoing(e.side, out)
	node.p = a + out
	flow := (a - out) * pipe.y
	o.prev_flow = o.flow
	o.flow = flow
	o.radiated = (flow - o.prev_flow) * n.fs * o.inv_rho
	node.flow = flow
}

@[direct_array_access]
fn (mut n Network) step_closed(i int) {
	mut node := &n.nodes[i]
	e := node.ends[0]
	a := n.pipes[e.pipe].arriving(e.side)
	n.pipes[e.pipe].set_outgoing(e.side, a * 0.995)
	node.p = 2.0 * a
}

@[direct_array_access]
fn (mut n Network) step_restriction(i int) {
	mut node := &n.nodes[i]
	e1 := node.ends[0]
	e2 := node.ends[1]
	y1 := n.pipes[e1.pipe].y
	y2 := n.pipes[e2.pipe].y
	a1 := n.pipes[e1.pipe].arriving(e1.side)
	a2 := n.pipes[e2.pipe].arriving(e2.side)
	d := 2.0 * (a1 - a2)
	b := 1.0 / y1 + 1.0 / y2
	k := node.k_restrict
	ad := math.abs(d)
	x := if k > 0 { (-b + math.sqrt(b * b + 4.0 * k * ad)) / (2.0 * k) } else { ad / b }
	q := if d >= 0 { x } else { -x }
	n.pipes[e1.pipe].set_outgoing(e1.side, a1 - q / y1)
	n.pipes[e2.pipe].set_outgoing(e2.side, a2 + q / y2)
	node.flow = q
	node.p = 2.0 * a1 - q / y1
}

// port_pressure returns the absolute pressure in port `i`.
pub fn (n &Network) port_pressure(i int) f64 {
	return n.ports[i].pressure()
}

// total_outlet_flow returns the summed mass flow out of all outlets (kg/s).
pub fn (n &Network) total_outlet_flow() f64 {
	mut s := 0.0
	for o in n.outlets {
		s += o.flow
	}
	return s
}
