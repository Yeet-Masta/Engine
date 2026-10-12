module acoustics

import math
import config
import gas

// exhaust_spec resolves the exhaust graph of a config into a NetworkSpec:
// temperatures are interpolated along the path from the ports to the
// outlets, quarter-wave branches become closed pipes, open ends get their
// end correction.
pub fn exhaust_spec(cfg config.Config) !NetworkSpec {
	x := cfg.exhaust
	ncyl := cfg.engine.cylinders.len
	mut pipes_cfg := x.pipes.clone()
	for i, r in x.resonators {
		if r.kind == 'quarter_wave' {
			pipes_cfg << config.PipeConfig{
				from:     r.at
				to:       '__qw${i}'
				length:   r.length
				diameter: r.diameter
			}
		}
	}
	// Distance (m) from the nearest port to every node.
	mut dist := map[string]f64{}
	for i in 1 .. ncyl + 1 {
		dist['cyl${i}'] = 0.0
	}
	for _ in 0 .. pipes_cfg.len + 1 {
		mut changed := false
		for p in pipes_cfg {
			if p.from in dist {
				d := dist[p.from] + p.length
				if p.to !in dist || d < dist[p.to] - 1e-9 {
					dist[p.to] = d
					changed = true
				}
			}
			if p.to in dist {
				d := dist[p.to] + p.length
				if p.from !in dist || d < dist[p.from] - 1e-9 {
					dist[p.from] = d
					changed = true
				}
			}
		}
		if !changed {
			break
		}
	}
	mut dmax := 0.0
	for o in x.outlets {
		if o.node !in dist {
			return error('exhaust outlet `${o.node}` is not reachable from any cylinder')
		}
		if dist[o.node] > dmax {
			dmax = dist[o.node]
		}
	}
	if dmax <= 0 {
		dmax = 1.0
	}
	outlet_names := x.outlets.map(it.node)
	temp_at := fn [dist, dmax, x] (name string) f64 {
		d := dist[name] or { dmax }
		t := if d / dmax > 1.0 { 1.0 } else { d / dmax }
		return x.header_temperature + (x.tailpipe_temperature - x.header_temperature) * t
	}
	mut pipes := []PipeSpec{}
	for p in pipes_cfg {
		mut temp := p.temperature
		if temp <= 0 {
			temp = 0.5 * (temp_at(p.from) + temp_at(p.to))
		}
		mut length := p.length
		if p.to in outlet_names || p.from in outlet_names {
			length += 0.61 * p.diameter * 0.5 // unflanged end correction
		}
		mut damping := p.damping
		if p.kind == 'absorptive' && damping <= 0 {
			damping = 0.5
		}
		if p.kind == 'catalyst' && damping <= 0 {
			damping = 0.6
		}
		// broadband attenuation of oscillations (Np/m): walls, plus packing
		// for absorptive sections and the porous brick of a catalyst
		mut alpha := 0.04
		if p.kind == 'absorptive' {
			alpha += 1.5 * damping
		} else if p.kind == 'catalyst' {
			alpha += 1.5
		}
		pipes << PipeSpec{
			from:     p.from
			to:       p.to
			length:   length
			diameter: p.diameter
			temp:     temp
			cutoff:   wall_loss_cutoff(length, p.diameter, damping)
			alpha:    alpha
		}
	}
	mut vols := []VolumeSpec{}
	for v in x.volumes {
		vols << VolumeSpec{
			node:   v.node
			volume: v.volume
			temp:   temp_at(v.node)
		}
	}
	mut helm := []HelmholtzSpec{}
	for r in x.resonators {
		if r.kind == 'helmholtz' {
			helm << HelmholtzSpec{
				node:          r.at
				volume:        r.volume
				neck_length:   r.neck_length
				neck_diameter: r.neck_diameter
				damping:       r.damping
				temp:          temp_at(r.at)
			}
		}
	}
	mut restr := []RestrictionSpec{}
	for r in x.restrictions {
		restr << RestrictionSpec{
			node: r.node
			k:    r.dp_ref / (r.flow_ref * r.flow_ref)
		}
	}
	mut outs := []OutletSpec{}
	for o in x.outlets {
		outs << OutletSpec{
			node: o.node
			pan:  o.pan
			gain: o.gain
		}
	}
	return NetworkSpec{
		pipes:        pipes
		volumes:      vols
		helmholtz:    helm
		outlets:      outs
		restrictions: restr
		port_prefix:  'cyl'
		port_count:   ncyl
		port_volume:  cfg.engine.exhaust_port_volume
		port_temp:    cfg.simulation.ambient_temperature
		burned:       1.0
		p_amb:        cfg.simulation.ambient_pressure
	}
}

// new_exhaust builds the exhaust network for a config.
pub fn new_exhaust(cfg config.Config, fs f64) !Network {
	spec := exhaust_spec(cfg)!
	return new_network(spec, fs)
}

// IntakeSystem is the induction tract: snorkel (radiating mouth) -> airbox ->
// throttle(s) -> plenum(s) -> runners -> intake ports.
pub struct IntakeSystem {
pub mut:
	net           Network
	airbox        int   // volume index
	plenums       []int // volume indices
	bore_area     f64   // throttle bore area (m^2) per plenum
	cd            f64
	cos_closed    f64
	bypass_area   f64 // maximum idle-air bypass area (m^2), shared
	table         gas.FlowTable
	throttle_flow f64 // kg/s through all throttles and the bypass
	hiss          f64 // throttle hiss strength (kg/s weighted by pressure drop)
	plate         f64 // last plate opening 0..1
}

// new_intake builds the induction system for a config.
pub fn new_intake(cfg config.Config, fs f64) !IntakeSystem {
	ic := cfg.intake
	ncyl := cfg.engine.cylinders.len
	nplen := if ic.plenum_per_bank { cfg.engine.banks.len } else { 1 }
	t := ic.temperature
	mut pipes := []PipeSpec{}
	pipes << PipeSpec{
		from:     'mouth'
		to:       'airbox'
		length:   ic.snorkel_length + 0.61 * ic.snorkel_diameter * 0.5
		diameter: ic.snorkel_diameter
		temp:     cfg.simulation.ambient_temperature
		cutoff:   wall_loss_cutoff(ic.snorkel_length, ic.snorkel_diameter, 0.4)
		alpha:    1.0 // air filter element
	}
	for i in 0 .. ncyl {
		pl := if nplen > 1 { cfg.engine.cylinders[i].bank } else { 0 }
		pipes << PipeSpec{
			from:     'plenum${pl}'
			to:       'cyl${i + 1}'
			length:   ic.runner_lengths[i]
			diameter: ic.runner_diameter
			temp:     t
			cutoff:   wall_loss_cutoff(ic.runner_lengths[i], ic.runner_diameter, 0)
		}
	}
	mut vols := [VolumeSpec{
		node:   'airbox'
		volume: ic.airbox_volume
		temp:   cfg.simulation.ambient_temperature
	}]
	for k in 0 .. nplen {
		// a bank without cylinders would leave its plenum unconnected
		mut used := nplen == 1
		for c in cfg.engine.cylinders {
			if c.bank == k {
				used = true
			}
		}
		if !used {
			return error('intake.plenum_per_bank: bank ${k} has no cylinders')
		}
		vols << VolumeSpec{
			node:   'plenum${k}'
			volume: ic.plenum_volume
			temp:   t
		}
	}
	spec := NetworkSpec{
		pipes:       pipes
		volumes:     vols
		outlets:     [OutletSpec{
			node: 'mouth'
		}]
		port_prefix: 'cyl'
		port_count:  ncyl
		port_volume: cfg.engine.intake_port_volume
		port_temp:   t
		burned:      0.0
		p_amb:       cfg.simulation.ambient_pressure
	}
	net := new_network(spec, fs)!
	mut plenums := []int{}
	for k in 0 .. nplen {
		plenums << net.nodes[net.node_ids['plenum${k}']].vol
	}
	bd := cfg.engine.idle.bypass_diameter
	return IntakeSystem{
		net:         net
		airbox:      net.nodes[net.node_ids['airbox']].vol
		plenums:     plenums
		bore_area:   math.pi * ic.throttle_diameter * ic.throttle_diameter * 0.25
		cd:          ic.throttle_cd
		cos_closed:  math.cos(ic.throttle_closed_angle * math.pi / 180.0)
		bypass_area: math.pi * bd * bd * 0.25
		table:       gas.new_flow_table()
	}
}

// throttle_area returns the geometric open area of one butterfly at plate
// opening `u` (0 = closed, 1 = wide open).
pub fn (s &IntakeSystem) throttle_area(u f64) f64 {
	if u <= 0 {
		return 0.0
	}
	phi0 := math.acos(s.cos_closed)
	phi := phi0 + (if u > 1 { 1.0 } else { u }) * (math.pi * 0.5 - phi0)
	return s.bore_area * (1.0 - math.cos(phi) / s.cos_closed)
}

// step moves air through the throttle(s) and advances the network.
// `plate` is throttle opening 0..1, `bypass` the idle valve opening 0..1.
pub fn (mut s IntakeSystem) step(dt f64, plate f64, bypass f64) {
	s.plate = plate
	nplen := s.plenums.len
	area := s.cd * s.throttle_area(plate) + 0.8 * s.bypass_area * bypass / f64(nplen) +
		2e-7 // closed-plate leakage
	mut flow := 0.0
	mut hiss := 0.0
	pa := s.net.volumes[s.airbox].pressure()
	for k in s.plenums {
		pp := s.net.volumes[k].pressure()
		dm := s.table.transfer_with(area, dt, mut s.net.volumes[s.airbox], pa, mut s.net.volumes[k],
			pp)
		flow += dm
		// Throttle hiss: turbulent jet noise grows with flow and pressure drop.
		drop := if pa > pp { (pa - pp) / pa } else { 0.0 }
		hiss += math.abs(dm) * drop
	}
	s.throttle_flow = flow / dt
	s.hiss = hiss / dt
	s.net.step()
}

// plenum_pressure returns the (mean) absolute manifold pressure.
pub fn (s &IntakeSystem) plenum_pressure() f64 {
	mut p := 0.0
	for k in s.plenums {
		p += s.net.volumes[k].pressure()
	}
	return p / f64(s.plenums.len)
}
