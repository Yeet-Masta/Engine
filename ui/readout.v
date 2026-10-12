// Module ui renders the driver readout and handles keyboard input. The text
// formatting here is shared by the window, the headless terminal mode and the
// offline renderer; it only reads `sim.Telemetry`, never the simulation.
module ui

import math
import config
import sim

pub fn gear_name(g int) string {
	return match g {
		-1 { 'R' }
		0 { 'N' }
		else { g.str() }
	}
}

fn shift_name(s int) string {
	return match s {
		1 { 'shifting' }
		2 { 'syncing' }
		3 { 'neutral' }
		else { '' }
	}
}

// readout_line is a one-line summary for terminals and logs.
pub fn readout_line(t sim.Telemetry, cfg config.Config) string {
	mut flags := []string{}
	if !t.ignition {
		flags << 'IGN OFF'
	}
	if t.starter {
		flags << 'STARTER'
	}
	if t.limiter_cut {
		flags << 'LIMITER'
	}
	if t.launch {
		flags << 'LAUNCH'
	}
	if t.fuel_cut && t.ignition && !t.limiter_cut {
		flags << 'fuel cut'
	}
	_ = cfg
	return '${t.time:6.1f}s  ${t.rpm:5.0f} rpm  gear ${gear_name(t.gear)}  ${t.speed_kmh:5.1f} km/h  thr ${t.pedal * 100:3.0f}%  ${t.torque:6.1f} Nm  ${t.power_kw:6.1f} kW  MAP ${t.map_kpa:5.1f} kPa  ${flags.join(' ')}'
}

// readout_lines is the detailed multi-line readout for the window.
pub fn readout_lines(t sim.Telemetry, cfg config.Config) []string {
	mic := if t.mic >= 0 && t.mic < cfg.audio.mics.len { cfg.audio.mics[t.mic].name } else { '?' }
	mut lines := []string{}
	lines << cfg.name
	lines << ''
	lines << 'RPM        ${t.rpm:6.0f}     redline ${cfg.engine.redline_rpm:.0f}'
	lines << 'Gear       ${gear_name(t.gear):6}     ${shift_name(t.shift_state)}'
	lines << 'Speed      ${t.speed_kmh:6.1f} km/h'
	lines << 'Throttle   ${t.pedal * 100:6.0f} %   plate ${t.plate * 100:3.0f} %  idle valve ${t.idle_valve * 100:3.0f} %'
	lines << 'Clutch     ${t.clutch * 100:6.0f} %   slip ${t.clutch_slip:6.0f} rpm'
	lines << 'Torque     ${t.torque:6.1f} N*m (indicated ${t.indicated:.1f})'
	lines << 'Power      ${t.power_kw:6.1f} kW  (${t.power_kw * 1.341:.0f} hp)'
	lines << 'Manifold   ${t.map_kpa:6.1f} kPa  air ${t.air_gs:5.1f} g/s  VE ${t.ve * 100:3.0f} %'
	lines << 'Spark      ${t.spark:6.1f} deg   AFR ${t.afr:4.1f}'
	mut state := []string{}
	state << if t.ignition { 'ignition ON' } else { 'ignition OFF' }
	if t.starter {
		state << 'STARTER'
	}
	if t.limiter_cut {
		state << 'LIMITER CUT'
	}
	if t.fuel_cut && t.ignition && !t.limiter_cut {
		state << 'overrun fuel cut'
	}
	lines << 'State      ${state.join(', ')}'
	lines << 'Assists    limiter ${onoff(t.limiter_on)}  launch ${onoff(t.launch)}  auto-clutch ${onoff(t.auto_clutch)}  rev-match ${onoff(t.rev_match)}'
	lines << 'Listener   ${mic}   level ${t.level_db:5.1f} dBFS'
	load := t.sim_load * 100.0
	lines << 'Audio      sim load ${load:4.1f} %  buffered ${t.buffered} frames  underruns ${t.underruns}'
	if math.abs(t.wheel_slip) > 0.8 {
		lines << 'WHEELSPIN  ${t.wheel_slip:4.1f} m/s'
	}
	return lines
}

fn onoff(b bool) string {
	return if b { 'on' } else { 'off' }
}

pub const help_lines = [
	'W / Up       throttle (hold)        1..9, 0   throttle depth 10..100%',
	'Space        clutch (hold)          B / Down  brake (hold)',
	'E / Right    gear up                Q / Left  gear down     N  neutral',
	'I            ignition on/off        S         starter (hold)',
	'L            rev limiter on/off     K         launch control',
	'A            auto-clutch            R         rev-matching',
	'M            next listener          + / -     volume',
	'F5           reload config          H         hide help    Esc  quit',
]
