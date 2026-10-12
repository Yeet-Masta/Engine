module sim

import os
import x.json2
import input

// ScriptEvent changes controls at time `t` (seconds). Any field left out is
// unchanged. Used for headless renders, tests and benchmarks.
pub struct ScriptEvent {
pub:
	t           f64
	throttle    ?f64
	brake       ?f64
	clutch      ?f64
	gear        ?int
	shift       ?string // "up" | "down"
	ignition    ?bool
	starter     ?bool
	limiter     ?bool
	launch      ?bool
	auto_clutch ?bool
	rev_match   ?bool
	mic         ?string
	note        string
}

pub struct Script {
pub mut:
	name     string
	duration f64 = 10.0
	events   []ScriptEvent
}

pub fn load_script(path string) !Script {
	text := os.read_file(path) or { return error('cannot read script `${path}`: ${err}') }
	mut s := json2.decode[Script](text) or { return error('${path}: invalid script: ${err}') }
	s.events.sort(a.t < b.t)
	return s
}

// ScriptPlayer feeds a script into a Controls object (the same path the
// keyboard uses) and the simulator.
pub struct ScriptPlayer {
pub mut:
	script Script
	next   int
}

// advance applies all events up to time `t`.
pub fn (mut p ScriptPlayer) advance(t f64, mut ctl input.Controls, mut s Simulator) {
	for p.next < p.script.events.len && p.script.events[p.next].t <= t {
		ev := p.script.events[p.next]
		p.next++
		if v := ev.throttle {
			ctl.set_throttle_cap(if v > 0.05 { v } else { 0.05 })
			ctl.set_f(.throttle, v)
		}
		if v := ev.brake {
			ctl.set_f(.brake, v)
		}
		if v := ev.clutch {
			ctl.set_f(.clutch, v)
		}
		if v := ev.ignition {
			ctl.set_flag(input.flag_ignition, v)
		}
		if v := ev.starter {
			ctl.set_flag(input.flag_starter, v)
		}
		if v := ev.limiter {
			ctl.set_flag(input.flag_limiter, v)
		}
		if v := ev.launch {
			ctl.set_flag(input.flag_launch, v)
		}
		if v := ev.auto_clutch {
			ctl.set_flag(input.flag_auto_clutch, v)
		}
		if v := ev.rev_match {
			ctl.set_flag(input.flag_rev_match, v)
		}
		if v := ev.shift {
			ctl.apply(if v == 'up' { input.Action.gear_up } else { input.Action.gear_down },
				true)
		}
		if v := ev.gear {
			s.request_gear(v)
		}
		if v := ev.mic {
			idx := s.cfg.audio.mic_index(v)
			if idx >= 0 {
				s.mic = idx
				s.mix.select_mic(idx, false)
			}
		}
	}
}
