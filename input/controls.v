// Module input holds the driver controls shared between the UI thread (which
// writes them from key events) and the simulation thread (which reads a
// snapshot once per audio block). All fields are 64-bit atomics, so neither
// side ever takes a lock.
module input

import math
import sync.stdatomic

// Flag bits in `Controls.flags`.
pub const flag_ignition = u64(1)
pub const flag_starter = u64(2)
pub const flag_limiter = u64(4)
pub const flag_launch = u64(8)
pub const flag_auto_clutch = u64(16)
pub const flag_rev_match = u64(32)
pub const flag_quit = u64(64)

// Action is a user command, independent of the key that triggers it.
pub enum Action {
	throttle
	brake
	clutch
	gear_up
	gear_down
	neutral
	ignition
	starter
	limiter
	launch
	auto_clutch
	rev_match
	mic_next
	volume_up
	volume_down
	reload
	quit
	help
}

@[heap]
pub struct Controls {
mut:
	throttle     u64 // f64 bits, pedal target 0..1
	throttle_cap u64 // f64 bits, max pedal when the throttle key is held
	brake        u64 // f64 bits
	clutch       u64 // f64 bits, pedal 0 (up) .. 1 (down)
	volume       u64 // f64 bits
	flags        u64
	gear_up      u64 // event counters
	gear_down    u64
	neutral      u64
	mic          u64
	reload       u64
}

// Snapshot is a plain copy the simulation can use without atomics.
pub struct Snapshot {
pub mut:
	throttle    f64
	brake       f64
	clutch      f64
	volume      f64 = 1.0
	ignition    bool
	starter     bool
	limiter     bool = true
	launch      bool
	auto_clutch bool
	rev_match   bool
	quit        bool
	gear_up     u64
	gear_down   u64
	neutral     u64
	mic         u64
	reload      u64
}

pub fn new_controls(ignition bool, auto_clutch bool, rev_match bool) &Controls {
	mut c := &Controls{}
	c.set_f(.volume, 1.0)
	c.set_f(.throttle_cap, 1.0)
	mut f := flag_limiter
	if ignition {
		f |= flag_ignition
	}
	if auto_clutch {
		f |= flag_auto_clutch
	}
	if rev_match {
		f |= flag_rev_match
	}
	stdatomic.store_u64(&c.flags, f)
	return c
}

pub enum Field {
	throttle
	throttle_cap
	brake
	clutch
	volume
}

fn (c &Controls) field_ptr(f Field) &u64 {
	return match f {
		.throttle { &c.throttle }
		.throttle_cap { &c.throttle_cap }
		.brake { &c.brake }
		.clutch { &c.clutch }
		.volume { &c.volume }
	}
}

pub fn (mut c Controls) set_f(f Field, v f64) {
	stdatomic.store_u64(c.field_ptr(f), math.f64_bits(v))
}

pub fn (c &Controls) get_f(f Field) f64 {
	return math.f64_from_bits(stdatomic.load_u64(c.field_ptr(f)))
}

// set_flag sets or clears a flag bit. There is a single writer (the UI or
// script thread), so load + store is enough.
pub fn (mut c Controls) set_flag(bit u64, on bool) {
	old := stdatomic.load_u64(&c.flags)
	stdatomic.store_u64(&c.flags, if on { old | bit } else { old & ~bit })
}

fn (mut c Controls) toggle_flag(bit u64) {
	old := stdatomic.load_u64(&c.flags)
	c.set_flag(bit, old & bit == 0)
}

pub fn (c &Controls) flag(bit u64) bool {
	return stdatomic.load_u64(&c.flags) & bit != 0
}

fn bump(p &u64) {
	stdatomic.store_u64(p, stdatomic.load_u64(p) + 1)
}

// apply handles an action from the UI. `pressed` is false on key release;
// held actions (throttle, brake, clutch, starter) use both edges, the others
// act on press only.
pub fn (mut c Controls) apply(a Action, pressed bool) {
	match a {
		.throttle {
			c.set_f(.throttle, if pressed { c.get_f(.throttle_cap) } else { 0.0 })
		}
		.brake {
			c.set_f(.brake, if pressed { 1.0 } else { 0.0 })
		}
		.clutch {
			c.set_f(.clutch, if pressed { 1.0 } else { 0.0 })
		}
		.starter {
			c.set_flag(flag_starter, pressed)
		}
		else {
			if !pressed {
				return
			}
			match a {
				.gear_up { bump(&c.gear_up) }
				.gear_down { bump(&c.gear_down) }
				.neutral { bump(&c.neutral) }
				.ignition { c.toggle_flag(flag_ignition) }
				.limiter { c.toggle_flag(flag_limiter) }
				.launch { c.toggle_flag(flag_launch) }
				.auto_clutch { c.toggle_flag(flag_auto_clutch) }
				.rev_match { c.toggle_flag(flag_rev_match) }
				.mic_next { bump(&c.mic) }
				.reload { bump(&c.reload) }
				.volume_up { c.set_f(.volume, math.min(4.0, c.get_f(.volume) * 1.25)) }
				.volume_down { c.set_f(.volume, math.max(0.05, c.get_f(.volume) / 1.25)) }
				.quit { c.set_flag(flag_quit, true) }
				else {}
			}
		}
	}
}

// set_throttle_cap sets how far the throttle key opens the pedal (0..1); if
// the throttle is currently held, the pedal follows.
pub fn (mut c Controls) set_throttle_cap(v f64) {
	cap := if v < 0.05 { 0.05 } else if v > 1.0 { 1.0 } else { v }
	held := c.get_f(.throttle) > 0
	c.set_f(.throttle_cap, cap)
	if held {
		c.set_f(.throttle, cap)
	}
}

pub fn (c &Controls) throttle_cap() f64 {
	return c.get_f(.throttle_cap)
}

// snapshot reads all controls (simulation side).
pub fn (c &Controls) snapshot() Snapshot {
	f := stdatomic.load_u64(&c.flags)
	return Snapshot{
		throttle:    c.get_f(.throttle)
		brake:       c.get_f(.brake)
		clutch:      c.get_f(.clutch)
		volume:      c.get_f(.volume)
		ignition:    f & flag_ignition != 0
		starter:     f & flag_starter != 0
		limiter:     f & flag_limiter != 0
		launch:      f & flag_launch != 0
		auto_clutch: f & flag_auto_clutch != 0
		rev_match:   f & flag_rev_match != 0
		quit:        f & flag_quit != 0
		gear_up:     stdatomic.load_u64(&c.gear_up)
		gear_down:   stdatomic.load_u64(&c.gear_down)
		neutral:     stdatomic.load_u64(&c.neutral)
		mic:         stdatomic.load_u64(&c.mic)
		reload:      stdatomic.load_u64(&c.reload)
	}
}
