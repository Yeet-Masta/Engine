module ui

import gg
import os
import config
import input
import sim

// Window is the interactive front end: a gg (sokol_app) window that turns key
// down/up events into control actions and draws the text readout. It only
// touches the simulation through `input.Controls` and `sim.TelemetryBox`, so
// a graphical engine view can be added here without touching the physics.
pub struct Window {
mut:
	ctx       &gg.Context = unsafe { nil }
	ctl       &input.Controls
	tel       &sim.TelemetryBox
	cfg       config.Config
	reload    fn () string = unsafe { nil }
	show_help bool = true
	message   string
	msg_time  f64
	frames    u64
	last      sim.Telemetry
	lines     []string
	mtime     i64
	cfg_path  string
	held      map[int]bool
}

pub fn new_window(ctl &input.Controls, tel &sim.TelemetryBox, cfg config.Config, reload fn () string) &Window {
	mut w := &Window{
		ctl:    ctl
		tel:    tel
		cfg:    cfg
		reload: reload
	}
	w.ctx = gg.new_context(
		width:         980
		height:        620
		window_title:  'enginesound - ${cfg.name}'
		bg_color:      gg.rgb(18, 20, 24)
		frame_fn:      frame
		event_fn:      on_event
		user_data:     w
		swap_interval: 1
	)
	return w
}

// watch_config enables hot reloading when `path` changes on disk.
pub fn (mut w Window) watch_config(path string) {
	w.cfg_path = path
	w.mtime = os.file_last_mod_unix(path)
}

pub fn (mut w Window) run() {
	w.ctx.run()
}

fn key_action(k gg.KeyCode) ?input.Action {
	return match k {
		.w, .up { input.Action.throttle }
		.b, .down { input.Action.brake }
		.space { input.Action.clutch }
		.e, .right, .page_up { input.Action.gear_up }
		.q, .left, .page_down { input.Action.gear_down }
		.n { input.Action.neutral }
		.i { input.Action.ignition }
		.s { input.Action.starter }
		.l { input.Action.limiter }
		.k { input.Action.launch }
		.a { input.Action.auto_clutch }
		.r { input.Action.rev_match }
		.m { input.Action.mic_next }
		.equal { input.Action.volume_up }
		.minus { input.Action.volume_down }
		.f5 { input.Action.reload }
		.escape { input.Action.quit }
		.h { input.Action.help }
		else { none }
	}
}

fn on_event(e &gg.Event, data voidptr) {
	mut w := unsafe { &Window(data) }
	if e.typ != .key_down && e.typ != .key_up {
		return
	}
	pressed := e.typ == .key_down
	code := int(e.key_code)
	// ignore auto-repeat: held actions are level-triggered already
	if pressed && w.held[code] {
		return
	}
	w.held[code] = pressed
	// number keys set the throttle depth
	if pressed && code >= int(gg.KeyCode._0) && code <= int(gg.KeyCode._9) {
		n := code - int(gg.KeyCode._0)
		depth := if n == 0 { 1.0 } else { f64(n) / 10.0 }
		w.ctl.set_throttle_cap(depth)
		w.flash('throttle depth ${int(depth * 100)}%')
		return
	}
	a := key_action(e.key_code) or { return }
	match a {
		.help {
			if pressed {
				w.show_help = !w.show_help
			}
		}
		.quit {
			if pressed {
				w.ctl.apply(a, true)
				w.ctx.quit()
			}
		}
		.reload {
			if pressed && w.reload != unsafe { nil } {
				w.flash(w.reload())
			}
		}
		else {
			w.ctl.apply(a, pressed)
		}
	}
}

fn (mut w Window) flash(msg string) {
	w.message = msg
	w.msg_time = 3.0
}

fn frame(data voidptr) {
	mut w := unsafe { &Window(data) }
	w.frames++
	// refresh text ~15 times a second (keeps allocations off the hot path)
	if w.frames % 4 == 1 {
		w.last = w.tel.read()
		w.lines = readout_lines(w.last, w.cfg)
	}
	// hot reload when the config file changes
	if w.cfg_path != '' && w.frames % 30 == 0 {
		m := os.file_last_mod_unix(w.cfg_path)
		if m != w.mtime {
			w.mtime = m
			if w.reload != unsafe { nil } {
				w.flash(w.reload())
			}
		}
	}
	if w.msg_time > 0 {
		w.msg_time -= 1.0 / 60.0
	}
	ctx := w.ctx
	ctx.begin()
	txt := gg.TextCfg{
		color: gg.rgb(220, 224, 230)
		size:  18
		mono:  true
	}
	dim := gg.TextCfg{
		color: gg.rgb(140, 146, 156)
		size:  15
		mono:  true
	}
	mut y := 16
	for i, line in w.lines {
		ctx.draw_text(20, y, line, if i == 0 { gg.TextCfg{
			...txt
			size: 22
		} } else { txt })
		y += if i == 0 { 30 } else { 23 }
	}
	// rpm bar
	t := w.last
	bar_x := f32(620)
	bar_w := f32(330)
	frac := f32(t.rpm / (w.cfg.engine.limiter.rpm * 1.05))
	red := f32(w.cfg.engine.redline_rpm / (w.cfg.engine.limiter.rpm * 1.05))
	ctx.draw_rect_filled(bar_x, 64, bar_w, 18, gg.rgb(40, 44, 52))
	ctx.draw_rect_filled(bar_x + bar_w * red, 64, bar_w * (1 - red), 18, gg.rgb(90, 30, 30))
	ctx.draw_rect_filled(bar_x, 64, bar_w * clampf(frac), 18, if t.limiter_cut {
		gg.rgb(240, 60, 50)
	} else {
		gg.rgb(80, 180, 255)
	})
	// pedal bars
	ctx.draw_rect_filled(bar_x, 158, bar_w * clampf(f32(t.pedal)), 10, gg.rgb(90, 220, 120))
	ctx.draw_rect_filled(bar_x, 181, bar_w * clampf(f32(t.clutch)), 10, gg.rgb(230, 200, 80))
	if w.show_help {
		mut hy := 430
		for line in help_lines {
			ctx.draw_text(20, hy, line, dim)
			hy += 21
		}
	}
	if w.msg_time > 0 && w.message != '' {
		ctx.draw_text(620, 16, w.message, gg.TextCfg{
			...dim
			color: gg.rgb(255, 210, 120)
		})
	}
	ctx.end()
}

fn clampf(x f32) f32 {
	return if x < 0 { 0 } else if x > 1 { 1 } else { x }
}
