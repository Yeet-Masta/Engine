module main

import os
import flag
import math
import time
import sync.stdatomic
import config
import engine
import sim
import audio
import input
import ui

const version = '0.1.0'

struct Options {
mut:
	config_path string
	render      string
	script      string
	duration    f64
	bench       f64
	tone        bool
	info        bool
	headless    bool
	mic         string
	sample_rate int
	no_audio    bool
	dyno        f64
	trace       string
	trace_rpm   f64
	trace_pedal f64
}

fn main() {
	mut fp := flag.new_flag_parser(os.args)
	fp.application('enginesound')
	fp.version(version)
	fp.description('Real-time, physics-based engine sound simulator.\n\nRuns the vehicle described by CONFIG (default configs/i4_economy.json) in a window with keyboard controls, or renders a scripted run to a WAV file.')
	fp.arguments_description('[CONFIG.json]')
	fp.skip_executable()
	mut o := Options{}
	o.render = fp.string('render', `r`, '', 'render to this WAV file instead of opening a window')
	o.script = fp.string('script', `s`, 'scripts/demo.json', 'control script for --render/--bench')
	o.duration = fp.float('duration', `d`, 0, 'override the script duration (seconds)')
	o.bench = fp.float('bench', `b`, 0, 'benchmark: simulate N seconds as fast as possible and report real-time headroom')
	o.tone = fp.bool('tone', `t`, false, 'play a 440 Hz test tone through the audio path and exit')
	o.info = fp.bool('info', `i`, false, 'print the derived engine data (displacement, firing angles, ...) and exit')
	o.headless = fp.bool('headless', 0, false, 'run in real time without a window (terminal readout, keyboard not available)')
	o.mic = fp.string('mic', `m`, '', 'listener: exhaust, engine_bay, cabin, ... (overrides the config)')
	o.sample_rate = fp.int('rate', 0, 0, 'audio sample rate (overrides the config)')
	o.no_audio = fp.bool('no-audio', 0, false, 'do not open the audio device (window and simulation only)')
	o.trace = fp.string('trace', 0, '', 'write one engine cycle of cylinder 1 as CSV to this file (use with --trace-rpm, --trace-pedal)')
	o.trace_rpm = fp.float('trace-rpm', 0, 3000, 'engine speed for --trace')
	o.trace_pedal = fp.float('trace-pedal', 0, 1, 'pedal position for --trace')
	o.dyno = fp.float('dyno', 0, -1, 'print a torque curve at this pedal position (0..1) with the engine speed held, then exit')
	rest := fp.finalize() or {
		eprintln(err)
		println(fp.usage())
		exit(1)
	}
	o.config_path = if rest.len > 0 { rest[0] } else { 'configs/i4_economy.json' }
	if o.tone {
		play_tone(o)
		return
	}
	mut cfg := config.load(o.config_path) or {
		eprintln('error: ${err}')
		exit(1)
	}
	if o.mic != '' {
		if cfg.audio.mic_index(o.mic) < 0 {
			eprintln('error: unknown mic `${o.mic}`; available: ${cfg.audio.mics.map(it.name)}')
			exit(1)
		}
		cfg.audio.listener = o.mic
	}
	if o.sample_rate > 0 {
		cfg.audio.sample_rate = o.sample_rate
	}
	if o.info {
		print_info(cfg)
		return
	}
	if o.trace != '' {
		mut s := sim.new_simulator(cfg, cfg.audio.sample_rate) or {
			eprintln('error: ${err}')
			exit(1)
		}
		os.write_file(o.trace, s.trace_cycle(o.trace_rpm, o.trace_pedal, 0)) or {
			eprintln('error: ${err}')
			exit(1)
		}
		println('wrote ${o.trace}')
		return
	}
	if o.dyno >= 0 {
		run_dyno(cfg, o.dyno) or {
			eprintln('error: ${err}')
			exit(1)
		}
		return
	}
	if o.render != '' {
		render_offline(cfg, o) or {
			eprintln('error: ${err}')
			exit(1)
		}
		return
	}
	if o.bench > 0 {
		run_bench(cfg, o.bench) or {
			eprintln('error: ${err}')
			exit(1)
		}
		return
	}
	run_realtime(cfg, o) or {
		eprintln('error: ${err}')
		exit(1)
	}
}

// print_info shows what the simulator derived from the config.
fn print_info(cfg config.Config) {
	e := cfg.engine
	println('${cfg.name}')
	println('  cylinders:     ${e.cylinders.len}, banks ${e.banks.map(it.angle)} deg, throws ${e.throws} deg')
	println('  displacement:  ${e.displacement() * 1000.0:.3f} L  (bore ${e.bore * 1000:.1f} mm, stroke ${e.stroke * 1000:.1f} mm, CR ${e.compression_ratio})')
	angles := engine.firing_angles(e) or {
		println('  firing:        ERROR ${err}')
		return
	}
	println('  firing order:  ${e.firing_order}')
	println('  firing angles: ${angles.map(math.round(it * 10) / 10)} (deg, per cylinder)')
	println('  intervals:     ${engine.firing_intervals(e, angles).map(math.round(it * 10) / 10)}')
	for b in 0 .. e.banks.len {
		mut bank_angles := []f64{}
		for i, c in e.cylinders {
			if c.bank == b {
				bank_angles << angles[i]
			}
		}
		bank_angles.sort()
		mut gaps := []f64{}
		for k in 0 .. bank_angles.len {
			nxt := if k + 1 < bank_angles.len { bank_angles[k + 1] } else { bank_angles[0] + 720 }
			gaps << math.round((nxt - bank_angles[k]) * 10) / 10
		}
		println('  bank ${b} pulses: ${gaps} deg apart')
	}
	vt := engine.new_valvetrain(e)
	println('  valve overlap: ${vt.overlap():.1f} deg')
	println('  idle/redline/limit: ${e.idle_rpm:.0f} / ${e.redline_rpm:.0f} / ${e.limiter.rpm:.0f} rpm')
	d := cfg.drivetrain
	println('  gears:         ${d.gear_ratios} x ${d.final_drive}')
	for g, r in d.gear_ratios {
		kmh := e.redline_rpm * math.pi / 30.0 / (r * d.final_drive) * cfg.vehicle.wheel_radius * 3.6
		println('    gear ${g + 1}: ${kmh:.0f} km/h at redline')
	}
	println('  exhaust:       ${cfg.exhaust.pipes.len} pipes, ${cfg.exhaust.resonators.len} resonators, ${cfg.exhaust.outlets.len} outlets')
	println('  mics:          ${cfg.audio.mics.map(it.name)} (listener: ${cfg.audio.listener})')
}

// render_offline runs a control script and writes a WAV file.
fn render_offline(cfg config.Config, o Options) ! {
	mut script := sim.load_script(o.script)!
	if o.duration > 0 {
		script.duration = o.duration
	}
	rate := cfg.audio.sample_rate
	mut s := sim.new_simulator(cfg, rate)!
	mut ctl := input.new_controls(cfg.simulation.start_running, cfg.drivetrain.auto_clutch,
		cfg.drivetrain.rev_match)
	mut player := sim.ScriptPlayer{
		script: script
	}
	block := cfg.audio.block_frames
	mut buf := []f32{len: block * 2}
	mut wav := audio.new_wav_writer(rate, 2)
	total := int(script.duration * f64(rate))
	mut done := 0
	mut next_print := 0.0
	sw := time.new_stopwatch()
	for done < total {
		t := f64(done) / f64(rate)
		player.advance(t, mut ctl, mut s)
		s.apply_controls(ctl.snapshot())
		n := if total - done < block { total - done } else { block }
		s.render(mut buf, n)
		wav.append(buf, n)
		done += n
		if t >= next_print {
			println(ui.readout_line(s.tel, cfg))
			next_print += 0.5
		}
	}
	wav.save(o.render)!
	el := sw.elapsed().seconds()
	println('wrote ${o.render}: ${script.duration:.1f} s of audio in ${el:.2f} s (${script.duration / el:.1f}x real time)')
}

// run_dyno prints a steady-state torque curve (engine speed held constant).
fn run_dyno(cfg config.Config, pedal f64) ! {
	mut s := sim.new_simulator(cfg, cfg.audio.sample_rate)!
	println('${cfg.name}: dyno at ${pedal * 100:.0f}% pedal')
	println('   rpm   torque Nm  indicated     kW     hp   MAP kPa    VE %  peak bar   air g/s')
	mut rpm := 1000.0
	for rpm <= cfg.engine.limiter.rpm - 100 {
		p := s.dyno(rpm, pedal, 0.35, 0.25)
		println('${p.rpm:6.0f}   ${p.torque:9.1f}  ${p.indicated:9.1f}  ${p.power_kw:5.1f}  ${p.power_kw * 1.341:5.0f}   ${p.map_kpa:7.1f}  ${p.ve * 100:6.1f}  ${p.peak_bar:8.1f}  ${p.air_gs:8.1f}')
		rpm += 500
	}
}

// run_bench measures how much faster than real time the simulation runs.
fn run_bench(cfg config.Config, seconds f64) ! {
	rate := cfg.audio.sample_rate
	mut s := sim.new_simulator(cfg, rate)!
	mut ctl := input.new_controls(true, true, true)
	block := cfg.audio.block_frames
	mut buf := []f32{len: block * 2}
	total := int(seconds * f64(rate))
	mut done := 0
	sw := time.new_stopwatch()
	for done < total {
		t := f64(done) / f64(rate)
		// alternate between idle and revving so every code path runs
		ctl.set_f(.throttle, if math.fmod(t, 2.0) < 1.0 { 1.0 } else { 0.0 })
		s.apply_controls(ctl.snapshot())
		s.render(mut buf, block)
		done += block
	}
	el := sw.elapsed().seconds()
	factor := seconds / el
	println('${cfg.name}: ${cfg.engine.cylinders.len} cyl, ${s.exhaust.pipes.len + s.intake.net.pipes.len} pipes, ${s.substeps}x${rate} Hz physics')
	println('  simulated ${seconds:.1f} s in ${el:.3f} s -> ${factor:.1f}x real time (${100.0 / factor:.1f}% of one core)')
}

// ---------------------------------------------------------------------------
// real-time operation

// App is shared between the UI thread and the simulation thread.
@[heap]
struct App {
mut:
	cfg_path  string
	cfg       config.Config
	ctl       &input.Controls
	ring      &audio.RingBuffer
	tel       &sim.TelemetryBox
	running   u64 // 1 while the sim thread should run
	target    int // ring buffer target fill (frames)
	pending   &sim.Simulator = unsafe { nil } // new simulator from a hot reload
	pend_flag u64
	message   string
}

// sim_thread renders blocks into the ring buffer, paced by the audio device:
// it renders while the buffer is below its target fill and sleeps otherwise.
fn sim_thread(mut app App, first &sim.Simulator) {
	mut s := unsafe { first }
	mut old := &sim.Simulator(unsafe { nil })
	block := app.cfg.audio.block_frames
	mut buf := []f32{len: block * 2}
	mut buf_old := []f32{len: block * 2}
	fade_len := 2048
	mut fade_pos := fade_len
	mut busy := 0.0
	mut wall := time.new_stopwatch()
	mut tel_timer := 0
	mut last_underruns := u64(0)
	rate := f64(s.fs_out)
	for stdatomic.load_u64(&app.running) == 1 {
		// hot reload hand-over (simulator built by the UI thread)
		if stdatomic.load_u64(&app.pend_flag) == 1 {
			mut ns := app.pending
			ns.sync_counters(app.ctl.snapshot())
			old = s
			s = ns
			fade_pos = 0
			stdatomic.store_u64(&app.pend_flag, 0)
		}
		if app.ring.fill() >= app.target {
			time.sleep(500 * time.microsecond)
			continue
		}
		t0 := time.sys_mono_now()
		snap := app.ctl.snapshot()
		s.apply_controls(snap)
		s.render(mut buf, block)
		if fade_pos < fade_len {
			old.apply_controls(snap)
			old.render(mut buf_old, block)
			for i in 0 .. block {
				k := f32(math.min(1.0, f64(fade_pos + i) / f64(fade_len)))
				buf[2 * i] = buf[2 * i] * k + buf_old[2 * i] * (1 - k)
				buf[2 * i + 1] = buf[2 * i + 1] * k + buf_old[2 * i + 1] * (1 - k)
			}
			fade_pos += block
		}
		app.ring.write(buf, block)
		busy += f64(time.sys_mono_now() - t0) * 1e-9
		tel_timer += block
		if tel_timer >= int(rate / 30.0) {
			el := wall.elapsed().seconds()
			mut t := s.tel
			t.sim_load = busy / math.max(el, 1e-6)
			t.underruns = app.ring.underrun_frames()
			t.buffered = app.ring.fill()
			app.tel.publish(t)
			// grow the buffer target if the device starved
			if t.underruns > last_underruns && app.target < app.ring.capacity() / 2 {
				app.target += block
				last_underruns = t.underruns
			}
			busy = 0
			wall.restart()
			tel_timer = 0
		}
	}
}

fn run_realtime(cfg config.Config, o Options) ! {
	rate := cfg.audio.sample_rate
	ring := audio.new_ring_buffer(rate, 2)
	mut ctl := input.new_controls(cfg.simulation.start_running, cfg.drivetrain.auto_clutch,
		cfg.drivetrain.rev_match)
	mut out := audio.Output{}
	mut actual_rate := rate
	if !o.no_audio {
		out = audio.open_output(ring, rate, cfg.audio.buffer_frames)
		if !out.valid {
			eprintln('warning: could not open the audio device; running silently')
		} else {
			actual_rate = out.sample_rate
		}
	}
	first := sim.new_simulator(cfg, actual_rate)!
	target := int(cfg.audio.target_latency_ms * 0.001 * f64(actual_rate)) + cfg.audio.block_frames
	mut app := &App{
		cfg_path: o.config_path
		cfg:      cfg
		ctl:      ctl
		ring:     ring
		tel:      sim.new_telemetry_box()
		running:  1
		target:   target
	}
	if out.valid {
		latency := f64(target + out.buffer_frames) / f64(actual_rate) * 1000.0
		println('audio: ${actual_rate} Hz, device buffer ${out.buffer_frames} frames, sim block ${cfg.audio.block_frames}, ~${latency:.0f} ms output latency')
	}
	th := spawn sim_thread(mut app, first)
	if o.headless {
		run_terminal(mut app)
	} else {
		mut w := ui.new_window(app.ctl, app.tel, cfg, reload_fn(mut app))
		w.watch_config(o.config_path)
		w.run()
	}
	stdatomic.store_u64(&app.running, 0)
	th.wait()
	out.close()
}

// reload_fn returns the hot-reload callback used by the window. It builds a
// new simulator on the calling (UI) thread and hands it to the sim thread.
fn reload_fn(mut app App) fn () string {
	return fn [mut app] () string {
		if stdatomic.load_u64(&app.pend_flag) == 1 {
			return 'reload already pending'
		}
		cfg := config.load(app.cfg_path) or { return 'reload failed: ${err}' }
		mut ns := sim.new_simulator(cfg, int(app.cfg.audio.sample_rate)) or {
			return 'reload failed: ${err}'
		}
		ns.sync_counters(app.ctl.snapshot())
		app.cfg = cfg
		app.pending = ns
		stdatomic.store_u64(&app.pend_flag, 1)
		return 'reloaded ${app.cfg_path}'
	}
}

// run_terminal prints the readout until the process is interrupted, checking
// the config for changes (hot reload) every half second.
fn run_terminal(mut app App) {
	println('running headless; press Ctrl+C to quit (edit the config to hot-reload)')
	reload := reload_fn(mut app)
	mut mtime := os.file_last_mod_unix(app.cfg_path)
	for {
		time.sleep(500 * time.millisecond)
		t := app.tel.read()
		println(ui.readout_line(t, app.cfg))
		m := os.file_last_mod_unix(app.cfg_path)
		if m != mtime {
			mtime = m
			println(reload())
		}
		if app.ctl.flag(input.flag_quit) {
			break
		}
	}
}

// play_tone checks the audio path: a 440 Hz tone rendered by a producer
// thread into the ring buffer, read by the device callback.
fn play_tone(o Options) {
	rate := if o.sample_rate > 0 { o.sample_rate } else { 48000 }
	mut ring := audio.new_ring_buffer(rate, 2)
	out := audio.open_output(ring, rate, 512)
	if !out.valid {
		eprintln('error: could not open the audio device')
		exit(1)
	}
	println('playing a 440 Hz test tone for 2 s at ${out.sample_rate} Hz (device buffer ${out.buffer_frames} frames)')
	mut phase := 0.0
	block := 64
	mut buf := []f32{len: block * 2}
	sw := time.new_stopwatch()
	for sw.elapsed().seconds() < 2.0 {
		if ring.fill() >= 1024 {
			time.sleep(time.millisecond)
			continue
		}
		for i in 0 .. block {
			env := math.min(1.0, sw.elapsed().seconds() * 10.0) * math.min(1.0, (2.0 - sw.elapsed().seconds()) * 10.0)
			v := f32(0.25 * env * math.sin(phase))
			buf[2 * i] = v
			buf[2 * i + 1] = v
			phase += 2.0 * math.pi * 440.0 / f64(out.sample_rate)
		}
		ring.write(buf, block)
	}
	time.sleep(100 * time.millisecond)
	out.close()
	println('done; underrun frames: ${ring.underrun_frames()}')
}
