# Research summary

This document comes before the code. For each topic it gives a short summary
of what the physics and prior art say, then the approach this project takes
and why. The numbers quoted are the defaults the implementation starts from.

---

## 1. Prior art

### AngeTheGreat's engine-sim (C++)

engine-sim (`github.com/ange-yaghi/engine-sim`, now continued as
"engine-sim community edition") is the reference for "the sound comes out of
the simulation". Reading its source shows this architecture:

* **Rigid-body mechanics.** Crankshaft, connecting rods and pistons are rigid
  bodies joined by constraints (`atg_scs`, a sequential constraint solver).
  The simulation runs at a configurable *simulation frequency*, typically
  10–20 kHz.
* **Lumped gas systems.** Each cylinder (`CombustionChamber`), each intake
  plenum and each exhaust runner is a `GasSystem`: a control volume tracking
  moles, energy and a crude momentum term. Flow between volumes uses the
  isentropic compressible-orifice equation, both choked and subsonic
  (`GasSystem::flowRate`), with an effective area set by valve lift and flow
  bench data (CFM at 28 inH2O). Gas flow runs 8 fluid substeps per mechanical
  step.
* **Combustion.** Driven by an ignition module. Flame propagation speed
  depends on turbulence, which scales with RPM, and on mixture.
* **Audio path** (`PistonEngineSimulator::writeToSynthesizer` →
  `Synthesizer::renderAudio`):
  1. Each cylinder's *exhaust runner pressure* minus 1 atm (plus a small
     dynamic-pressure term) becomes the "exhaust flow" signal.
  2. A delay filter delays it by `(header length + exhaust length) / 343 m/s`,
     scales it by `1/L²`, and sums it per exhaust system.
  3. The synthesizer resamples the simulation-rate signal to 44.1 kHz. It
     adds jitter and "air noise" (noise modulated by the signal), mixes in a
     derivative, and **convolves with a recorded impulse response** of a real
     exhaust. Then it runs a leveling filter (AGC) and clips to int16.
  4. The synthesizer runs on its own thread with mutex/condition-variable
     hand-off.

  The takeaways: the timbre comes mostly from the **time pattern of exhaust
  pulses** (firing order, header delays) and from a **linear filter
  representing the exhaust and its environment**. engine-sim gets that filter
  from a measured IR. Real waves in the exhaust pipes are not simulated.

### Procedural engine audio in games and research

* **Commercial racing games** (Forza, Gran Turismo, Assetto Corsa and so on)
  almost all use **recorded samples**: on/off-throttle loops at fixed RPMs,
  crossfaded and pitch-shifted, or granular synthesis such as AudioMotors and
  REV. This is not what we want, but it confirms that on-load vs. off-load
  timbre, and source position (exhaust, intake, engine bay, cabin), matter as
  much as RPM.
* **Andy Farnell, *Designing Sound* (MIT Press, 2010).** Procedural engines
  built from a cylinder pulse train driving waveguide (delay-line) models of
  the exhaust.
* **Baldan, Delle Monache et al., "Physically informed car engine sound
  synthesis for virtual and augmented environments" (2015)** and the
  **Sound Design Toolkit `motor` module.** The engine cycle drives resonators
  built from **digital waveguides** for the intake, cylinders, exhaust
  collector, exhaust pipe and muffler, with feedback gains standing in for
  reflection. This is closest to the approach chosen here.
* **J. O. Smith, *Physical Audio Signal Processing*** (CCRMA). Digital
  waveguides, scattering junctions and fractional-delay interpolation.

### Approach chosen

**A hybrid model.** Nonlinear lumped thermodynamics for the parts that are
nonlinear: cylinders, ports, plenum, valve and throttle orifices. Linear
**digital waveguides** for all the pipes: headers, collectors, mid-pipes,
mufflers, tailpipes, intake runners and snorkel. Sound is the tailpipe and
intake-mouth **volume-velocity derivative** (monopole radiation), plus
smaller mechanical sources.

Why: engine-sim shows that pulse timing plus an exhaust filter gives
convincing sound. Using waveguides instead of a measured IR means the
"filter" comes from the configured pipe lengths and diameters, so it changes
when the config changes and needs no recordings, which is the stated goal.
The waveguides also carry real reflections back to the valves, so tuned
headers affect volumetric efficiency the way they do in a real engine.
I dropped engine-sim's full rigid-body constraint solver. A slider-crank has
a closed-form solution, and the Lagrangian of a single rotating crank with
reciprocating masses (§3) captures the same speed ripple at a tiny fraction
of the cost.

---

## 2. Four-stroke thermodynamics

**Cycle.** Over 720° of crank: intake (TDC→BDC, intake valve open), compression
(BDC→TDC, both closed), power (TDC→BDC, combustion near TDC then expansion),
exhaust (BDC→TDC, exhaust valve open). Valve events are offset from the dead
centres. The exhaust opens 40–70° before BDC ("blowdown", which is where
most of the exhaust *sound* comes from: a near-choked burst at 3–6 bar). The
intake closes 30–70° after BDC to use the charge's inertia. Overlap
(both valves open around TDC) scavenges at high RPM, but at idle it causes
backflow and residual gas. That is the "lopey cam" sound.

**State per control volume.** Mass `m`, internal energy `E` and burned-gas
fraction `b`. With the ideal-gas law, `p = (γ−1)·E/V` and `T = p·V/(m·R)`,
so pressure needs no `pow`. The energy equation per step:

```
dE = −p·dV + dQ_comb − dQ_wall + Σ h_in·dm_in − Σ h_out·dm_out,   h = γ/(γ−1)·p/ρ
```

`γ` is mixed from fresh charge (≈1.35) and burned gas (≈1.27) using `b`.

**Valve and throttle flow.** The standard compressible orifice equation with
an effective area `A_eff = Cd · min(π·d·L(θ), π·d_port²/4)`:

```
ṁ = A_eff · p0/√(R·T0) · ψ(pr),   pr = p_down/p_up
ψ = √γ·(2/(γ+1))^((γ+1)/(2(γ−1)))                    (choked, pr < 0.528)
ψ = √(2γ/(γ−1)·(pr^(2/γ) − pr^((γ+1)/γ)))             (subsonic)
```

`ψ(pr)` comes from a 1024-entry lookup table. Near `pr → 1` it is replaced by
a linear segment, because `dψ/dpr → ∞` there would make explicit integration
chatter. Each transfer is also clamped to the amount that would equalise the
two pressures.

**Combustion (Wiebe function).** Mass fraction burned:

```
x_b(θ) = 1 − exp(−a·((θ − θ_s)/Δθ)^(m+1)),   a = 5, m = 2
```

`θ_s` = spark + ignition delay. `Δθ` (≈40–70°) grows with residual-gas
fraction and is randomised per cycle, ±4–15%, larger at light load and high
residual. This *cycle-to-cycle variation* is an important part of a living
idle. Heat released is
`Q = m_fresh/(1+AFR) · LHV · η_c`, with `LHV = 44 MJ/kg` for gasoline and
`η_c` ≈ 0.85–0.9 lumping incomplete combustion and dissociation. Wiebe is
precomputed in a table of `x_b` vs. normalised progress.

**Wall heat transfer.** A Woschni-style `h ∝ p^0.8·w^0.8·T^-0.53·B^-0.2`
with `w ∝` mean piston speed, recomputed every few steps (the `pow` calls are
the expensive part).

**Volumetric efficiency** is *not an input*. It falls out of throttling,
valve timing and pipe reflections. The config sets only geometry and cams.

**Valve lift.** Each cam lobe is defined by opening/closing angles (or
centreline + duration) and peak lift, with a smooth `sin²`-shaped profile
(continuous velocity, finite acceleration). Lift is precomputed per 0.25°
over 720°.

**Chosen:** a lumped 0-D model per cylinder plus intake/exhaust port volumes.
It is the standard "filling-and-emptying" engine model (Heywood ch. 14). It
is accurate enough for pressure-crank-angle traces and cheap enough to run
at ~100 kHz for 12 cylinders.

---

## 3. Engine geometry and why layouts sound different

**Slider-crank.** For crank radius `r = stroke/2` and rod length `l`, piston
distance from crank axis is `x(θ) = r·cos θ + √(l² − r²·sin²θ)`. Cylinder
volume is `V = V_c + A_p·(l + r − x)`, where `V_c = V_d/(CR − 1)`.
`dV/dθ` is analytic, so **gas torque is exactly `T = (p − p_crankcase)·dV/dθ`**.

**Reciprocating mass.** Treating piston + ⅓ rod as a point mass `m` moving
with `s(θ)` gives a Lagrangian with angle-dependent inertia:
`I_eff(θ) = I_crank + Σ m·s'(θ)²` and an extra torque `−Σ m·s'·s''·ω²`.
This reproduces 2nd-order speed ripple (strong in I4s, absent in I6s)
without a constraint solver.

**Layout from data only.** A cylinder sits on a *bank* (angle from vertical)
and on a *crank throw* (angle on the crank). Cylinder `i` reaches TDC when the
throw lines up with its bank axis: `θ_TDC = bank_angle − throw_angle`
(mod 360). Each cylinder has two TDCs per cycle. Starting from the first
cylinder in the firing order, we pick for each next cylinder the TDC (`θ` or
`θ+360`) that comes soonest after the previous firing. That determines all
firing angles from geometry plus firing order. Shared crankpins (V-twins,
crossplane V8s), offset pins (odd-fire V6) and any bank angle work
automatically, with no special cases. Inconsistent configs (two cylinders
firing at the same angle) are rejected with a message.

**Why the reference layouts sound different:**

| Layout | Firing interval | Character |
|---|---|---|
| Inline-4 (flat crank, 1-3-4-2) | even 180° | Strong 2nd order; two pulses per rev, buzzy/thin. Big 2nd-order inertial ripple. |
| Inline-6 (1-5-3-6-2-4) | even 120° | 3 pulses/rev, perfectly balanced. Smooth "silky" tone. Usually a 3-into-1 ×2 manifold. |
| Cross-plane V8 (90° throws, 90° bank) | even 90° *overall*, but **per bank** 180/90/180/270 | Each bank's exhaust sees uneven pulses. The classic "burble"/"rumble" has a strong half-order (0.5×, 1×) content unless an X-pipe merges the banks. |
| Flat-plane V8 (180° throws) | even 90° overall **and** even 180° per bank | Each bank behaves like an I4. High-pitched, "screaming" sound with no burble (Ferrari, Coyote GT350). |
| Boxer-4 (opposed banks, 180° throws) | even 180° | Even firing, but with unequal-length headers (typical of Subaru) pulses arrive unevenly at the collector, giving the "boxer rumble". With equal-length headers it sounds like an I4. |
| 45° V-twin, shared pin | 315°/405° | "Potato-potato" lope; very strong sub-harmonics. |
| 90° V-twin (L-twin) | 270°/450° | Uneven but less extreme; a "Ducati" bark. |
| 60° V12 | even 60° | 6 pulses/rev, very high fundamental. With 6-into-1 per bank each side is an even 120° I6. Smooth and shrieking. |

The other big factors are **header length differences** (pulse arrival
spreading), **exhaust merging** (true duals vs. Y vs. X/H-pipe) and
**cam overlap** (idle lope).

**Chosen:** explicit per-config `banks`, `throws`, `cylinders` and
`firing_order`, with firing angles computed as above. Nothing about the
layout is hard-coded.

---

## 4. Exhaust and intake acoustics (waveguides)

**1-D wave propagation.** For pipe diameters well below the wavelength
(first cross-mode of a 50 mm pipe at ~600 K ≈ 8 kHz), sound in a pipe is a
superposition of forward and backward travelling waves:
`p = p⁺ + p⁻`. A **digital waveguide** is two delay lines of length
`L/(c·Δt)` samples. With sound speed `c = √(γRT)`, exhaust gas at 600–900 K
gives `c` ≈ 490–600 m/s, so pipe resonances are much higher than the same
pipe cold. Fractional delays use linear interpolation, plus a one-pole
lowpass and gain per traversal for viscothermal wall loss (`∝ √f / a`).

**Mass flow.** Volume velocity is `U = (p⁺ − p⁻)/Z` with `Z = ρc/A`. The
mass flow `ṁ = ρU = (A/c)·(p⁺ − p⁻)` **does not depend on ρ**. The
waveguides carry mass flow through the system exactly (in the linear
approximation) even when the intake plenum is at 0.3 bar.

**Junctions.** At a point where N pipes meet (collector, area step, side
branch), continuity of pressure plus conservation of mass gives the
scattering junction

```
p_J = 2·Σ Y_i·p_i⁻ / Σ Y_i ,   Y_i = A_i/c_i ,   p_i⁺ = p_J − p_i⁻
```

An area change is a 2-pipe junction (reflection coefficient
`(A1−A2)/(A1+A2)`). A collector is an N-pipe junction. This one primitive
covers headers, 4-2-1 secondaries, collectors, X/H crossovers and chambers.

**Ends.**
* *Closed end* (valve shut): `R = +1`.
* *Open end* (tailpipe, intake mouth): `R ≈ −1` at low frequency, falling as
  `ka` grows (Levine–Schwinger). This is implemented as `−(1−g)` through a
  one-pole lowpass with cutoff `≈ c/(2π·a·0.6)`, plus an end-correction
  delay of `0.61·a`. The *transmitted* (radiated) part is the high-passed
  remainder.
* *Lumped volume end* (exhaust port, intake plenum): the volume's pressure
  sets the pipe end pressure, and `ṁ = (p_V − 2p⁻)·A/c`. This is solved
  **implicitly** (linearised backward Euler) so it stays stable even with
  tiny port volumes.

**Radiation.** A tailpipe or intake mouth radiates as a monopole:
`p_far(t) = ρ/(4π r) · dU/dt(t − r/c)`. The audio is the derivative of the
outlet volume velocity, scaled by distance to the listener. For a 2 L engine
this gives ~100 dB at 1.5 m without any tuning.

**Pipe length and diameter** set resonance frequencies (`f_n ≈ n·c/2L` for
open-open, `(2n−1)·c/4L` for closed-open) and impedance. Area steps set how
much is reflected. Long, narrow headers give a slower, "tuned" sound.
Shorter, wider ones give a sharper bark.

**Chosen:** waveguides at the simulation rate (`Δt` = 1/96 kHz → ~5 mm per
sample at 500 m/s), linear acoustics, lumped port volumes at the valve end,
scattering junctions everywhere else. Nonlinear wave steepening is ignored.
Exhaust temperature is interpolated from header to tailpipe so `c` falls
along the path.

---

## 5. Mufflers, resonators, catalysts

All are **acoustic filters** and all are built from the same waveguide
primitives:

* **Expansion chamber** (reactive): pipe → large area → pipe. Transmission
  loss `TL = 10·log10(1 + ¼(m − 1/m)²·sin²(kL))`, with `m` the area ratio. In
  the model it is just a short wide pipe between two narrower ones; the
  junctions produce the TL and its periodic pass-bands automatically. A
  "chambered" muffler is two or three chambers with connecting tubes.
* **Helmholtz resonator** (side branch: neck + cavity): resonates at
  `f = c/2π·√(A_n/(V·L_eff))`, where `L_eff = L + 1.7·a_n`. It is a sharp
  notch, usually tuned against drone at cruise RPM. Modelled as a lumped
  mass–spring–damper (neck inertance `ρL/A`, cavity compliance
  `V/(ρc²)`, resistance for damping) hung on a junction. The neck flow is
  integrated with symplectic Euler.
* **Quarter-wave tube** (side branch closed at the far end): notch at
  `c/4L`. A waveguide with `R = +1` on a junction.
* **Absorptive / glasspack** (perforated pipe through fibre packing): mostly
  broadband high-frequency attenuation with little low-frequency loss, so
  glasspacks keep the bass. Modelled as a waveguide with strong per-pass
  lowpass and loss inside a slightly larger chamber.
* **Catalytic converter**: a large-area honeycomb brick (an expansion with
  heavy viscous loss and a small flow resistance). Modelled as a lossy
  chamber.
* **Tailpipe radiation**: open-end model (§4). High frequencies radiate more
  efficiently, which is the natural lowpass/highpass split.

**Chosen:** an `exhaust.sections` list per exhaust path with types `pipe`,
`chamber`, `absorptive`, `catalyst`, `helmholtz`, `quarter_wave`, plus a
crossover (`x`/`h`) option between banks. Mufflers are composed from these
in config, so a "straight pipe", "glasspack", "chambered" or "OEM
3-chamber + resonator" system is just data.

---

## 6. Drivetrain and vehicle

**Rotating parts:**
* Engine: crank + flywheel inertia `I_e` (0.08–0.3 kg·m² for cars,
  ~0.02–0.05 for bikes) plus the reciprocating-mass term (§3).
* **Friction (FMEP)**: `fmep = a + b·S_p + c·S_p²` in kPa, with `S_p` the
  mean piston speed (Chen–Flynn form). Torque `= fmep·V_d/(4π)`. A `tanh`
  near zero speed gives stiction without a discontinuity. **Pumping losses
  are not modelled separately**: they emerge from the gas exchange
  (throttled intake, exhaust back-pressure).
* **Clutch**: torque capacity `T_max·engagement(pedal)`. The *discrete-time
  friction* method computes the torque that would bring the two sides to the
  same speed in this step, `T_req = (Δω + Δt·(T_e/I_e − T_g/I_g)) / (Δt·(1/I_e + 1/I_g))`,
  then clamps it to ±capacity. This handles slip, lock-up and re-slip
  exactly, with no stick-slip chatter and no special-case state.
* **Gearbox**: ratios plus efficiency. A shift goes to neutral for
  `shift_time`. The new gear's synchroniser is a friction coupling (same
  discrete friction method) that pulls the input shaft into sync. With the
  clutch in, it syncs the light input shaft quickly. Shifting without the
  clutch makes the synchro drag the whole engine to the new speed (a
  "clutchless" shift with a jolt). Optional **auto-clutch** and
  **rev-matching** (throttle blip to the target RPM on downshifts) help
  keyboard drivers.
* **Driveshaft compliance**: a torsional spring-damper between gearbox
  output and wheels (~3–8 Hz "shunt" mode). It gives realistic RPM
  oscillation on tip-in and after a shift.
* **Final drive** ratio and **wheels** (radius, inertia).

**Tyres.** Longitudinal friction with peak `μ_peak` and sliding `μ_slide`,
also using the discrete friction method between wheel surface speed and
vehicle speed. Wheelspin happens when demand exceeds `μ·N`. The driven-axle
normal load includes longitudinal weight transfer `m·a·h/L`.

**Vehicle.** `m·dv/dt = F_tyre − C_rr·m·g − ½ρ·C_d·A·v² − F_brake`.
Engine load is whatever the tyre needs to accelerate this mass, so 1st
gear revs fast and 5th gear lugs.

**Starter**: torque `T_stall·(1 − ω/ω_free)` through a one-way clutch.
**Idle**: a PI controller on an idle-air bypass orifice around the throttle,
which is what real ECUs do. **Rev limiter**: spark (or fuel) cut with
hysteresis, which gives the audible "brrap-brrap". **Launch control**: a
lower limiter while stationary with the clutch in. **Overrun fuel cut**
above idle with the throttle closed.

**Chosen:** everything above runs at the full simulation rate (cheap,
5 states). The discrete friction method makes it unconditionally stable.

---

## 7. Real-time audio in V

**Backends.** V ships `sokol.audio` (a wrapper for `sokol_audio.h`). It
covers ALSA on Linux, WASAPI on Windows, CoreAudio on macOS and others. It
runs its *own* audio thread and calls a stream callback
`fn (buffer &f32, num_frames i32, num_channels i32)` for interleaved f32
frames. `buffer_frames` sets the device buffer (default 2048 = 46 ms at
44.1 kHz, too high for us). Alternatives: miniaudio via C interop (an extra
vendored dependency) or `audio.push()` (a sokol FIFO, which adds another
buffer). sokol.audio's callback model is exactly what we need.

**Latency budget** (target < 30 ms input→sound at 48 kHz):

| Stage | Default | Latency |
|---|---|---|
| Key event → sim (UI event loop at 60 Hz) | — | ≤ 16 ms, ~8 avg |
| Sim block size | 64 frames | 1.3 ms |
| Ring buffer target fill | 512 frames | ~10 ms |
| Device buffer | 512 frames | 10.7 ms |

That totals about 25–30 ms average. All of these are configurable.

**Never blocking the audio thread.** The callback only reads from a
**lock-free single-producer/single-consumer ring buffer** (power-of-two
capacity, monotonically increasing `u64` read/write indices with atomic
loads and stores). No locks, allocation, logging or syscalls. On underrun it
fades the last sample to zero (no click), counts the event, and the
producer raises its target fill.

**Never starving.** The simulation thread runs ahead of the reader. It
renders in small blocks until the buffer reaches the target fill, then
sleeps ~1 ms. The audio device clock therefore paces the sim, so there is
no drift. Control inputs are read once per block from atomics. Telemetry is
published once per block through a seqlock-style snapshot. The hot loop
allocates nothing (all buffers are sized at load time), so V's GC has
nothing to collect from it. Short GC pauses caused by the UI thread are
absorbed by the ring buffer.

**Chosen:** `sokol.audio`, 48 kHz stereo, 512-frame device buffer, SPSC
ring, and `gg` (sokol_app) for the window and key up/down events, as agreed.
There is also a headless WAV renderer driven by a control script, used for
tests and for sharing renders.

---

## 8. Numerical methods

**Fixed timestep.** All physics use one fixed `Δt = 1/(f_audio·substeps)`.
Waveguide delays are defined in samples of this `Δt`, so a fixed step is
mandatory. It also makes the simulation deterministic and makes the
benchmark meaningful.

**Stiffness inventory** (explicit Euler is stable for `Δt < ~2τ` and
accurate for `Δt ≪ τ`):

| Coupling | Time constant | at 96 kHz (`Δt` = 10.4 µs) |
|---|---|---|
| Cylinder ↔ port through valve (V_port ≈ 50 cc, A ≈ 8 cm², c ≈ 600) | ~100 µs | ratio ~10, OK |
| Port ↔ waveguide | ~40 µs | **implicit** (solved exactly) |
| Valve flow near Δp → 0 | → 0 | linearised ψ + transfer clamp |
| Helmholtz neck | ~0.5 ms | OK |
| Clutch / synchro / tyre friction | stiff | discrete friction method (exact) |
| Driveshaft spring (k ≈ 10⁴ Nm/rad) | ~30 ms | trivial |
| Compression stroke `−p·dV` at 9000 rpm | 180° = 3.3 ms ≈ 320 steps | OK |

**Substeps.** At 1× (48 kHz) blowdown on small port volumes is marginal, and
the audio sees aliasing from the sharp blowdown edges. At **2× (96 kHz)**
everything is comfortably stable and aliasing is removed by a windowed-sinc
FIR decimator (cutoff 0.45·f_audio). 3–4× changes the sound only slightly.
Default: 2 substeps, configurable 1–4.

**Integrators.** Symplectic (semi-implicit) Euler for crank and drivetrain
(`ω += α·Δt; θ += ω·Δt`) conserves energy well for oscillatory modes.
Forward Euler with clamps for gas exchange. Implicit linearised update for
port–pipe coupling. Exact delay lines for waves.

**Cost.** Per step, each cylinder needs roughly 2 table lookups for lift,
2 orifice flows (table + `sqrt`), 1 `sqrt` for kinematics, and occasional
Wiebe/heat-transfer lookups. Waveguides cost ~4 memory ops per pipe end plus
a junction sum. That is O(cylinders + pipes) per step, with no
O(pipe length) work. Expected cost is well under one core for a V12 at
96 kHz. The benchmark (`v run bench/`) reports the real-time factor for
each example config.

---

## Sources

* engine-sim source — <https://github.com/ange-yaghi/engine-sim>
  (`piston_engine_simulator.cpp`, `gas_system.cpp`, `synthesizer.cpp`)
* engine-sim community edition — <https://github.com/Engine-Simulator/engine-sim-community-edition>
* Baldan, Delle Monache et al., *Physically informed car engine sound synthesis
  for virtual and augmented environments* (2015) — <https://air.iuav.it/handle/11578/264484>
* Sound Design Toolkit, `motor` module — <https://sdt.lim.di.unimi.it/docs/API/v2.2-078/group__motor.html>
* *Four Decades of Digital Waveguides* (survey, 2026) — <https://arxiv.org/pdf/2604.12878>
* J. B. Heywood, *Internal Combustion Engine Fundamentals* (Wiebe, Woschni, FMEP, filling-and-emptying models)
* J. O. Smith III, *Physical Audio Signal Processing* — <https://ccrma.stanford.edu/~jos/pasp/>
* M. L. Munjal, *Acoustics of Ducts and Mufflers* (expansion chamber TL, Helmholtz, perforates)
* H. Levine & J. Schwinger, "On the radiation of sound from an unflanged circular pipe", Phys. Rev. 73 (1948)
* A. Farnell, *Designing Sound* (MIT Press, 2010)
* sokol_audio.h — <https://github.com/floooh/sokol> and V's `sokol.audio` wrapper — <https://modules.vlang.io/sokol.audio.html>
