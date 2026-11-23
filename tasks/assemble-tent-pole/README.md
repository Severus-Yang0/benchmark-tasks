# Assemble a folded shock-corded tent pole: swing each end section round its joint against the cord's pull and seat both joints

A three-section shock-corded tent pole lies folded on the table, as it comes out of its bag: the middle section across
the table, the two end sections folded back beside it at their joints, the elastic cord inside pulling every joint
together. The robot must unfold the pole, push both joints fully home, and let go of it.

Dome and tunnel tents go up on shock-corded poles, and pitching one starts with unfolding each pole from its bundle and
seating every joint: over 52 million North American households camped in 2025 (KOA, *2026 Camping & Outdoor
Hospitality Report*). Pole makers warn that joints not fully pushed together put extra strain on the connection and may
cause the pole to fail (Terra Nova, *Pole Care and Maintenance*).

The hard part is physical. The cord pulls every joint together all the time (about 4.6 N with the pole folded), so a
20 g section swung round its fold is dragged at its end the whole way round. One hand has to hold the middle section
still, or the joint already made is pulled apart while the other is made. An insert goes home only once it is in line
with its socket: 8.3 mm in 8.5 mm, 30 mm deep.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the shock cord's pull, `score()`, `success()`, the let-go watch and `self_test()` |
| `dims.py` | The pole's sizes, weights and the cord's tension, with their sources |
| `scenekit.py` | USD bodies for yam_arena task files (hooks yam_arena without changing it) |
| `assets/section_1.usda` (11 KB) | Section 1: endpiece with insert |
| `assets/section_2.usda` (19 KB) | Section 2: center piece, socket at one end, insert at the other |
| `assets/section_3.usda` (11 KB) | Section 3: endpiece with socket |

No third-party assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Section 2 (center piece) | across the table along y: its socket's mouth at (30.0, -20.0), its insert's end at (30.0, 23.0) | DAC Featherlite NSL 9 mm, 7001 aluminium, golden anodised: a 40 cm tube, 9.0 mm outside, 7.8 mm inside; its first 35 mm widened to 9.5 mm outside, 8.5 mm inside (the socket); at the other end a silver aluminium insert, 8.3 mm outside, 6.3 mm inside, standing 30 mm out; 19.8 g |
| Section 1 (endpiece with insert) | folded back beside section 2 at joint 1, fanned 20 degrees away on the far side: its insert's end at (30.8, -20.4), beside section 2's mouth; its capped end at (45.5, 20.0) | The same tube with an insert at one end and a cap at the other; 19.5 g |
| Section 3 (endpiece with socket) | folded back beside section 2 at joint 2, fanned 15 degrees away on the far side: its socket's mouth at (30.9, 22.6), beside section 2's insert; its capped end at (41.2, -16.0), crossing over section 1 and resting on it | The same tube with a socket at one end and a cap at the other; 16.7 g |
| Shock cord | inside the pole, tied at both its ends | 3 mm polyester-covered shock cord, 84 cm long unstretched (70 % of the 1.20 m pole) |

Assembled, the pole is 1.20 m long.

Sources: sections, extremtextil.de articles 71301 (center piece: "40cm (without insert), 43cm (with insert)", 19.8 g),
71304 (endpiece with insert, 19.5 g) and 71307 (endpiece with insert entry), and Shelby Outdoor's listing of the same
center piece ("400 mm + 30 mm insert tube"; outside 9.0 mm, inside 7.8 mm; widened end 8.5 mm inside; insert 8.3 mm);
section 3's weight is the center piece's less its insert's (48 mm of 8.3 / 6.3 mm 7001 tube, 3.1 g). Cord, Marlow
Ropes datasheet MA-DA-014 (3 mm: 7 to 11 N at 100 % elongation; 9 N used); its length, Hilleberg's pole instructions
("cut a new length of shock cord that is 70% the length of your pole"). Practice, Terra Nova, *Pole Care and
Maintenance* (terra-nova.co.uk/blog/how-tos/pole-care-and-maintenance). Campers, KOA, 2026 Camping & Outdoor
Hospitality Report (koa.com/north-american-camping-report).

## Run a trial

From this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the folded tent pole: unfold it and push both joints fully home, then let go of the pole." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pole-dream
```

`task_score` prints the trial's score: 1.0 on success (held for 1 s), otherwise `score()` at the end of the episode.
A trial takes about 3 minutes on one RTX 4080 (scene start-up included). Seed 0; the scene has no random placement
(`-E object_jitter=0`).

Offline check (no GPU): `python task.py` runs 55 checks of the scoring rules, the joints' geometry, the cord's pull and
the start layout.

## Demo

The demo video is a scripted demo (both arms at once). The right hand holds the middle section; the left hand takes
section 3 near its fold; both lift the pole 2 cm, so the cord between them pulls level. The left hand swings section 3
round its mouth, about 165 degrees in five steps, each aimed at where the pole really is, until it lies in line with the
middle section; it lines the socket up on the insert and pushes it home. Both hands lower the pole; the
left hand takes the middle section and the right hand unfolds section 1 the same way round its insert's end, the other
way round, and pushes joint 1 home. Both hands lower the pole, let go and move away; the trial reaches 1.0 at about
40 s (about 800 steps). Five runs of the demo all scored 1.0.

Left alone, the folded pole stays folded: a trial in which the robot does nothing scores 0.

## Scoring

The score at the end of the episode counts. Tools for a real trial: a ruler and a 1 mm feeler gauge.

- **Home:** the insert is in its socket and the 1 mm feeler gauge does not go in between the two tube ends. In
  simulation: the insert's tip at least 29 mm into the socket's bore and within 1 mm of its axis.
- **Started:** at least 10 mm of the insert is in the socket (no more than 20 mm of it showing).
- **Let go:** no gripper within 3 cm of the pole (in simulation, of any section's axis).

| Outcome | Score |
|---|---|
| Both joints open | 0 |
| For each joint started (not home) | +0.2 |
| For each joint home | +0.4 |
| Both joints home, the robot still holding the pole | 0.8 |
| Both joints home and the robot off the pole, held for 1 s | 1.0 |

## Reset

Pull both joints apart against the cord and lay the folded pole back as in the setup.

## Real trials

The pole weighs 56 g and the cord pulls with under 5 N; nothing is sharp, hot or powered. The sections are standard
DAC replacement parts (a few euros each) on a standard 3 mm shock cord.
