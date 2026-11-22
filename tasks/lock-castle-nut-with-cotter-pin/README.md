# Lock a castle nut with a cotter pin: push it through the slot and the stud's cross-hole, then spread its prongs

A car's steering joint stands on a steel stand as it is with the wheel off: the knuckle body, the steering arm coming
off it, and through the arm an upright tie-rod-end stud with a slotted (castle) nut on top, one slot lined up over the
stud's cross-hole. A new standard cotter pin lies in a groove on an aluminium block, and a precision flat-head
screwdriver stands in a rack. The robot must push the pin through the nut's slot and the stud's hole and spread its two
prongs apart round the nut so the pin can't come out.

Automotive service technicians (about 826,000 US jobs; BLS *Occupational Outlook Handbook*, 2025) do this whenever
they rebuild steering linkages (O*NET 49-3023.00, task: "Tear down, repair, and rebuild faulty assemblies, such as
power systems, steering systems, and linkages."). Service manuals call for a new pin every time, its prongs bent round
the nut once it is tightened to the next slot (Suzuki RA410 service manual: "bend new split pin"; Meritor bulletin:
tighten to the next slot, never back the nut off to line up the hole). An unpinned tie-rod nut can work loose, and the
steering with it.

The hard part is physical. The pin (2.2 mm across) has to go in square through a 3.0 mm hole hidden under the nut's
slot, between the steering arm, the knuckle and the tie rod. Its two prongs, 1.1 mm thick each, lie together: a
parallel gripper can't take hold of one, so a tool has to catch the end where the longer prong runs on past the shorter
one, open a gap, and push each prong round past its yield (about 12 N, 10 mm from its root). While the prongs are
pushed, the pin's head has to be held home, or the pin turns and slides in the slot.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `score()`, `success()`, the released watch and `self_test()` |
| `cotterkit.py` | The prongs' bending: each prong's hinge is elastic up to the prong's plastic moment and keeps its bend past it |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `touch.py` | Robot-to-body contacts from PhysX: whether the robot still touches the pin |
| `assets/fixture.usda` (19 KB) | The steering joint: knuckle body, brake dust shield, steering arm, tie-rod end (ball housing, boot, tie rod), 3/8-24 stud with its 3.0 mm cross-hole, 3/8-24 slotted hex nut |
| `assets/cotter.usda` (21 KB) | The cotter pin: eye and shank, two prongs hinged at their roots |
| `assets/screwdriver.usda` (4 KB) | The precision screwdriver: thin straight tip, tapering blade, handle |

No third-party assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Stand | x 27.5 to 44.0, y -8.5 to 11.5 | Steel, 16.5 x 20 x 5 cm; the steering joint stands on it |
| Steering arm | from the knuckle (x 40.0) to just past the stud | Steel, 3.4 cm wide and 1.4 cm thick, its top 9.0 cm up |
| Knuckle body, dust shield | x 40.0 to 42.5; x 43.0 | Knuckle 6 cm wide, 8.5 cm tall; brake dust shield 16 cm wide, 11 cm tall |
| Tie-rod end | under the arm, round the stud | Ball housing 3.2 cm across and its rubber boot; the tie rod (1.8 cm thick) running off to y 11.0 |
| Stud | upright at (30.0, 0.0) | 3/8-24 steel (9.5 mm), its 3.0 mm cross-hole running front to back, the hole's centre 9.7 cm up |
| Castle nut | on the arm, round the stud | Zinc-plated 3/8-24 slotted hex nut: 9/16 in (14.3 mm) across flats, flats facing front and back, 8.3 mm thick, six slots 3.2 mm wide and 3.2 mm deep; one slot runs front to back over the cross-hole |
| Cotter pin | in the block's groove, prongs pointing at the robot | New standard 3/32 x 1 in low-carbon steel extended-prong cotter pin (ASME B18.8.1): half-round prongs 1.1 x 2.2 mm lying together, 25.4 mm from under the eye to the short prong's end, the other prong 4 mm longer, eye 6 mm across, about 1.4 g |
| Block | x 24.6 to 30.6, centred on y -18.0 | Aluminium, 6 x 2 x 2 cm, a groove 3.0 mm wide and 1.0 mm deep along its top; the pin's eye and 8 mm of shank stick out past its far end (the eye's centre at (31.7, -18.0)) |
| Screwdriver | standing tip down in the rack | Precision flat-head, 120 mm long, 30 g: a steel blade 2.5 x 0.4 mm for its first 8 mm, widening evenly to 4 x 3 mm 40 mm up, then a red plastic handle 18 mm square and 80 mm long |
| Rack | centred at (22.0, 18.0) | Grey plastic block 3 x 3 cm, 2.8 cm tall, with a 4 mm square hole |

Sources: workers, BLS OOH, *Automotive Service Technicians and Mechanics*
(bls.gov/ooh/installation-maintenance-and-repair/automotive-service-technicians-and-mechanics.htm); task line, O*NET
49-3023.00 (onetonline.org/link/details/49-3023.00); practice, Meritor bulletin (static.nhtsa.gov/odi/tsbs/2015/
MC-10110472-9999.pdf) and Suzuki RA410 service manual; pin sizes and material (low-carbon steel 1005-1010), ASME
B18.8.1 (table at fastenersuperstore.com/product_specs/pins-cotter); the prongs' plastic moment, 118 N mm, is that of a
half-round 1.1 x 2.2 mm section at the 250 MPa yield of 1005-1010 steel.

## Run a trial

From this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lock the castle nut with the cotter pin: push it through the nut's slot and the stud's hole, then spread its prongs apart." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cotter-dream
```

`task_score` prints the trial's score: 1.0 on success (held for 1 s), otherwise `score()` at the end of the episode.
A trial takes about 3 minutes on one RTX 4080 (scene start-up included). Seed 0; the scene has no random placement
(`-E object_jitter=0`).

Offline check (no GPU): `python task.py` runs the scoring rules on 16 hand-built states and the bending rule on 3.

## Demo

The demo video is a scripted demo (both arms at once). The right arm picks the pin by its eye where it sticks out of
the block, carries it round to the nut's far side, slides it along the slot through the stud's hole and pushes the eye
home with its closed fingertips; it keeps pressing it there while the left arm works. The left arm takes the
screwdriver out of its rack and holds it tip down, leaning back 35 degrees. It puts the blade down beside the end of
the longer prong, where it runs on past the shorter one, and pushes that end out; then it puts the blade into the gap
that opens and pushes the longer prong round about its root, and pushes the shorter one round the other way. The
prongs end 46 and 47 degrees out (93 degrees apart), the eye home; the right arm lets go, and the trial reaches 1.0 at
about 68 s (about 1,350 steps). Six runs of the demo all scored 1.0.

A wrong strategy, pushing the pin through and home and leaving its prongs straight, scores 0.25 (through, the prongs
0 degrees apart).

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`). Tools for a real trial: a phone camera held
straight above the nut, a protractor and a 1 mm feeler gauge.

- **Through:** the pin passes through the stud's cross-hole along the slot (its axis within 15 degrees of the slot's,
  within 1.5 mm of the hole's axis), the prongs' roots out past the stud.
- **Prong angle:** in the photo, the angle between the pin's axis and a line from a prong's root (where it leaves the
  nut) to its tip, counted outwards. The prongs are that far apart in all: the sum of the two. A prong bent inwards
  counts negative. In simulation: each prong's hinge angle.
- **Home:** the feeler gauge does not go in between the pin's eye and the nut. In simulation: the eye within 1 mm of
  the nut.
- **Released:** the robot does not touch the pin; the screwdriver's tip is more than 2 cm from the prongs' roots.

| Outcome | Score |
|---|---|
| The pin not through the cross-hole | 0 |
| Through, the prongs less than 45 degrees apart | 0.25 |
| At least 45 but less than 90 degrees apart, or one prong less than 30 degrees out | 0.5 |
| At least 90 degrees apart, each prong at least 30 degrees out, but the eye not home, the robot touching the pin, or the screwdriver's tip within 2 cm of the prongs' roots | 0.75 |
| At least 90 degrees apart, each prong at least 30 degrees out, the eye home, the robot off the pin and the screwdriver away, held for 1 s | 1.0 |

Spread 90 degrees apart, the prongs' tips stand well over 3 mm apart, wider than the slot and the hole, so the pin
can't be pulled back out; each prong at least 30 degrees out asks for both to be spread, not one.

## Reset

A spread cotter pin is not reused: each trial takes a new straight pin, laid in the block's groove as in the setup.
Stand the screwdriver back in its rack.

## Real trials

Bending a prong takes about 12 N, 10 mm from its root, within the arms' strength (about 40 N per finger). The
screwdriver is a precision one with a blunt 2.5 mm edge; nothing is hot, powered or under load. The stand is clamped
to the table.
