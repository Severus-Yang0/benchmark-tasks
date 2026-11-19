# Sow a bean seed one inch deep in a pot of loose potting mix and leave it covered

A small square nursery pot stands on the table, filled with loose potting mix. One dry red kidney bean lies beside
it. The robot must sow the bean in the middle of the pot, one inch (2.5 cm) deep, and leave it covered with soil.

Sowing seeds in a pot is one of the most common jobs in home and school gardening, and seed packets give a depth:
bean seeds go one inch deep (University of Illinois Extension, *Snap Beans*: "Plant seeds of all varieties one inch
deep."). The hard part is physical: the bean is small (16.5 x 8.5 x 5 mm) and smooth and the mix gives way under it.
Pushed down with a fingertip, the bean slips out sideways unless the push stays square on it; a hole dug first
fills back in as soon as the fingers leave it, because loose mix holds no wall. The bean has to go in while the hole
is open, or be pushed down, and soil has to end up over it.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the arms-clear watch, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `soilkit.py` | The potting mix: thousands of small rigid spheres authored straight into USD, read back through one PhysX view every control step, put back at every reset |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `touch.py` | Robot-to-body contacts from PhysX (helper shared with other tasks; not used by the score) |
| `assets/seed.usdc` (8 KB) | The bean: one rigid body, 0.6 g, 16.5 x 8.5 x 5.0 mm, a bent ellipsoid collided as its convex hull |
| `assets/soil_start.npz` (276 KB) | The mix's starting layout: 8,708 grain centres and radii, settled and levelled 10 mm below the rim |

`seed.usdc` comes from `work/sow-seed/make_seed.py`; `soil_start.npz` from `work/sow-seed/prepare_soil.sh` (a loose
lattice of grains settles in the pot, grains above the level are struck off, the rest settles again). No third-party
assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Pot | centred (36.0, -12.0), taped to the table | Black plastic square nursery pot, 3.5 in (8.9 cm) square at the top and 3.5 in (8.9 cm) tall, walls about 1.5 mm thick (as Greenhouse Megastore's 3.5" square form pot) |
| Potting mix | in the pot | Loose peat-based potting mix (bulk density about 0.15 g/cm3; about 0.57 l, 80 g), poured in loosely and levelled with a flat hand 1.0 cm below the rim, not pressed down |
| Bean | (30.0, -22.0), lying flat on the bare table, its length pointing forward | One dry red kidney bean, 16.5 x 8.5 x 5.0 mm, 0.6 g |

The left arm has nothing to do.

Sources: sowing depth, University of Illinois Extension, *Snap Beans* (extension.illinois.edu/gardening/snap-beans);
bean size, red kidney bean at 10-12 % moisture (d.b.), Aviara and Ma'aji 2023, "Effect of moisture content on some
physical properties of red kidney bean (*Phaseolus vulgaris* L.) seed grown in North Eastern Nigeria", AZOJETE
19(1):129-142, Table 1; bean mass, 62 g per 100 seeds for a red kidney accession (USDA GRIN, PI 527771); bulk density
of peat-based mixes 0.09-0.16 g/cm3 (Greenhouse Management Online, Unit 7, University of Arkansas); pot,
Greenhouse Megastore 3.5" square form pot (3.5 x 3.5 x 3.5 in).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sow the bean in the middle of the pot, one inch deep, and leave it covered with soil." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/sow-seed-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. A trial takes
about 4 minutes on one RTX 4080 (scene start-up included; the 8,708 grains run at about a third of real time). Seed 0;
the scene has no random placement (`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules on 13 hand-built states and the arms-clear rule on 5,
and replays the demo's plan through the arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Sow the bean in the middle of the pot, one inch deep, and leave it covered with soil." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sow-seed-demo
```

The right arm picks the bean by its middle (fingers closing across its width), carries it over the pot's centre and
lays it flat on the soil. It closes its fingers, comes down on the bean with their tips and pushes it straight down,
the fingertips 30 mm below the soil's level, holds, and pulls them out: the loose mix falls back into the hole over
the bean. The bean ends 28 mm deep (its top below the soil's starting level), 9 mm from the pot's centre, covered,
and the trial reaches 1.0 at about 17.5 s (350 steps), once the fingertips are out of the pot.

A wrong strategy, laying the bean on the soil and leaving it there, scores 0.25 (in the pot, not covered).

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`). Tools for a real trial: a phone camera, a ruler
with a straight edge, a teaspoon and a kitchen scale.

- **In the pot:** the bean is inside the pot's walls, below its rim.
- **Spilled:** soil outside the pot (on the table or floor, swept up and weighed) is more than 10 % of the mix's
  starting weight. In simulation: the grains outside the pot's walls, by mass.
- **Covered:** no part of the bean is visible in a photo taken from straight above the pot. In simulation: over each
  point of a 5 x 3 grid across the bean's upper side lies a grain.
- **Depth:** how far the bean's top is below the soil's starting level (1.0 cm below the rim). On a real trial, after
  the covered photo: lay the ruler's edge across the rim, scrape the soil off the bean from above with the teaspoon
  until its top shows, measure from the edge down to the bean's top and subtract 1.0 cm.
- **Middle:** the bean's centre is within 2.5 cm of the pot's centre (measured from two walls).
- **Arms clear:** no fingertip is over the pot's opening lower than 1 cm above its rim (photo from the side). In
  simulation, the fingertips are 88 mm below the finger links' origins, midway between them.

| Outcome | Score |
|---|---|
| Bean not in the pot, or more than 10 % of the soil spilled | 0 |
| In the pot, not covered | 0.25 |
| Covered, but its top less than 1.5 cm or more than 3.5 cm deep, or more than 2.5 cm from the centre | 0.5 |
| Covered, 1.5-3.5 cm deep, in the middle, a fingertip still in the pot | 0.75 |
| Covered, 1.5-3.5 cm deep, in the middle, arms clear, held for 1 s | 1.0 |

## Reset

Tip the pot's soil back into its bag, refill the pot loosely and level it with a flat hand 1.0 cm below the rim
without pressing it down, and put a fresh dry bean on the table as in the setup. Sweep up any spilled soil.

## Real trials

The bean weighs 0.6 g and the mix is loose: pushing the bean in takes a few newtons at most, well within the arms'
strength (about 40 N per finger). Nothing is sharp, hot or powered. The pot is taped down so a push against its wall
cannot tip it.
