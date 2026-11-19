# Pick a ripe strawberry off a potted plant without bruising it or tearing its stalk, and put it in a basket

A strawberry plant grows in a low pot. One fruit truss arches out of it over the rim, and a ripe berry hangs from it.
The robot must pick the berry and put it in a berry basket beside the pot, with one arm.

Picking strawberries is familiar from gardens and pick-your-own farms, and the hard part is physical:

- A berry pulled straight along its stalk needs **13.94 N** to come off; bent against its stalk and pulled, **3.17 N**
  (Dimeas et al. 2013, "Towards designing a robot gripper for efficient strawberry harvesting", RAAD, as quoted in
  Rajendran et al. 2022, arXiv 2207.12552).
- Pulled straight, the stalk sometimes breaks midway and a piece stays on the berry; tilted 90 degrees, the berry
  separates cleanly at its abscission layer with little force (Yamamoto et al. 2014, JARQ 48(3):261-269). A stalk
  piece left on a berry punctures the others in the basket.
- The berry bruises if gripped harder than about **10 N** (Dimeas et al., same quote).

A light grip slips on a straight pull and a hard one bruises the berry, so a clean pick needs the berry tilted against
its stalk before it is pulled, while the flexible truss gives way under the pull.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the whole-trial grip and detachment watch, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `stalkkit.py` | The fruit truss and the berry's stalk: one chain of bending links anchored at the crown, laid along a curve at rest, with a spring weld to the berry that the task releases |
| `touch.py` | Robot-to-berry contacts from PhysX: "released" (no part of the robot touches it) and the grip force |
| `scenekit.py` | USD bodies for yam_arena task files (hooks yam_arena without changing it) |
| `assets/berry.usdc` (553 KB) + `berry_skin.jpg` (97 KB) | The berry: one rigid body, 15 g, 28 mm across and 36 mm long, collided as the convex hull of its mesh. Mesh and texture from the YCB object set's `012_strawberry` (Google 16k scan; Calli et al. 2015, ycbbenchmarks.com, CC BY 4.0), turned calyx up, scaled and the texture resized |
| `assets/plant.usdc` (207 KB) | One fixed body: the pot (collided as the convex hull of its outside), soil, and the strawberry plant's leaves (seven trifoliate leaves and a flower; not collided) |
| `assets/basket.usdc` (14 KB) | The berry basket, fixed: tapered walls with slots, collided as its floor and four tilted walls |

`berry.usdc` comes from `work/strawberry/make_berry.py`, `plant.usdc` and `basket.usdc` from
`work/strawberry/make_plant.py`.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Pot and plant | pot centred (38.0, -10.0), fixed | Black plastic 6 in azalea pot (15.2 cm across the top, 11.4 cm at the bottom, 11.4 cm tall), soil 0.4 cm below the rim, holding a day-neutral strawberry plant (e.g. 'Albion'). Leaves, flowers and any green fruit on the far half; no other ripe fruit |
| Fruit truss | from the crown, over the near rim on the pot's centre line (y -10.0) | About 2 mm thick; rises among the leaf stalks to about 15 cm above the table, arches towards the robot over the rim about 1 cm above it and hangs down beyond it; the berry's own stalk hangs nearly straight down to the calyx |
| Strawberry | centre (26.5, -10.0), 5.9 cm high, 3.5 cm beyond the rim | One ripe berry (fully red, 14-16 g, about 2.8 cm across and 3.6 cm long), tip down. Turn and place the pot so it hangs there |
| Basket | centred (23.0, -25.0), fixed | Square green molded-fiber quart berry basket (Pactiv 1 qt: 14.0 cm square at the top, 10.8 cm at the bottom, 8.3 cm tall), empty |

Out of season, when no plant carries a ripe berry, a long-stem strawberry (sold year round for chocolate dipping,
stalk about 5 cm) stands in: its stalk is joined to the end of a fruit truss with a 2.5 mm silicone grafting clip, so
the berry hangs at the same place. The berry still comes off at its own abscission layer.

Sources: berry mass, the mean of harvested cv. Beni hoppe fruit (14.8 g, Yamamoto et al. 2014); stalk diameter
1.78 mm (Dimeas et al.); pot, a standard 6 in azalea pot (three-quarters as tall as it is wide); basket, Pactiv's 1 qt
green molded fiber basket (5.5 x 5.5 x 3.25 in, 4.25 in at the bottom).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick the strawberry and put it in the basket." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/strawberry-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. A trial takes
about 2 minutes on one GPU (scene start-up included). Seed 0; the scene has no random placement
(`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules on 15 hand-built states, checks the detachment rule
(13.94 N straight, 3.17 N square, the pull's angle from the spring's stretch) and replays the demo's plan through the
arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Pick the strawberry and put it in the basket." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strawberry-demo
```

The right arm comes down round the berry with its fingers 60 mm apart (the stalk between them, the fingers closing
along y), pads 5 mm above the berry's centre on its wide shoulders, and closes them 6 mm narrower than the berry
(about 8 N a finger). Then it draws the berry 60 mm towards the robot and 20 mm up: the truss bends and the stalk
pulls across the berry's axis, and it comes off at about 3.3 N, 81 degrees to its axis. The arm carries it over the
basket, lowers it to 3 cm above the floor, opens and goes back; the empty truss springs back. It reaches 1.0 in about
12.5 s (250 steps). A straight pull (the same grip, then 40 mm straight down) only bends the truss down: the pull
reaches about 3 N at 25 degrees to the berry's axis (it needs 5.8 N there), the berry slips in the grip and stays on
(0).

## How the berry comes off (simulation)

The fruit truss and the berry's stalk are one chain of 17 links, 1 cm each, anchored at the crown and laid at rest
along the truss's curve. Each bend has a spring back to that curve (1.5 N·m/rad), so the truss droops about 1 cm
under the berry and bends when the berry is pulled. The berry hangs from the chain's end on a stiff six-way spring
(3000 N/m). The first second of a trial lets the truss take the berry's weight; from then on, every control step, the
spring's stretch gives the stalk's pull on the berry and the pull's angle `a` to the berry's axis (calyx up). When the
pull reaches `13.94 / (1 + 3.40 sin a)` N (13.94 N straight, 4.10 N at 45 degrees, 3.17 N square), the spring is
switched off and the berry is free. The two ends are the measured values; the smooth curve between them, falling as
the tilt grows (as Yamamoto et al. observed), is a modelling choice. So is "torn": pulled off within 45 degrees of
straight, the stalk is taken to have broken midway. On a real trial both are simply observed.

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`), after a whole-trial rule. Tools for a real trial:
the trial video, a close-up photo of the berry taken right after the trial, and a ruler.

- **Squeezed:** at any moment a robot finger pressed the berry with more than 10 N. In simulation, the PhysX contact
  force of each robot link on the berry, checked every control step. On a real trial, a dented, flattened or leaking
  patch 3 mm or more across where a finger pressed, in the photo.
- **Off:** the berry no longer hangs from the plant.
- **Torn:** a piece of stalk more than 10 mm long remains above the calyx (simulation: it came off with the pull within
  45 degrees of its axis).
- **In the basket:** the berry's centre inside the basket's walls (by at least half its radius) and the berry resting
  on the basket's floor, any way up (simulation: its centre no more than 22 mm above the floor, and still).
- **Released:** no part of the robot touches the berry (simulation: PhysX contacts within 1 mm).

| Outcome | Score |
|---|---|
| Squeezed with more than 10 N at any time | 0 |
| Still on the plant | 0 |
| Off but not in the basket (held, on the table or on the floor) | 0.5 |
| Off and torn, wherever it lies | 0.5 |
| Off, not torn, in the basket, still touched | 0.75 |
| Off, not torn, in the basket and released, held for 1 s | 1.0 |

## Reset

One berry is used per trial. Turn the pot so the truss of the plant's next ripe berry arches over the near rim and the
berry hangs as in the setup (out of season, clip a fresh long-stem berry to a truss end) and empty the basket. Use
berries that are fully red and firm.

## Real trials

The berry weighs 15 g; the pulls involved are 3-14 N and the grip stays under 10 N, all well within the arms'
strength (about 40 N per finger). Nothing is sharp, hot or powered.
