"""Sizes and positions shared by the task file and the scene generator (work/unplug-pv-connector/make_site.py).

Station frame: metres, x forward from the line through the arm bases, y to the robot's left, z up from the table top.
Every size is from a public source named here; "est." marks an estimate measured off a published drawing or photo.

Sources
- MC4 connector: Staubli "Original MC4" datasheet 11014112 (dimensional drawings, p. 3): socket PV-KBT4 ~60 mm long,
  plug PV-KST4 ~62 mm long, both 18.8 mm across; "In accordance with NEC 2023, requires a tool to open"; insulation
  PC/PA. Section lengths along each part are measured off that drawing (est.).
- Unlocking tool: Staubli PV-MS open-end spanner set 32.6024 (PV tools catalogue 11014090 p. 9; MA270 operating
  instructions: "push the unlocking pins of PV-MS-PLS/1 onto the locking clips of the socket and separate the
  connection"; "PV connections must not be disconnected while under load"). One spanner 120 x 40 x 4 mm, the set
  50 g (BayWa r.e. and Krannich listings). Pin and jaw sizes measured off the MA270 product image (est.).
- DC isolator: IMO SI16-PEL64R-2 enclosed rotary isolator (IMO solar isolator brochure, dimensions p. 28: 180 x 98 mm,
  76 mm deep plus a 31 mm handle; datasheet: 0.43 kg, 3-position handle ON - OFF - LOCK-OFF). ON with the handle bar
  upright, OFF a quarter turn anticlockwise (IMO datasheet photographs).
- PV module: Canadian Solar CS6R-400MS-HL (datasheet v1.1C25: 1722 x 1134 x 35 mm, 22.4 kg, frame cross-section
  35 x 35 mm, 4 mm2 cable, 1100 / 1550 mm leads, MC4 connectors).
- Strut channel: Unistrut P1000, 1-5/8 x 1-5/8 in (41.3 mm), the usual ground-mount purlin and post.
"""

import math

MM = 0.001
IN = 0.0254

# -- the MC4 pair hangs in the air along x, pointing at the robot: the module's plug (PV-KST4) at +x, its lead
# cable-tied to the rack post 4 cm behind it; the home-run socket (PV-KBT4) at -x
D = 18.8 * MM  # both parts' diameter (datasheet)
AXIS_Y = 0.0
AXIS_Z = 0.110
X0 = 0.355  # the plug's front face (the mating line)
PLUG_L = 62 * MM  # PV-KST4, front face to the back of its cap nut
PLUG_FRONT_L = 37.8 * MM  # its flat-sided front sleeve (est.)
PLUG_CAP_L = 18.7 * MM  # its cap nut (est.)
WINDOW = (13.6 * MM, 25.0 * MM)  # the plug's two latch windows (+-y), from its front face (est.)
WINDOW_HALF_IN = 7.0 * MM  # the sleeve's half-width inside, where the spanner's pins go in (est.)
SOCKET_L = 60 * MM  # PV-KBT4, tip of its contact tube to the back of its cap nut
SOCKET_IN = 26 * MM  # how far its front goes into the plug when mated (est.)
SOCKET_TIP_X = X0 + SOCKET_IN
SOCKET_REAR_X = SOCKET_TIP_X - SOCKET_L  # 0.321
SOCKET_TUBE = (8.6 * MM, 14.0 * MM)  # contact tube: diameter, length from the tip (est.)
SOCKET_CLIPS = (12.0 * MM, 22.6 * MM)  # the two locking clips, from the tip (est.)
SOCKET_BODY_END = 36.0 * MM  # the body ends and the thread starts, from the tip (est.)
SOCKET_CAP = (42.0 * MM, SOCKET_L)  # its cap nut, from the tip (est.)
SOCKET_MASS = 0.035  # the socket with 12 cm of 4 mm2 cable (est.)
APART = 20 * MM  # full credit: the halves this far apart
ARC = 3 * MM  # parted this far = contacts separated (an arc if the string is live)
RACK_POST_X = 0.4757  # the rack post the module's lead is tied to (Unistrut, front face at 0.455)

# -- the PV-MS spanner (MA270 ill. 1-3: used square to the connector like an open-end spanner, its fork round the
# plug's front, the two unlocking pins on the fork's prongs pointing along the connector into the plug's windows).
# Frame: x along it (the fork at -x, the cap-nut jaw at +x), pins on its -z face.
TOOL = (120 * MM, 40 * MM, 4 * MM)  # length, width, thickness (listing)
TOOL_MASS = 0.025  # half the two-piece set's 50 g
FORK = (18.5 * MM, 12.0 * MM)  # the unlocking fork: slot width, depth from the end (est.)
PIN = (3.0 * MM, 3.0 * MM, 10.0 * MM)  # each unlocking pin, at a prong's tip (est.)
PIN_IN = 8.5 * MM  # each pin's inner face from the fork's centre line (est.)
PIN_Y = PIN_IN + PIN[1] / 2
PIN_X = -(TOOL[0] / 2 - PIN[0] / 2 - 0.5 * MM)  # the pins' centre line along the spanner
NUT_JAW = (15.5 * MM, 14.0 * MM)  # the open-end jaw for the cap nut at +x: width, depth (est.)
# lying across a Unistrut on the table, fork end towards the robot, pins down in front of it
STRUT = 41.3 * MM
STRUT_X = 0.240
STRUT_Y = (0.060, 0.300)
STRUT_TOP = STRUT
TOOL_POS = (STRUT_X, 0.175, STRUT_TOP + TOOL[2] / 2 + 0.3 * MM)


def pin_tips_local():
    """The two pins' tips in the spanner's frame."""
    return [(PIN_X, s * PIN_Y, -TOOL[2] / 2 - PIN[2]) for s in (-1, 1)]


# -- the isolator: IMO SI16-PEL64R-2 on a Unistrut post, its face towards the robot ----------------------------------
ISO = (76 * MM, 98 * MM, 180 * MM)  # depth (x), width (y), height (z)
ISO_Y = -0.280
ISO_FACE_X = 0.372
ISO_BOTTOM = 0.120
HANDLE_Z = ISO_BOTTOM + ISO[2] / 2
HANDLE_H = 31 * MM  # the handle stands this far proud of the face (brochure)
ESCUTCHEON_D = 64 * MM  # "PEL64" (est. from the drawing)
KNOB_D = 46 * MM  # the handle's round base (est.)
BAR = (56 * MM, 14 * MM)  # the bar across it: length, width (est.)
OFF_DEG = -90.0  # the handle's joint angle at OFF: a quarter turn anticlockwise seen from the front
OFF_TOL_DEG = 15.0
POST_X = ISO_FACE_X + ISO[0] + STRUT / 2

# -- the module: CS6R-400MS-HL in landscape on a ground-mount rack behind, tilted 20 degrees --------------------------
MODULE = (1134 * MM, 1722 * MM, 35 * MM)  # up the slope, across (y), thick
MODULE_TILT = math.radians(20)
MODULE_LOW_X = 0.460  # its lower edge
MODULE_LOW_Z = 0.300
