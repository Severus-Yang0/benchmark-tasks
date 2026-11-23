"""Sizes of the three-section shock-corded tent pole (metres, kilograms, newtons). Pure Python.

The sections are DAC Featherlite NSL 9 mm aluminium pole sections, 40 cm long (extremtextil.de articles 71304
"endpiece w. insert", 71301 "center piece" and 71307 "endpiece with insert entry"; Shelby Outdoor's listing of the same
center piece): tube 9.0 mm outside, 7.8 mm inside; the socket end widened to 9.5 mm outside, 8.5 mm inside; the
insert (ferrule) 8.3 mm outside, 6.3 mm inside, standing 30 mm out of the tube's end ("400 mm + 30 mm insert tube").
Weights: 19.5 g (endpiece with insert), 19.8 g (center piece).

The shock cord is 3 mm polyester-covered shock cord (Marlow Ropes datasheet MA-DA-014: 7 to 11 N at 100 % elongation),
cut to 70 % of the pole's length as Hilleberg's pole instructions say ("cut a new length of shock cord that is 70% the
length of your pole").

Section frame: z along the section, from its first end (the outer end of section 1, the socket mouth of sections 2 and
3) to its second end (the insert of sections 1 and 2, the outer end of section 3); the origin at the first end's face,
on the axis.
"""

from __future__ import annotations

L = 0.400  # tube length of each section (without the insert)
INS_OUT = 0.030  # the insert stands this far out of the tube's second end
INS_IN = 0.018  # ... and is this far inside it (48 mm insert)
SOCK = 0.035  # the widened socket end's length

R_TUBE = 0.0045  # 9.0 mm tube
R_TUBE_IN = 0.0039  # 7.8 mm bore
R_SOCK = 0.00475  # 9.5 mm socket end
R_BORE = 0.00425  # 8.5 mm socket bore
R_INS = 0.00415  # 8.3 mm insert
R_INS_IN = 0.00315  # 6.3 mm insert bore

N_SECTIONS = 3
POLE_LEN = N_SECTIONS * L  # 1.20 m assembled
HAS_INSERT = (True, True, False)  # section 1: endpiece with insert; 2: center piece; 3: endpiece with insert entry
HAS_SOCKET = (False, True, True)

# masses: the listed weights (sections 1 and 2); section 3, the center piece's 19.8 g less its insert's 3.1 g
RHO_AL = 2810.0  # kg/m^3, 7001 aluminium
_INSERT_MASS = RHO_AL * 3.141592653589793 * (R_INS ** 2 - R_INS_IN ** 2) * (INS_OUT + INS_IN)
MASS = (0.0195, 0.0198, 0.0198 - _INSERT_MASS)


def com(k: int) -> float:
    """Section k's (0, 1, 2) centre of mass along its z: a uniform tube with the insert's mass at its middle."""
    m_ins = _INSERT_MASS if HAS_INSERT[k] else 0.0
    m_tube = MASS[k] - m_ins
    z_ins = L - INS_IN + (INS_OUT + INS_IN) / 2
    return (m_tube * L / 2 + m_ins * z_ins) / MASS[k]


# shock cord
CORD_F100 = 9.0  # N at 100 % elongation: the middle of Marlow's 7-11 N for 3 mm cord
CORD_REST = 0.70 * POLE_LEN  # 0.84 m
CORD_K = CORD_F100 / CORD_REST  # N/m of the whole cord's stretch
CORD_DAMP = 0.3  # N s/m: the rubber's internal damping
THROUGH = tuple(L + (INS_OUT if HAS_INSERT[k] else 0.0) for k in range(N_SECTIONS))  # the cord's run in each section

# friction (anodised aluminium)
MU_AL = (0.4, 0.3)  # static, dynamic


def cord_tension(path: float, rate: float = 0.0) -> float:
    """The cord's tension (N) when its path through the pole is ``path`` m long, stretching at ``rate`` m/s."""
    stretch = path - CORD_REST
    if stretch <= 0.0:
        return 0.0
    return max(0.0, CORD_K * stretch + CORD_DAMP * rate)
