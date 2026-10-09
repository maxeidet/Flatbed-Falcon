#!/usr/bin/env python3
"""Genererar worlds/hive_base.sdf: ett militärt tältläger i en svensk barrskog.

Träden är meshen från Fuel-modellerna "Pine Tree" och "Oak tree" (samma trädfamilj
som Baylands), lagrade i worlds/trees/. Allt annat är SDF-primitiver.
Kräver Gazebo Harmonic (sdformat 14).

Kör från repots rot:  python3 worlds/generate_hive_base.py
Slumpen är seedad, så samma skript ger alltid samma värld.
"""

import math
import random
from pathlib import Path

OUT = Path(__file__).with_name("hive_base.sdf")
rng = random.Random(206)

# ----------------------------------------------------------------------------
# Färger (RGBA)
# ----------------------------------------------------------------------------
MOSS = (0.23, 0.30, 0.15, 1)
SPRUCE = (0.10, 0.20, 0.10, 1)
SPRUCE_DARK = (0.07, 0.15, 0.08, 1)
PINE_BARK = (0.55, 0.33, 0.20, 1)
PINE_CROWN = (0.16, 0.27, 0.13, 1)
SPRUCE_BARK = (0.30, 0.22, 0.16, 1)
BIRCH_BARK = (0.88, 0.87, 0.82, 1)
BIRCH_LEAF = (0.35, 0.52, 0.18, 1)
GRANITE = (0.45, 0.44, 0.43, 1)
GRANITE_LICHEN = (0.55, 0.56, 0.50, 1)
BLUEBERRY = (0.18, 0.32, 0.14, 1)
DIRT = (0.40, 0.32, 0.22, 1)
GRAVEL = (0.50, 0.45, 0.38, 1)
RUT = (0.30, 0.24, 0.17, 1)
TENT = (0.27, 0.31, 0.20, 1)
TENT_DARK = (0.20, 0.23, 0.15, 1)
FM_GREEN = (0.24, 0.30, 0.20, 1)   # svensk försvarsmaktsgrön
FM_DARK = (0.12, 0.15, 0.10, 1)
FM_BROWN = (0.33, 0.27, 0.18, 1)
FM_BLACK = (0.08, 0.08, 0.07, 1)
RUBBER = (0.05, 0.05, 0.05, 1)
STEEL = (0.25, 0.25, 0.25, 1)
GLASS = (0.10, 0.14, 0.17, 1)
CANVAS = (0.30, 0.33, 0.22, 1)
WOOD = (0.50, 0.36, 0.22, 1)
ASH = (0.20, 0.19, 0.18, 1)
WHITE = (0.9, 0.9, 0.9, 1)


def f(x):
    return f"{x:.3f}".rstrip("0").rstrip(".") if isinstance(x, float) else str(x)


def pose(*p):
    return " ".join(f(float(v)) for v in p)


def material(c):
    rgba = " ".join(f(float(v)) for v in c)
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.05 0.05 0.05 1</specular></material>"


def geom(kind, *dims):
    if kind == "box":
        return f"<box><size>{pose(*dims)}</size></box>"
    if kind == "cyl":
        return f"<cylinder><radius>{f(float(dims[0]))}</radius><length>{f(float(dims[1]))}</length></cylinder>"
    if kind == "cone":
        return f"<cone><radius>{f(float(dims[0]))}</radius><length>{f(float(dims[1]))}</length></cone>"
    if kind == "sphere":
        return f"<sphere><radius>{f(float(dims[0]))}</radius></sphere>"
    if kind == "ell":
        return f"<ellipsoid><radii>{pose(*dims)}</radii></ellipsoid>"
    if kind == "cap":
        return f"<capsule><radius>{f(float(dims[0]))}</radius><length>{f(float(dims[1]))}</length></capsule>"
    raise ValueError(kind)


class Model:
    """Statisk modell med en länk; delar läggs till som visual (+ ev. collision)."""

    def __init__(self, name, p=(0, 0, 0, 0, 0, 0)):
        self.name, self.pose, self.parts, self.n = name, p, [], 0

    def add(self, kind, dims, p, color, collide=False, shadows=True):
        self.n += 1
        g = geom(kind, *dims)
        # DART saknar kon-kollision; en smalare cylinder räcker gott
        cg = geom("cyl", dims[0] * 0.6, dims[1]) if kind == "cone" else g
        cs = "" if shadows else "<cast_shadows>false</cast_shadows>"
        self.parts.append(
            f'<visual name="v{self.n}"><pose>{pose(*p)}</pose><geometry>{g}</geometry>{material(color)}{cs}</visual>'
        )
        if collide:
            self.parts.append(
                f'<collision name="c{self.n}"><pose>{pose(*p)}</pose><geometry>{cg}</geometry></collision>'
            )

    def sdf(self):
        body = "\n        ".join(self.parts)
        return (
            f'    <model name="{self.name}">\n'
            f"      <static>true</static>\n"
            f"      <pose>{pose(*self.pose)}</pose>\n"
            f'      <link name="link">\n        {body}\n      </link>\n'
            f"    </model>\n"
        )


models = []

# ----------------------------------------------------------------------------
# Layout
# ----------------------------------------------------------------------------
# Lägret ligger inne i tät skog kring origo, där drönaren spawnar.
# Träd växer ända fram till och mellan tälten; bara objekten själva har fri yta.
# Grusvägen är en slinga runt lägret (se ROAD_CORNERS), och en infart leder
# norrut från södra raksträckan in till fordonsplatsen.
WORLD_R = 150.0
TREE_TARGET = 1300
MIN_TREE_D = 4.8
SPAWN_FREE_R = 12.0

TENTS = [(-14, 10, 0.3), (-4, 17, -0.2), (7, 16, 0.5), (-18, -2, 1.2), (17, 6, 2.4)]
# Fria cirklar (x, y, radie) kring lägrets objekt
# Origo hålls fri för drönarens start
CAMP_ZONES = [(0, 0, 7.0)] + [(tx, ty, 4.2) for tx, ty, _ in TENTS] + [
    (-3, 6, 3.0),     # eldstad
    (-9, -5.5, 4.5),  # stabsbord under kamouflagenät
    (-10, 18.5, 1.5), # vedtrave
    (-24, -12, 3.0),  # flyttblock
]
VEH_AREA = (5.0, 23.0, -21.0, -8.0)  # xmin, xmax, ymin, ymax


# Grusvägen är en sluten slinga runt lägret som lastbilen kör varv på.
# Den definieras som hörnpunkter (x, y, radie): mellan hörnen går raka sträckor
# och varje hörn rundas av med en cirkelbåge. Raksträckorna har alltså exakt
# noll krökning; de långa blir landningszoner för drönaren.
# Ordningen är moturs, så positiv krökning = vänstersväng.
ROAD_CORNERS = [
    (-100.0, -80.0, 17.0),   # serpentinens västra hårnål ...
    (-100.0, -115.0, 17.0),  # ... (två 90°-hörn = 180° vändning)
    (90.0, -115.0, 25.0),    # sydost (södra raksträckan = landningszon)
    (125.0, -62.0, 20.0),
    (118.0, 10.0, 20.0),     # östra kurvan
    (80.0, 35.0, 15.0),      # S-kurva in mot lägret ...
    (112.0, 72.0, 18.0),     # ... och ut igen
    (50.0, 100.0, 25.0),     # nordost
    (-20.0, 75.0, 20.0),     # svacka norr om lägret
    (-60.0, 105.0, 15.0),    # snäv krön-kurva
    (-110.0, 90.0, 20.0),    # nordväst
    (-118.0, -15.0, 20.0),   # västra sidan
    (-65.0, -45.0, 15.0),    # inre raksträckan förbi infarten (landningszon)
    (65.0, -45.0, 17.0),     # serpentinens östra hårnål ...
    (65.0, -80.0, 17.0),     # ... tillbaka västerut (landningszon)
]
ROAD_W = 5.0
ACCESS_X = 12.0  # infartens x-läge
LANDING_MIN_LEN = 100.0  # raksträckor minst så här långa blir landningszoner
ROAD_CSV = Path(__file__).with_name("road_centerline.csv")


def build_road(corners):
    """Delar upp slingan i raka bitar och bågar.

    Returnerar en lista med ("line", p0, p1) och ("arc", centrum, R, a0, dvinkel).
    """
    n = len(corners)
    tangents = []
    for i, (px, py, R) in enumerate(corners):
        ax, ay, _ = corners[i - 1]
        bx, by, _ = corners[(i + 1) % n]
        a_in = math.atan2(py - ay, px - ax)
        a_out = math.atan2(by - py, bx - px)
        turn = (a_out - a_in + math.pi) % (2 * math.pi) - math.pi
        t = R * math.tan(abs(turn) / 2)
        if t > 0.5 * min(math.hypot(px - ax, py - ay), math.hypot(bx - px, by - py)):
            raise ValueError(f"Hörn {i}: radien {R} får inte plats")
        t1 = (px - t * math.cos(a_in), py - t * math.sin(a_in))
        t2 = (px + t * math.cos(a_out), py + t * math.sin(a_out))
        side = 1 if turn > 0 else -1
        c = (t1[0] - side * R * math.sin(a_in), t1[1] + side * R * math.cos(a_in))
        a0 = math.atan2(t1[1] - c[1], t1[0] - c[0])
        tangents.append((t1, t2, c, R, a0, turn))
    pieces = []
    for i in range(n):
        t1, t2, c, R, a0, turn = tangents[i]
        nxt = tangents[(i + 1) % n][0]
        pieces.append(("arc", c, R, a0, turn))
        pieces.append(("line", t2, nxt))
    return pieces


def sample_road(pieces, ds):
    """Punkter längs mittlinjen: (s, x, y, kurs, krökning, landningszon-id eller -1)."""
    out, s, zone = [], 0.0, 0
    for p in pieces:
        if p[0] == "line":
            (x0, y0), (x1, y1) = p[1], p[2]
            L = math.hypot(x1 - x0, y1 - y0)
            hdg = math.atan2(y1 - y0, x1 - x0)
            zid = zone if L >= LANDING_MIN_LEN else -1
            zone += zid >= 0
            k_n = max(1, round(L / ds))
            for k in range(k_n):
                u = k / k_n
                out.append((s + u * L, x0 + u * (x1 - x0), y0 + u * (y1 - y0), hdg, 0.0, zid))
        else:
            _, (cx, cy), R, a0, turn = p
            L = R * abs(turn)
            side = 1 if turn > 0 else -1
            k_n = max(1, round(L / ds))
            for k in range(k_n):
                a = a0 + turn * k / k_n
                hdg = (a + side * math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
                out.append((s + L * k / k_n, cx + R * math.cos(a), cy + R * math.sin(a), hdg, side / R, -1))
        s += L
    return out, s


ROAD_PIECES = build_road(ROAD_CORNERS)
ROAD_PTS, ROAD_LEN = sample_road(ROAD_PIECES, 1.0)
# Spatialt rutnät över mittlinjepunkterna så avståndsfrågan blir snabb
_RCELL = 10.0
_road_grid = {}
for _s, _x, _y, *_ in ROAD_PTS:
    _road_grid.setdefault((int(_x // _RCELL), int(_y // _RCELL)), []).append((_x, _y))


def dist_to_road(x, y):
    """Avstånd till vägens mittlinje (mättat till 30 m långt bort från vägen)."""
    gx, gy = int(x // _RCELL), int(y // _RCELL)
    d2 = 30.0 ** 2
    for ix in range(gx - 3, gx + 4):
        for iy in range(gy - 3, gy + 4):
            for px, py in _road_grid.get((ix, iy), ()):
                d2 = min(d2, (px - x) ** 2 + (py - y) ** 2)
    return math.sqrt(d2)


# En Bv 206 står parkerad på vägrenen längs norra sträckan (efter hörnet i (50, 100)),
# helt utanför körbanan så den inte står i vägen för lastbilen.
(_bx0, _by0), (_bx1, _by1) = ROAD_PIECES[2 * 7 + 1][1], ROAD_PIECES[2 * 7 + 1][2]
_bh = math.atan2(_by1 - _by0, _bx1 - _bx0)
_boff = ROAD_W / 2 + 1.5
ROADSIDE_BV = ((_bx0 + _bx1) / 2 - math.sin(_bh) * _boff, (_by0 + _by1) / 2 + math.cos(_bh) * _boff, 0, 0, 0, _bh)
CAMP_ZONES.append((ROADSIDE_BV[0], ROADSIDE_BV[1], 4.5))

# Infarten går söderut från gläntan till närmaste vägbit under den
ACCESS_Y0 = max(y for _, x, y, *_ in ROAD_PTS if abs(x - ACCESS_X) < 1.0 and y < -8)


def in_access(x, y, margin=0.0):
    return abs(x - ACCESS_X) < 3.5 + margin and ACCESS_Y0 - 2 < y < -8


# ----------------------------------------------------------------------------
# Mark
# ----------------------------------------------------------------------------
ground = Model("ground_plane")
ground.parts.append(
    "<collision name=\"collision\"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>"
    "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>"
)
ground.parts.append(
    f"<visual name=\"visual\"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>{material(MOSS)}</visual>"
)
models.append(ground)

# ----------------------------------------------------------------------------
# Grusväg (slinga av raka bitar och bågar) + infart
# ----------------------------------------------------------------------------
road = Model("dirt_road")


def road_strip(x0, y0, x1, y1, overlap):
    """En vägbit från (x0, y0) till (x1, y1) med hjulspår och grusad mittsträng."""
    L = math.hypot(x1 - x0, y1 - y0) + overlap
    yaw = math.atan2(y1 - y0, x1 - x0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    road.add("box", (L, ROAD_W, 0.04), (cx, cy, 0.02, 0, 0, yaw), DIRT, shadows=False)
    for off, col in ((-1.1, RUT), (1.1, RUT), (0.0, GRAVEL)):
        ox, oy = -math.sin(yaw) * off, math.cos(yaw) * off
        w = 0.5 if col is RUT else 0.6
        road.add("box", (L, w, 0.02), (cx + ox, cy + oy, 0.045, 0, 0, yaw), col, shadows=False)


for p in ROAD_PIECES:
    if p[0] == "line":
        road_strip(*p[1], *p[2], 0.2)
    else:
        # Korta korda-bitar i bågen; överlappet täcker kilen i ytterkanten
        _, (cx, cy), R, a0, turn = p
        k_n = max(2, math.ceil(R * abs(turn) / 2.0))
        for k in range(k_n):
            a1, a2 = a0 + turn * k / k_n, a0 + turn * (k + 1) / k_n
            road_strip(cx + R * math.cos(a1), cy + R * math.sin(a1), cx + R * math.cos(a2), cy + R * math.sin(a2), 0.6)

# Landningszoner: vit/orange stolpe på båda vägrenarna i början och slutet
LANDING_ZONES = {}
for _s, _x, _y, _h, _k, _z in ROAD_PTS:
    if _z >= 0:
        LANDING_ZONES.setdefault(_z, []).append((_x, _y, _h))
for pts in LANDING_ZONES.values():
    for x, y, h in (pts[0], pts[-1]):
        for side in (-1, 1):
            px, py = x - side * math.sin(h) * (ROAD_W / 2 + 0.6), y + side * math.cos(h) * (ROAD_W / 2 + 0.6)
            road.add("cyl", (0.06, 1.2), (px, py, 0.6, 0, 0, 0), WHITE)
            road.add("cyl", (0.065, 0.2), (px, py, 1.05, 0, 0, 0), (0.9, 0.4, 0.05, 1))

# Infart från vägen upp till gläntan
ay0 = ACCESS_Y0
alen = -6 - ay0
road.add("box", (4.0, alen, 0.04), (ACCESS_X, ay0 + alen / 2, 0.021, 0, 0, 0), DIRT, shadows=False)
for off in (-0.9, 0.9):
    road.add("box", (0.45, alen, 0.02), (ACCESS_X + off, ay0 + alen / 2, 0.046, 0, 0, 0), RUT, shadows=False)
# Uppställningsplats för fordon i gläntan (grus)
road.add("box", (16, 10, 0.04), (14, -14, 0.022, 0, 0, 0), GRAVEL, shadows=False)
models.append(road)

# ----------------------------------------------------------------------------
# Skog: tall- och lövträdsmeshen från Fuel-modellerna "Pine Tree" och "Oak tree"
# (samma trädfamilj som Baylands, CC0), lagrade i worlds/trees/. De refereras
# via /workspace där startskripten monterar repot, så inget hämtas över nätet.
# Varje 40x40 m-ruta blir en statisk modell för att hålla nere antalet entiteter.
# ----------------------------------------------------------------------------
TREES = "/workspace/worlds/trees"
# name: (mesh, grenlövstextur, barktextur, kronradie/skala, höjd/skala, stamradie/skala)
TREE_TYPES = {
    "pine": ("pine_tree.dae", "pine_branch.png", "pine_bark.png", 1.6, 5.0, 0.15),
    "oak": ("oak_tree.dae", "oak_branch.png", "oak_bark.png", 5.0, 6.5, 0.2),
}


def add_tree(m, kind, x, y, s, yaw):
    mesh, leaf, bark, crown_r, height, trunk_r = TREE_TYPES[kind]
    m.n += 1
    p = pose(x, y, 0, 0, 0, yaw)
    for sub, tex, extra in (("Branch", leaf, "<double_sided>true</double_sided>"), ("Bark", bark, "")):
        m.parts.append(
            f'<visual name="v{m.n}_{sub}"><pose>{p}</pose><geometry><mesh><uri>{TREES}/{mesh}</uri>'
            f"<submesh><name>{sub}</name></submesh><scale>{pose(s, s, s)}</scale></mesh></geometry>"
            f"<material>{extra}<diffuse>1 1 1</diffuse><pbr><metal><albedo_map>{TREES}/{tex}</albedo_map>"
            f"</metal></pbr></material><cast_shadows>false</cast_shadows></visual>"
        )
    # Förenklad kollision: stam + smal cylinder genom kronan
    h = height * s
    cr = crown_r * s * 0.35
    m.parts.append(
        f'<collision name="c{m.n}"><pose>{pose(x, y, h / 2, 0, 0, 0)}</pose>'
        f"<geometry>{geom('cyl', max(trunk_r * s, cr), h)}</geometry></collision>"
    )


def zone_clearance(x, y):
    """Avstånd från (x, y) till närmaste lägerobjekt (negativt = inuti)."""
    d = min(math.hypot(x - zx, y - zy) - zr for zx, zy, zr in CAMP_ZONES)
    # Fordonsplatsen är en rektangel
    dx = max(VEH_AREA[0] - x, 0, x - VEH_AREA[1])
    dy = max(VEH_AREA[2] - y, 0, y - VEH_AREA[3])
    return min(d, math.hypot(dx, dy))


def clear_of_road(x, y, margin):
    return dist_to_road(x, y) > ROAD_W / 2 + margin and not in_access(x, y, margin)


CELL = 40.0
forests = {}
grid = {}
placed = 0
tries = 0
while placed < TREE_TARGET and tries < 200000:
    tries += 1
    x, y = rng.uniform(-WORLD_R, WORLD_R), rng.uniform(-WORLD_R, WORLD_R)
    if math.hypot(x, y) > WORLD_R:
        continue
    # Lövträd mest längs vägen, annars tall/gran
    near_road = dist_to_road(x, y) < 12
    kind = "oak" if rng.random() < (0.3 if near_road else 0.08) else "pine"
    s = rng.uniform(1.3, 1.7) if kind == "oak" else rng.uniform(2.6, 4.0)
    crown = TREE_TYPES[kind][3] * s
    # Hela kronan hålls utanför vägkanten så drönaren har fri luftkorridor
    # ovanför lastbilen
    if not clear_of_road(x, y, 1.0 + crown):
        continue
    clear = zone_clearance(x, y)
    if clear < crown * 0.75:
        continue
    # Hela kronan hålls utanför startytan så följkameran inte hamnar i ett träd
    if math.hypot(x, y) - crown < SPAWN_FREE_R:
        continue
    gx, gy = int(x // 5), int(y // 5)
    ok = True
    for ix in range(gx - 2, gx + 3):
        for iy in range(gy - 2, gy + 3):
            for px, py in grid.get((ix, iy), ()):
                if (px - x) ** 2 + (py - y) ** 2 < MIN_TREE_D ** 2:
                    ok = False
    if not ok:
        continue
    grid.setdefault((gx, gy), []).append((x, y))
    placed += 1
    key = (int(x // CELL), int(y // CELL))
    if key not in forests:
        forests[key] = Model(f"forest_{len(forests)}")
    add_tree(forests[key], kind, x, y, s, rng.uniform(0, 2 * math.pi))
models.extend(forests.values())

# ----------------------------------------------------------------------------
# Markvegetation: blåbärsris, granitblock, fallna stockar
# ----------------------------------------------------------------------------
under = Model("undergrowth")
for i in range(600):
    r = WORLD_R * math.sqrt(rng.random())
    a = rng.uniform(0, 2 * math.pi)
    x, y = r * math.cos(a), r * math.sin(a)
    if not clear_of_road(x, y, 0.8) or zone_clearance(x, y) < 1.0:
        continue
    under.add(
        "ell",
        (rng.uniform(0.6, 1.4), rng.uniform(0.6, 1.4), rng.uniform(0.2, 0.35)),
        (x, y, 0.05, 0, 0, rng.uniform(0, 3.14)),
        BLUEBERRY,
        shadows=False,
    )
models.append(under)

rocks = Model("granite_boulders")
for i in range(60):
    r = WORLD_R * math.sqrt(rng.random())
    a = rng.uniform(0, 2 * math.pi)
    x, y = r * math.cos(a), r * math.sin(a)
    # Marginalen räknar med största blocket (halvaxel 2.2 * 1.6 m)
    if not clear_of_road(x, y, 4.5) or zone_clearance(x, y) < 3.0:
        continue
    sz = rng.uniform(0.6, 2.2)
    col = GRANITE if rng.random() < 0.6 else GRANITE_LICHEN
    rocks.add(
        "ell",
        (sz * rng.uniform(1.0, 1.6), sz * rng.uniform(0.8, 1.3), sz * rng.uniform(0.5, 0.8)),
        (x, y, sz * 0.2, rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), rng.uniform(0, 3.14)),
        col,
        collide=True,
    )
# Ett stort flyttblock i gläntans kant
rocks.add("ell", (3.2, 2.4, 1.8), (-24, -12, 0.6, 0, 0.05, 0.4), GRANITE, collide=True)
models.append(rocks)

logs = Model("fallen_logs")
for i in range(30):
    r = WORLD_R * math.sqrt(rng.random())
    a = rng.uniform(0, 2 * math.pi)
    x, y = r * math.cos(a), r * math.sin(a)
    # Marginalen räknar med längsta stocken (9 m) åt valfritt håll
    if not clear_of_road(x, y, 5.5) or zone_clearance(x, y) < 6.0:
        continue
    L = rng.uniform(4, 9)
    logs.add("cyl", (rng.uniform(0.15, 0.3), L), (x, y, 0.22, 0, math.pi / 2, rng.uniform(0, 3.14)), SPRUCE_BARK, collide=True)
models.append(logs)

# ----------------------------------------------------------------------------
# Tält 20 (fem stycken): tolvkantigt pyramidtält med mittstång, låga väggar,
# ingång och kaminrör.
# ----------------------------------------------------------------------------
for i, (tx, ty, yaw) in enumerate(TENTS, 1):
    t = Model(f"tent_{i}", (tx, ty, 0, 0, 0, yaw))
    R = 3.0
    wall_h = 0.6
    roof_h = 2.6
    t.add("cyl", (R, wall_h), (0, 0, wall_h / 2, 0, 0, 0), TENT_DARK, collide=True)
    t.add("cone", (R + 0.15, roof_h), (0, 0, wall_h + roof_h / 2, 0, 0, 0), TENT, collide=True)
    # Topp och kaminrör
    t.add("cyl", (0.05, 0.4), (0, 0, wall_h + roof_h + 0.1, 0, 0, 0), WOOD)
    t.add("cyl", (0.07, 1.4), (1.0, 0.6, wall_h + roof_h - 0.2, 0, 0, 0), STEEL)
    t.add("cyl", (0.1, 0.12), (1.0, 0.6, wall_h + roof_h + 0.5, 0, 0, 0), STEEL)
    # Ingång (uppslagen tältduk) mot lägrets mitt (+x i tältets ram)
    t.add("box", (0.12, 1.1, 1.4), (R - 0.05, 0, 0.7, 0, -0.35, 0), FM_BLACK)
    t.add("box", (0.05, 0.6, 1.5), (R + 0.15, 0.65, 0.75, 0, -0.25, 0.4), TENT_DARK)
    # Tältlinor med pinnar
    for k in range(6):
        a = k * math.pi / 3 + math.pi / 6
        gx, gy = math.cos(a) * (R + 1.4), math.sin(a) * (R + 1.4)
        ex, ey = math.cos(a) * R, math.sin(a) * R
        L = math.hypot(math.hypot(gx - ex, gy - ey), wall_h)
        pitch = math.atan2(math.hypot(gx - ex, gy - ey), wall_h)
        t.add("cyl", (0.01, L), ((gx + ex) / 2, (gy + ey) / 2, wall_h / 2, 0, pitch, a + math.pi), (0.6, 0.6, 0.5, 1), shadows=False)
        t.add("cyl", (0.03, 0.25), (gx, gy, 0.1, 0, 0, 0), WOOD, shadows=False)
    models.append(t)

# ----------------------------------------------------------------------------
# Lägerutrustning
# ----------------------------------------------------------------------------
camp = Model("camp_gear")
# Eldstad med stenring och sittstockar
for k in range(10):
    a = k * 2 * math.pi / 10
    camp.add("ell", (0.25, 0.2, 0.15), (-3 + 0.9 * math.cos(a), 6 + 0.9 * math.sin(a), 0.08, 0, 0, a), GRANITE)
camp.add("cyl", (0.75, 0.03), (-3, 6, 0.015, 0, 0, 0), ASH, shadows=False)
for k in range(3):
    a = k * 2.1
    camp.add("cyl", (0.08, 0.9), (-3, 6, 0.2, 0.6 * math.cos(a), 0.6 * math.sin(a), 0), WOOD)
for (lx, ly, lyaw) in ((-3, 8.3, 0), (-5.3, 6, 1.57), (-0.7, 6.3, 1.4)):
    camp.add("cyl", (0.2, 2.4), (lx, ly, 0.2, 0, math.pi / 2, lyaw), SPRUCE_BARK, collide=True)
# Vedtrave
for row in range(3):
    for k in range(6 - row):
        camp.add("cyl", (0.1, 0.6), (-10 + k * 0.2 + row * 0.1, 18.5, 0.1 + row * 0.18, math.pi / 2, 0, 0), WOOD)
# Vapenställ / lådor och drivmedelsdunkar vid fordonsplatsen
for k in range(4):
    camp.add("box", (1.2, 0.6, 0.5), (8 + k * 1.3, -19, 0.25, 0, 0, 0), FM_GREEN, collide=True)
camp.add("box", (1.2, 0.6, 0.5), (8.6, -19, 0.75, 0, 0, 0.1), FM_GREEN, collide=True)
for k in range(6):
    camp.add("box", (0.17, 0.35, 0.47), (20 + k * 0.22, -18.5, 0.235, 0, 0, 0), FM_DARK, collide=True)
# Kamouflagenät på stolpar över ett stabsbord
for (px, py) in ((-12, -8), (-6, -8), (-12, -3), (-6, -3)):
    camp.add("cyl", (0.04, 2.5), (px, py, 1.25, 0, 0, 0), WOOD)
for k in range(12):
    col = (FM_GREEN, FM_BROWN, FM_DARK)[k % 3]
    camp.add(
        "box",
        (rng.uniform(1.5, 2.5), rng.uniform(1.5, 2.5), 0.02),
        (-9 + rng.uniform(-2.5, 2.5), -5.5 + rng.uniform(-2, 2), 2.5 + rng.uniform(-0.05, 0.05), 0, 0, rng.uniform(0, 3.14)),
        col,
    )
camp.add("box", (2.0, 0.9, 0.05), (-9, -5.5, 0.75, 0, 0, 0), WOOD, collide=True)
for (lx, ly) in ((-9.9, -5.9), (-8.1, -5.9), (-9.9, -5.1), (-8.1, -5.1)):
    camp.add("cyl", (0.03, 0.75), (lx, ly, 0.375, 0, 0, 0), STEEL)
camp.add("box", (0.5, 0.35, 0.3), (-9.4, -5.5, 0.93, 0, 0, 0.2), FM_DARK)  # radio
camp.add("cyl", (0.01, 2.5), (-9.25, -5.4, 2.2, 0, 0, 0), FM_BLACK)  # antenn
models.append(camp)

# ----------------------------------------------------------------------------
# Fordon
# ----------------------------------------------------------------------------


def camo_patches(m, cx, cy, z, sx, sy, n):
    """Oregelbundna fläckar i M90-liknande färger ovanpå en yta."""
    for _ in range(n):
        m.add(
            "box",
            (rng.uniform(0.3, 0.9), rng.uniform(0.15, 0.4), 0.01),
            (cx + rng.uniform(-sx, sx), cy + rng.uniform(-sy, sy), z, 0, 0, rng.uniform(0, 3.14)),
            FM_BROWN if rng.random() < 0.5 else FM_DARK,
            shadows=False,
        )


def track_unit(m, x0, length, width, gauge):
    """Ett bandpar (höger/vänster) centrerat på x0. Returnerar bandets topphöjd."""
    r = 0.33
    for side in (-1, 1):
        y = side * gauge / 2
        m.add("box", (length - 2 * r, width, 2 * r), (x0, y, r, 0, 0, 0), RUBBER, collide=True)
        for ex in (-1, 1):
            m.add("cyl", (r, width), (x0 + ex * (length / 2 - r), y, r, math.pi / 2, 0, 0), RUBBER, collide=True)
        # Bärhjul synliga på utsidan
        for k in range(4):
            wx = x0 - length / 2 + r + 0.2 + k * (length - 2 * r - 0.4) / 3
            m.add("cyl", (0.22, 0.04), (wx, y + side * (width / 2 + 0.01), 0.3, math.pi / 2, 0, 0), STEEL)
        # Stålgaller/klackar
        for k in range(10):
            gx = x0 - length / 2 + r + k * (length - 2 * r) / 9
            m.add("box", (0.06, width + 0.02, 0.02), (gx, y, 2 * r + 0.005, 0, 0, 0), STEEL, shadows=False)
    return 2 * r


def build_bv206(name, p):
    """Bandvagn 206: två bandgående vagnar sammankopplade med styrled."""
    m = Model(name, p)
    L1, L2, W, gauge = 3.3, 3.0, 0.62, 1.2
    body_w = 1.85
    # Främre vagn (hytt + motor), centrerad på x = +2.0
    fx = 2.0
    top = track_unit(m, fx, L1, W, gauge)
    m.add("box", (L1 - 0.2, body_w, 0.35), (fx, 0, top + 0.1, 0, 0, 0), FM_GREEN, collide=True)
    # Motorhuv sluttar framåt
    m.add("box", (0.95, body_w - 0.05, 0.45), (fx + 1.0, 0, top + 0.47, 0, 0.25, 0), FM_GREEN, collide=True)
    m.add("box", (0.4, body_w - 0.1, 0.35), (fx + 1.55, 0, top + 0.2, 0, -0.5, 0), FM_GREEN)
    # Hytt
    cab_h = 1.05
    cab_z = top + 0.27 + cab_h / 2
    m.add("box", (1.9, body_w, cab_h), (fx - 0.45, 0, cab_z, 0, 0, 0), FM_GREEN, collide=True)
    m.add("box", (1.95, body_w + 0.02, 0.05), (fx - 0.45, 0, cab_z + cab_h / 2 + 0.02, 0, 0, 0), FM_DARK)
    # Framruta (lutad) och sidorutor
    m.add("box", (0.05, body_w - 0.2, 0.5), (fx + 0.52, 0, cab_z + 0.18, 0, -0.3, 0), GLASS)
    for side in (-1, 1):
        m.add("box", (0.7, 0.02, 0.4), (fx - 0.05, side * (body_w / 2 + 0.005), cab_z + 0.2, 0, 0, 0), GLASS)
        m.add("box", (0.6, 0.02, 0.4), (fx - 0.95, side * (body_w / 2 + 0.005), cab_z + 0.2, 0, 0, 0), GLASS)
    # Strålkastare, takräcke, antenn
    for side in (-1, 1):
        m.add("cyl", (0.09, 0.06), (fx + 1.5, side * 0.65, top + 0.55, 0, math.pi / 2, 0), (0.8, 0.8, 0.7, 1))
    m.add("box", (1.5, 1.5, 0.04), (fx - 0.5, 0, cab_z + cab_h / 2 + 0.12, 0, 0, 0), FM_BLACK)
    m.add("cyl", (0.01, 2.2), (fx - 1.2, 0.8, cab_z + cab_h / 2 + 1.1, 0, 0, 0), FM_BLACK)
    camo_patches(m, fx - 0.45, 0, cab_z + cab_h / 2 + 0.05, 0.8, 0.7, 10)

    # Styrled mellan vagnarna
    m.add("box", (0.9, 0.35, 0.3), (fx - L1 / 2 - 0.3, 0, top + 0.1, 0, 0, 0), FM_BLACK)
    m.add("cyl", (0.2, 0.3), (fx - L1 / 2 - 0.3, 0, top + 0.1, 0, 0, 0), STEEL)

    # Bakre vagn (trupputrymme), centrerad på x = -2.0
    rx = -2.0
    track_unit(m, rx, L2, W, gauge)
    m.add("box", (L2 - 0.1, body_w, 0.35), (rx, 0, top + 0.1, 0, 0, 0), FM_GREEN, collide=True)
    rear_h = 1.2
    rz = top + 0.27 + rear_h / 2
    m.add("box", (L2 - 0.3, body_w, rear_h), (rx - 0.05, 0, rz, 0, 0, 0), FM_GREEN, collide=True)
    m.add("box", (L2 - 0.25, body_w + 0.02, 0.05), (rx - 0.05, 0, rz + rear_h / 2 + 0.02, 0, 0, 0), FM_DARK)
    for side in (-1, 1):
        for k in range(2):
            m.add("box", (0.55, 0.02, 0.35), (rx + 0.6 - k * 0.9, side * (body_w / 2 + 0.005), rz + 0.25, 0, 0, 0), GLASS)
    # Bakdörrar
    m.add("box", (0.02, 0.75, 0.9), (rx - L2 / 2 + 0.09, -0.42, rz - 0.05, 0, 0, 0), FM_DARK)
    m.add("box", (0.02, 0.75, 0.9), (rx - L2 / 2 + 0.09, 0.42, rz - 0.05, 0, 0, 0), FM_DARK)
    camo_patches(m, rx, 0, rz + rear_h / 2 + 0.05, 1.2, 0.7, 12)
    models.append(m)


def build_tgb40(name, p):
    """Terrängbil 40 (Scania SBA111): 4x4 med frambyggd hytt och kapellflak."""
    m = Model(name, p)
    wr, ww = 0.6, 0.45
    track = 2.1
    axles = (2.35, -1.6)
    # Hjul
    for ax in axles:
        for side in (-1, 1):
            m.add("cyl", (wr, ww), (ax, side * track / 2, wr, math.pi / 2, 0, 0), RUBBER, collide=True)
            m.add("cyl", (0.3, ww + 0.02), (ax, side * track / 2, wr, math.pi / 2, 0, 0), FM_DARK)
            # Mönster (klackar) på däcket
            for k in range(8):
                a = k * math.pi / 4
                m.add(
                    "box",
                    (0.12, ww + 0.02, 0.05),
                    (ax + math.cos(a) * wr, side * track / 2, wr + math.sin(a) * wr, 0, -a, 0),
                    RUBBER,
                    shadows=False,
                )
    # Ram och axlar
    m.add("box", (6.2, 0.9, 0.3), (0.3, 0, 1.0, 0, 0, 0), FM_BLACK, collide=True)
    for ax in axles:
        m.add("cyl", (0.12, track), (ax, 0, wr, math.pi / 2, 0, 0), FM_BLACK)
    # Stänkskärmar
    for ax in axles:
        for side in (-1, 1):
            m.add("box", (1.45, 0.55, 0.06), (ax, side * track / 2, 1.3, 0, 0, 0), FM_GREEN)
    # Frambyggd hytt
    cx, cw, ch = 2.35, 2.35, 1.55
    cz = 1.25 + ch / 2
    m.add("box", (1.9, cw, ch), (cx, 0, cz, 0, 0, 0), FM_GREEN, collide=True)
    m.add("box", (0.25, cw - 0.3, 0.45), (cx + 1.05, 0, 1.3, 0, 0, 0), FM_DARK)  # kylargrill
    m.add("box", (0.2, cw + 0.2, 0.2), (cx + 1.15, 0, 1.0, 0, 0, 0), FM_BLACK)   # stötfångare
    m.add("box", (0.05, cw - 0.2, 0.65), (cx + 0.96, 0, cz + 0.35, 0, -0.08, 0), GLASS)
    for side in (-1, 1):
        m.add("box", (0.8, 0.02, 0.55), (cx + 0.3, side * (cw / 2 + 0.005), cz + 0.3, 0, 0, 0), GLASS)
        m.add("box", (0.05, 0.3, 0.35), (cx + 1.0, side * (cw / 2 + 0.2), cz + 0.3, 0, 0, 0), FM_BLACK)  # backspegel
        m.add("cyl", (0.11, 0.08), (cx + 1.12, side * 0.9, 1.55, 0, math.pi / 2, 0), (0.8, 0.8, 0.7, 1))
    m.add("box", (1.95, cw + 0.02, 0.06), (cx, 0, cz + ch / 2 + 0.03, 0, 0, 0), FM_DARK)
    camo_patches(m, cx, 0, cz + ch / 2 + 0.07, 0.8, 1.0, 10)
    # Flak med kapell på bågar
    bx, bl, bw = -1.0, 4.3, 2.4
    m.add("box", (bl, bw, 0.15), (bx, 0, 1.25, 0, 0, 0), FM_GREEN, collide=True)
    for side in (-1, 1):
        m.add("box", (bl, 0.05, 0.5), (bx, side * bw / 2, 1.55, 0, 0, 0), FM_GREEN)
    m.add("box", (0.05, bw, 0.5), (bx - bl / 2, 0, 1.55, 0, 0, 0), FM_GREEN)
    # Kapell: box + halvcylinder-tak
    m.add("box", (bl - 0.05, bw - 0.05, 1.0), (bx, 0, 2.3, 0, 0, 0), CANVAS, collide=True)
    m.add("ell", ((bl - 0.05) / 2, (bw - 0.05) / 2, 0.35), (bx, 0, 2.8, 0, 0, 0), CANVAS)
    for k in range(4):
        m.add("box", (0.06, bw, 0.02), (bx - bl / 2 + 0.4 + k * (bl - 0.8) / 3, 0, 3.1, 0, 0, 0), TENT_DARK, shadows=False)
    m.add("box", (0.02, bw - 0.3, 0.9), (bx - bl / 2 - 0.01, 0, 2.25, 0, 0, 0), TENT_DARK)
    # Reservhjul bakom hytten, avgasrör
    m.add("cyl", (0.55, 0.4), (cx - 1.15, 0, 2.4, 0, math.pi / 2, 0), RUBBER)
    m.add("cyl", (0.06, 1.8), (cx - 1.0, 1.05, 2.4, 0, 0, 0), STEEL)
    models.append(m)


# Bv 206 på uppställningsplatsen, Tgb 40 bredvid, en till Bv 206 ute på vägen
build_bv206("bv206_1", (10, -13, 0, 0, 0, 1.45))
build_tgb40("tgb40_1", (17, -14, 0, 0, 0, 1.6))
build_bv206("bv206_2", ROADSIDE_BV)

# ----------------------------------------------------------------------------
# Pickup med flak (Flatbed Falcon): dynamisk modell som kör på vägen.
# ----------------------------------------------------------------------------
# Måtten används även av ROS-noderna (ros2_ws/src/flatbed_falcon/config/falcon.yaml):
# ändras de här måste de ändras där också.
# Modellens origo ligger på marken mitt mellan axlarna; det är den punkten
# /truck/odom rapporterar. +x är framåt.
TRUCK_WHEELBASE = 3.0
TRUCK_TRACK = 1.7          # hjulavstånd sida-sida
TRUCK_WHEEL_R = 0.4
TRUCK_WHEEL_W = 0.3
TRUCK_STEER_LIMIT = 0.6    # rad
BED_LEN, BED_W = 2.5, 2.0  # flakets landningsyta (x, y)
BED_X = -1.15              # flakets mitt relativt modellens origo
BED_TOP = 1.0              # flakets ovansida över marken
TRUCK_COLOR = (0.55, 0.12, 0.10, 1)
PAD_ORANGE = (0.95, 0.45, 0.05, 1)


def box_inertia(m, x, y, z):
    return (m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y))


def inertial(m, ixx, iyy, izz):
    return (
        f"<inertial><mass>{f(float(m))}</mass><inertia><ixx>{f(float(ixx))}</ixx><ixy>0</ixy><ixz>0</ixz>"
        f"<iyy>{f(float(iyy))}</iyy><iyz>0</iyz><izz>{f(float(izz))}</izz></inertia></inertial>"
    )


class Truck:
    """Dynamisk pickup: chassi + fyra hjul, Ackermann-styrning, odometri och
    kontaktsensor på flaket. Ingen Model-instans eftersom den inte är statisk."""

    def __init__(self, name, p):
        self.name, self.pose, self.n = name, p, 0
        self.visuals = []

    def vis(self, kind, dims, p, color):
        self.n += 1
        self.visuals.append(
            f'<visual name="v{self.n}"><pose>{pose(*p)}</pose><geometry>{geom(kind, *dims)}</geometry>{material(color)}</visual>'
        )

    def sdf(self):
        r, w, L, T = TRUCK_WHEEL_R, TRUCK_WHEEL_W, TRUCK_WHEELBASE, TRUCK_TRACK
        # Kaross (visuellt): ram, hytt, motorhuv, flak med lågt räcke och landningsplatta
        self.vis("box", (4.9, 1.8, 0.35), (0.1, 0, 0.75, 0, 0, 0), STEEL)
        self.vis("box", (1.1, 1.85, 0.6), (1.95, 0, 1.15, 0, 0, 0), TRUCK_COLOR)          # motorhuv
        self.vis("box", (1.3, 1.9, 1.1), (0.75, 0, 1.45, 0, 0, 0), TRUCK_COLOR)           # hytt
        self.vis("box", (0.05, 1.7, 0.55), (1.41, 0, 1.65, 0, -0.25, 0), GLASS)           # vindruta
        for side in (-1, 1):
            self.vis("box", (0.8, 0.02, 0.45), (0.75, side * 0.96, 1.7, 0, 0, 0), GLASS)
            self.vis("cyl", (0.1, 0.06), (2.5, side * 0.7, 1.2, 0, math.pi / 2, 0), (0.9, 0.9, 0.8, 1))
        self.vis("box", (BED_LEN, BED_W, 0.1), (BED_X, 0, BED_TOP - 0.05, 0, 0, 0), STEEL)
        for side in (-1, 1):
            self.vis("box", (BED_LEN, 0.05, 0.12), (BED_X, side * (BED_W / 2 - 0.025), BED_TOP + 0.06, 0, 0, 0), TRUCK_COLOR)
        # Landningsmarkering: orange ram och ett H, ligger precis ovanpå flaket
        mz = BED_TOP + 0.003
        for dx, dy, sx, sy in ((0, 0.8, 2.1, 0.08), (0, -0.8, 2.1, 0.08), (1.0, 0, 0.08, 1.6), (-1.0, 0, 0.08, 1.6)):
            self.vis("box", (sx, sy, 0.006), (BED_X + dx, dy, mz, 0, 0, 0), PAD_ORANGE)
        for dy in (-0.35, 0.35):
            self.vis("box", (0.9, 0.1, 0.006), (BED_X, dy, mz, 0, 0, 0), WHITE)
        self.vis("box", (0.1, 0.7, 0.006), (BED_X, 0, mz, 0, 0, 0), WHITE)

        ix = box_inertia(1500, 4.8, 1.8, 0.8)
        friction = "<surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface>"
        chassis = (
            f'<link name="chassis"><pose>0 0 {f(r)} 0 0 0</pose>'
            # Tyngdpunkt lågt och något framför mitten (motor, hytt)
            f"<inertial><pose>0.2 0 0.2 0 0 0</pose><mass>1500</mass><inertia><ixx>{f(ix[0])}</ixx><ixy>0</ixy><ixz>0</ixz>"
            f"<iyy>{f(ix[1])}</iyy><iyz>0</iyz><izz>{f(ix[2])}</izz></inertia></inertial>"
            # Kollision: ram, hytt + motorhuv, och flaket (som får kontaktsensorn)
            f'<collision name="frame"><pose>0.1 0 {f(0.75 - r)} 0 0 0</pose><geometry>{geom("box", 4.9, 1.8, 0.35)}</geometry></collision>'
            f'<collision name="cab"><pose>1.25 0 {f(1.45 - r)} 0 0 0</pose><geometry>{geom("box", 2.3, 1.9, 1.1)}</geometry></collision>'
            f'<collision name="bed"><pose>{f(BED_X)} 0 {f(BED_TOP - 0.05 - r)} 0 0 0</pose><geometry>{geom("box", BED_LEN, BED_W, 0.1)}</geometry>{friction}</collision>'
            + "".join(self._shifted(-r))
            + '<sensor name="bed_contact" type="contact"><always_on>true</always_on><update_rate>50</update_rate>'
            "<topic>/truck/bed_contact</topic><contact><collision>bed</collision><topic>/truck/bed_contact</topic></contact></sensor>"
            "</link>"
        )
        links, joints = [chassis], []
        # Cylinderns axel är länkens z (hjulaxeln)
        i_side = 30 / 12 * (3 * r * r + w * w)
        wheel_in = inertial(30, i_side, i_side, 0.5 * 30 * r * r)
        wheel_fr = "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>"
        for name, x, y in (("front_left", L / 2, T / 2), ("front_right", L / 2, -T / 2),
                           ("rear_left", -L / 2, T / 2), ("rear_right", -L / 2, -T / 2)):
            wheel_geom = geom("cyl", r, w)
            links.append(
                f'<link name="{name}_wheel"><pose>{pose(x, y, r, -math.pi / 2, 0, 0)}</pose>{wheel_in}'
                f'<visual name="tyre"><geometry>{wheel_geom}</geometry>{material(RUBBER)}</visual>'
                f'<visual name="rim"><geometry>{geom("cyl", r * 0.55, w + 0.02)}</geometry>{material(STEEL)}</visual>'
                f'<collision name="collision"><geometry>{wheel_geom}</geometry>{wheel_fr}</collision></link>'
            )
            parent = "chassis"
            if name.startswith("front"):
                parent = f"{name}_steering"
                links.append(
                    f'<link name="{name}_steering"><pose>{pose(x, y * 0.85, r, 0, 0, 0)}</pose>{inertial(5, 0.05, 0.05, 0.05)}</link>'
                )
                joints.append(
                    f'<joint name="{name}_steering_joint" type="revolute"><parent>chassis</parent><child>{name}_steering</child>'
                    f"<axis><xyz>0 0 1</xyz><limit><lower>{-TRUCK_STEER_LIMIT}</lower><upper>{TRUCK_STEER_LIMIT}</upper>"
                    "<velocity>2.0</velocity><effort>20000</effort></limit></axis></joint>"
                )
            joints.append(
                f'<joint name="{name}_wheel_joint" type="revolute"><parent>{parent}</parent><child>{name}_wheel</child>'
                "<axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>"
            )
        plugins = (
            '<plugin filename="gz-sim-ackermann-steering-system" name="gz::sim::systems::AckermannSteering">'
            "<left_joint>front_left_wheel_joint</left_joint><left_joint>rear_left_wheel_joint</left_joint>"
            "<right_joint>front_right_wheel_joint</right_joint><right_joint>rear_right_wheel_joint</right_joint>"
            "<left_steering_joint>front_left_steering_joint</left_steering_joint>"
            "<right_steering_joint>front_right_steering_joint</right_steering_joint>"
            f"<kingpin_width>{f(T * 0.85)}</kingpin_width><steering_limit>{TRUCK_STEER_LIMIT}</steering_limit>"
            f"<wheel_base>{f(L)}</wheel_base><wheel_separation>{f(T)}</wheel_separation><wheel_radius>{f(r)}</wheel_radius>"
            "<steer_p_gain>8.0</steer_p_gain>"
            "<min_velocity>-3</min_velocity><max_velocity>12</max_velocity>"
            "<min_acceleration>-4</min_acceleration><max_acceleration>2.5</max_acceleration>"
            "<topic>/truck/cmd_vel</topic></plugin>"
            '<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
            "<odom_topic>/truck/odom</odom_topic><odom_frame>world</odom_frame><robot_base_frame>truck</robot_base_frame>"
            "<odom_publish_frequency>50</odom_publish_frequency><dimensions>3</dimensions></plugin>"
        )
        body = "\n      ".join(links + joints) + "\n      " + plugins
        return (
            f'    <model name="{self.name}">\n'
            f"      <pose>{pose(*self.pose)}</pose>\n"
            f"      {body}\n"
            f"    </model>\n"
        )

    def _shifted(self, dz):
        """Karossens visuals ritas i modellens ram; flytta dem till chassilänkens ram."""
        out = []
        for v in self.visuals:
            head, rest = v.split("<pose>", 1)
            p, tail = rest.split("</pose>", 1)
            vals = [float(x) for x in p.split()]
            vals[2] += dz
            out.append(f"{head}<pose>{pose(*vals)}</pose>{tail}")
        return out


# Lastbilen startar i början av landningszon 1 (inre raksträckan förbi infarten),
# vänd i körriktningen.
_z1 = next(pt for pt in ROAD_PTS if pt[5] == 1)
TRUCK_START = (_z1[1], _z1[2], 0.05, 0, 0, _z1[3])
truck = Truck("truck", TRUCK_START)
models.append(truck)

# ----------------------------------------------------------------------------
# Skriv SDF
# ----------------------------------------------------------------------------
HEADER = """<?xml version="1.0" ?>
<!--
  GENERERAD FIL - redigera inte för hand.
  Källa: worlds/generate_hive_base.py  (kör: python3 worlds/generate_hive_base.py)

  Militärt tältläger i svensk barrskog: fem tält 20, grusvägsslinga, Bv 206 och Tgb 40.
  Träden läses från /workspace/worlds/trees (repot monterat i containern), resten är SDF-primitiver.
-->
<sdf version="1.11">
  <world name="hive_base">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"></plugin>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"></plugin>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"></plugin>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"></plugin>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.55 0.58 0.6</ambient>
      <background>0.62 0.72 0.82</background>
      <sky></sky>
      <shadows>true</shadows>
    </scene>

    <!-- Låg nordisk sol -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>1 0.95 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.6 0.4 -0.55</direction>
    </light>

"""

FOOTER = """
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""

OUT.write_text(HEADER + "".join(m.sdf() for m in models) + FOOTER)
print(f"Wrote {OUT} ({placed} trees, {sum(m.n for m in models)} parts)")

# Mittlinjen för lastbilens banföljning (world ENU, en punkt per meter).
# landing_zone = id för raksträckor där drönaren får landa, annars -1.
with ROAD_CSV.open("w") as fh:
    fh.write("s,x,y,heading,curvature,landing_zone\n")
    for row in ROAD_PTS:
        fh.write(",".join(f"{v:.4f}" if isinstance(v, float) else str(v) for v in row) + "\n")
zones = ", ".join(f"{z}: {len(p)} m" for z, p in LANDING_ZONES.items())
print(f"Wrote {ROAD_CSV} (loop {ROAD_LEN:.0f} m, landing zones {zones})")
