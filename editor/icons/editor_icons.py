#!/usr/bin/env python3
# draws the icons of the scenery editor into editor_icons.png, next to this script: white shapes on a transparent ground,
# 64 x 64 pixels each, 8 in a row, in the order of editor_icons::icon (editor/editorIcons.hpp).
# the editor tints them as it draws them; the png is built into the executable by CMake, see CMakeLists.txt.
# needs Pillow: python3 editor_icons.py
import math
import os

from PIL import Image, ImageDraw

CELL = 64
SUPER = 4  # drawn this many times bigger, then scaled down for smooth edges
S = CELL * SUPER
W = 18  # width of the lines, at the bigger size
COLUMNS = 8


def p(x, y):
    # coordinates given on a 64 x 64 grid
    return (x * SUPER, y * SUPER)


def line(d, points, width=W):
    pts = [p(*q) for q in points]
    d.line(pts, fill=255, width=width, joint="curve")
    for q in (pts[0], pts[-1]):
        r = width / 2
        d.ellipse((q[0] - r, q[1] - r, q[0] + r, q[1] + r), fill=255)


def poly(d, points, outline=False, width=W):
    pts = [p(*q) for q in points]
    if outline:
        line(d, points + [points[0]], width)
    else:
        d.polygon(pts, fill=255)


def circle(d, x, y, r, outline=False, width=W):
    box = (p(x - r, y - r), p(x + r, y + r))
    if outline:
        d.ellipse(box, outline=255, width=width)
    else:
        d.ellipse(box, fill=255)


def rect(d, x0, y0, x1, y1, outline=False, width=W, radius=0):
    box = (p(x0, y0), p(x1, y1))
    if outline:
        d.rounded_rectangle(box, radius=radius * SUPER, outline=255, width=width)
    else:
        d.rounded_rectangle(box, radius=radius * SUPER, fill=255)


def arrowhead(d, x, y, angle, size=9):
    # filled head pointing at angle (degrees, 0 = right, 90 = down)
    a = math.radians(angle)
    tip = (x + math.cos(a) * size * 0.6, y + math.sin(a) * size * 0.6)
    left = (x + math.cos(a + 2.5) * size, y + math.sin(a + 2.5) * size)
    right = (x + math.cos(a - 2.5) * size, y + math.sin(a - 2.5) * size)
    poly(d, [tip, left, right])


def arc(d, x, y, r, start, end, width=W):
    box = (p(x - r, y - r), p(x + r, y + r))
    d.arc(box, start, end, fill=255, width=width)


def dashed(d, a, b, dash=6, gap=4, width=W):
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    if length == 0:
        return
    ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
    t = 0.0
    while t < length:
        e = min(t + dash, length)
        line(d, [(a[0] + ux * t, a[1] + uy * t), (a[0] + ux * e, a[1] + uy * e)], width)
        t = e + gap


# the icons, in the order of editor_icons::icon


def select(d):
    poly(d, [(16, 8), (16, 50), (26, 40), (33, 56), (40, 53), (33, 37), (47, 37)])


def insert(d):
    # a box, with a plus over its corner
    poly(d, [(8, 26), (28, 16), (48, 26), (28, 36)], outline=True)
    line(d, [(8, 26), (8, 46), (28, 56), (48, 46), (48, 26)])
    line(d, [(28, 36), (28, 56)])
    circle(d, 48, 14, 11)


def brush(d):
    line(d, [(52, 8), (30, 32)], W + 4)
    poly(d, [(26, 30), (34, 38), (24, 52), (8, 56), (12, 42)])
    for x, y in ((44, 40), (52, 30), (50, 50)):
        circle(d, x, y, 3)


def area_fill(d):
    outline = [(8, 18), (34, 8), (56, 24), (50, 54), (16, 52)]
    for i in range(len(outline)):
        dashed(d, outline[i], outline[(i + 1) % len(outline)], 6, 5, W - 4)
    for x, y in ((24, 38), (38, 30), (36, 44)):
        poly(d, [(x, y - 9), (x + 6, y + 4), (x - 6, y + 4)])


def copy_to_bank(d):
    rect(d, 18, 6, 46, 26, outline=True, radius=3)
    line(d, [(32, 26), (32, 40)])
    arrowhead(d, 32, 42, 90, 12)
    line(d, [(8, 40), (8, 56), (56, 56), (56, 40)])


def translate(d):
    line(d, [(32, 10), (32, 54)])
    line(d, [(10, 32), (54, 32)])
    arrowhead(d, 32, 8, 270, 13)
    arrowhead(d, 32, 56, 90, 13)
    arrowhead(d, 8, 32, 180, 13)
    arrowhead(d, 56, 32, 0, 13)


def rotate(d):
    # turning clockwise, the gap at the top
    arc(d, 32, 34, 21, 300, 225)
    a = math.radians(225)
    arrowhead(d, 32 + math.cos(a) * 21, 34 + math.sin(a) * 21, 315, 15)
    circle(d, 32, 34, 4)


def scale(d):
    rect(d, 8, 30, 34, 56, radius=2)
    for a, b in (((8, 8), (56, 8)), ((56, 8), (56, 56)), ((8, 8), (8, 22)), ((42, 56), (56, 56))):
        dashed(d, a, b, 6, 5, W - 6)
    line(d, [(30, 34), (48, 16)])
    arrowhead(d, 50, 14, 315, 14)


def local_space(d):
    # the axes turned with the model
    o = (22, 42)
    for angle, length in ((-20, 34), (-110, 30), (150, 18)):
        a = math.radians(angle)
        e = (o[0] + math.cos(a) * length, o[1] + math.sin(a) * length)
        line(d, [o, e], W - 2)
        arrowhead(d, e[0], e[1], angle, 11)
    circle(d, o[0], o[1], 5)


def ortho(d):
    # the plan seen from over it: a grid without perspective
    rect(d, 8, 8, 56, 56, outline=True, width=W - 2)
    for t in (24, 40):
        line(d, [(t, 8), (t, 56)], W - 8)
        line(d, [(8, t), (56, t)], W - 8)


def perspective(d):
    # the grid going away
    poly(d, [(20, 14), (44, 14), (58, 54), (6, 54)], outline=True, width=W - 2)
    line(d, [(28, 14), (24, 54)], W - 8)
    line(d, [(36, 14), (40, 54)], W - 8)
    line(d, [(15, 30), (49, 30)], W - 8)


def sculpt(d):
    poly(d, [(4, 56), (22, 26), (32, 38), (42, 22), (60, 56)])
    line(d, [(50, 18), (50, 4)], W - 4)
    arrowhead(d, 50, 3, 270, 11)


def smooth(d):
    pts = [(x, 40 - 12 * math.sin((x - 6) / 52 * math.pi)) for x in range(6, 59, 2)]
    line(d, pts)
    line(d, [(6, 54), (58, 54)], W - 6)
    circle(d, 32, 28, 14, outline=True, width=W - 8)


def chunks(d):
    for x, y in ((6, 6), (33, 6), (6, 33)):
        rect(d, x, y, x + 25, y + 25, radius=3)
    rect(d, 33, 33, 58, 58, outline=True, width=W - 6, radius=3)
    line(d, [(45.5, 39), (45.5, 52)], W - 8)
    line(d, [(39, 45.5), (52, 45.5)], W - 8)


def orthophoto(d):
    rect(d, 6, 10, 58, 54, outline=True, radius=4)
    poly(d, [(10, 50), (26, 28), (36, 40), (42, 34), (54, 50)])
    circle(d, 44, 21, 5)


def road(d):
    line(d, [(24, 8), (8, 56)])
    line(d, [(40, 8), (56, 56)])
    dashed(d, (32, 8), (32, 56), 8, 6, W - 4)


def place(d):
    # a pin standing on the place
    poly(d, [(32, 58), (18, 34), (46, 34)])
    circle(d, 32, 26, 15)


def lanes(d):
    for x in (14, 50):
        line(d, [(x, 6), (x, 58)], W - 4)
    dashed(d, (32, 6), (32, 58), 7, 6, W - 6)
    line(d, [(23, 52), (23, 22)], W - 6)
    arrowhead(d, 23, 18, 270, 10)
    line(d, [(41, 12), (41, 42)], W - 6)
    arrowhead(d, 41, 46, 90, 10)


def lay_track(d):
    # two rails on sleepers, going up to the right
    for t in range(0, 6):
        x, y = 10 + t * 9, 54 - t * 9
        line(d, [(x - 8, y - 8), (x + 8, y + 8)], W - 8)
    line(d, [(4, 50), (50, 4)], W - 4)
    line(d, [(14, 60), (60, 14)], W - 4)


def switch(d):
    line(d, [(18, 60), (18, 4)], W - 2)
    line(d, [(34, 60), (34, 4)], W - 2)
    pts = [(18 + 30 * (1 - math.cos(t / 20 * math.pi / 2)), 52 - t * 2.4) for t in range(0, 21)]
    line(d, pts, W - 2)
    pts = [(34 + 22 * (1 - math.cos(t / 20 * math.pi / 2)), 60 - t * 2.4) for t in range(0, 21)]
    line(d, pts, W - 2)


def straight(d):
    line(d, [(12, 52), (52, 12)])
    rect(d, 4, 44, 20, 60, radius=2)
    rect(d, 44, 4, 60, 20, radius=2)


def curve(d):
    arc(d, 56, 56, 44, 180, 270)
    dashed(d, (56, 56), (25, 25), 5, 4, W - 8)
    circle(d, 12, 56, 6)
    circle(d, 56, 12, 6)
    circle(d, 25, 25, 6)


def signal(d):
    line(d, [(32, 34), (32, 60)], W - 2)
    line(d, [(20, 60), (44, 60)], W - 2)
    rect(d, 20, 4, 44, 38, radius=10)


def objects(d):
    # a post by the track with a board on it
    line(d, [(24, 30), (24, 60)], W - 2)
    rect(d, 10, 6, 38, 30, radius=3)
    line(d, [(42, 60), (60, 60)], W - 6)
    line(d, [(48, 52), (60, 52)], W - 6)


def vehicle(d):
    poly(d, [(4, 22), (50, 22), (60, 32), (60, 46), (4, 46)])
    circle(d, 16, 52, 6)
    circle(d, 46, 52, 6)
    line(d, [(4, 58), (60, 58)], W - 8)


def profile(d):
    pts = [(6, 46), (22, 30), (38, 36), (58, 14)]
    line(d, pts)
    for q in pts:
        circle(d, q[0], q[1], 5)
    line(d, [(6, 58), (58, 58)], W - 8)


def speed(d):
    arc(d, 32, 40, 24, 180, 360)
    for a in (200, 240, 280, 320):
        r = math.radians(a)
        line(d, [(32 + math.cos(r) * 15, 40 + math.sin(r) * 15), (32 + math.cos(r) * 20, 40 + math.sin(r) * 20)], W - 8)
    line(d, [(32, 40), (46, 24)], W - 2)
    circle(d, 32, 40, 6)


def joints(d):
    line(d, [(4, 26), (26, 26)])
    line(d, [(38, 26), (60, 26)])
    line(d, [(4, 40), (26, 40)])
    line(d, [(38, 44), (60, 44)])
    rect(d, 28, 14, 36, 54, outline=True, width=W - 10, radius=2)


def infra(d):
    # a catenary mast with its arm and the wire hanging from it
    line(d, [(14, 60), (14, 6)], W + 2)
    line(d, [(14, 12), (54, 12)], W - 4)
    line(d, [(14, 26), (46, 12)], W - 8)
    line(d, [(44, 12), (44, 28)], W - 8)
    pts = [(x, 30 + 0.012 * (x - 36) ** 2) for x in range(14, 61, 2)]
    line(d, pts, W - 6)


def gauge(d):
    # the outline of the structure gauge over the rails
    line(d, [(12, 52), (12, 22), (22, 8), (42, 8), (52, 22), (52, 52)], W - 4)
    line(d, [(4, 56), (60, 56)], W - 6)
    for x in (24, 40):
        rect(d, x - 3, 48, x + 3, 56)


def turntable(d):
    # the pit with its bridge, and the tracks leading out of it
    circle(d, 32, 32, 20, outline=True, width=W - 4)
    line(d, [(18, 46), (46, 18)], W)
    for a in (-90, -30, 30):
        r = math.radians(a)
        line(d, [(32 + math.cos(r) * 24, 32 + math.sin(r) * 24), (32 + math.cos(r) * 31, 32 + math.sin(r) * 31)], W - 4)


# the fields of work of the editor, in the row over the toolbar


def work_surroundings(d):
    # a tree and a house by it
    poly(d, [(18, 4), (32, 30), (4, 30)])
    poly(d, [(18, 14), (34, 44), (2, 44)])
    line(d, [(18, 44), (18, 60)], W - 2)
    poly(d, [(36, 34), (49, 22), (62, 34)])
    rect(d, 39, 34, 59, 60)


def work_tracks(d):
    # the track going away, rails on sleepers
    line(d, [(8, 60), (26, 4)], W - 4)
    line(d, [(56, 60), (38, 4)], W - 4)
    for y, half in ((54, 28), (40, 22), (28, 17), (18, 13), (10, 10)):
        line(d, [(32 - half, y), (32 + half, y)], W - 8 if y > 20 else W - 10)


def work_roads(d):
    # a car seen from its side
    poly(d, [(4, 44), (4, 34), (14, 30), (22, 18), (44, 18), (52, 30), (60, 34), (60, 44)])
    circle(d, 17, 46, 8)
    circle(d, 47, 46, 8)


def work_terrain(d):
    # the mesh of the ground over a hill
    def at(u, v):
        # u across 0..1, v from the back (0) to the front (1)
        half = 18 + 12 * v
        x = 32 + (u - 0.5) * 2 * half
        hill = 18 * math.exp(-((u - 0.5) ** 2) / 0.07 - ((v - 0.35) ** 2) / 0.15)
        return (x, 24 + 34 * v - hill)
    steps = [i / 12 for i in range(13)]
    for v in (0.0, 0.33, 0.66, 1.0):
        line(d, [at(u, v) for u in steps], W - 6)
    for u in (0.0, 0.33, 0.66, 1.0):
        line(d, [at(u, v) for v in steps], W - 6)


ICONS = [select, insert, brush, area_fill, copy_to_bank, translate, rotate, scale,
         local_space, ortho, perspective, sculpt, smooth, chunks, orthophoto, road,
         place, lanes, lay_track, switch, straight, curve, signal, objects,
         vehicle, profile, speed, joints, infra, gauge, turntable, work_surroundings,
         work_tracks, work_roads, work_terrain]


def cutouts(name, d):
    # the parts drawn in the colour of the ground: the plus of the insert, the hole of the pin, the lights of the signal
    if name == "insert":
        line(d, [(48, 8), (48, 20)], W - 8)
        line(d, [(42, 14), (54, 14)], W - 8)
    elif name == "place":
        circle(d, 32, 26, 6)
    elif name == "signal":
        circle(d, 32, 14, 5)
        circle(d, 32, 28, 5)
    elif name == "vehicle":
        rect(d, 10, 27, 20, 35)
        rect(d, 26, 27, 36, 35)
        rect(d, 42, 27, 50, 35)
    elif name == "work_surroundings":
        # the door and the gap between the crowns of the tree
        rect(d, 46, 46, 52, 60)
        line(d, [(9, 31), (27, 31)], 8)
    elif name == "work_roads":
        # the windows, and the gaps around the wheels
        poly(d, [(25, 22), (31, 22), (31, 30), (19, 30)])
        poly(d, [(35, 22), (42, 22), (47, 30), (35, 30)])
        circle(d, 17, 46, 11, outline=True, width=12)
        circle(d, 47, 46, 11, outline=True, width=12)
        circle(d, 17, 46, 3)
        circle(d, 47, 46, 3)


def main():
    rows = (len(ICONS) + COLUMNS - 1) // COLUMNS
    atlas = Image.new("RGBA", (COLUMNS * CELL, rows * CELL), (255, 255, 255, 0))
    for index, draw in enumerate(ICONS):
        mask = Image.new("L", (S, S), 0)
        d = ImageDraw.Draw(mask)
        draw(d)
        holes = Image.new("L", (S, S), 0)
        cutouts(draw.__name__, ImageDraw.Draw(holes))
        mask.paste(0, (0, 0), holes)
        mask = mask.resize((CELL, CELL), Image.LANCZOS)
        icon = Image.new("RGBA", (CELL, CELL), (255, 255, 255, 0))
        icon.putalpha(mask)
        atlas.paste(icon, ((index % COLUMNS) * CELL, (index // COLUMNS) * CELL))
    atlas.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "editor_icons.png"), optimize=True)


if __name__ == "__main__":
    main()
