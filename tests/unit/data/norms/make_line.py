#!/usr/bin/env python3
# writes line.scn next to this script: a line of plain tracks for the golden report of the norms (test_norms_report.cpp), each track
# written the way the game writes it (TTrack::export_as_text_). The curves are cubic Bezier paths: an arc of up to 20 degrees a track,
# its control vectors 4/3 tan(a/4) R long; a transition curve a cubic parabola, which a Bezier curve is exactly. The cant is the roll of
# the paths, on the base of 1500 mm, leaning into the turn; the grade the height of the ends and of the control vectors.
#   python3 make_line.py
import math
import os

BASE = 1500.0  # mm


class Line:
    def __init__(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.dx, self.dz = 0.0, 1.0  # along +z
        self.grade = 0.0
        self.count = 0
        self.lines = []

    def left(self):
        # the normal to the left, the way the curvature is positive: counterclockwise in x, z
        return -self.dz, self.dx

    def track(self, comment, p0, c1, c2, p3, roll0, roll1, speed, length, radius=0.0):
        self.count += 1
        name = "n%02d" % self.count
        # the heights of the ends and of the control vectors along the grade
        y0 = self.y
        y3 = y0 + self.grade * length
        c1y = self.grade * math.hypot(c1[0], c1[1])
        c2y = -self.grade * math.hypot(c2[0], c2[1])
        self.y = y3
        numbers = [p0[0], y0, p0[1], roll0, c1[0], c1y, c1[1], c2[0], c2y, c2[1], p3[0], y3, p3[1], roll1, radius]
        text = " ".join(fmt(v) for v in numbers)
        self.lines.append("// %s" % comment)
        self.lines.append("node -1 0 %s track normal %s 1.435 0.15 25 20 0 flat vis rail_screw_used1 4 tpbps-new2 0.2 0.5 1.1 %s velocity %s endtrack"
                          % (name, fmt(length), text, fmt(speed)))

    def straight(self, comment, length, speed, cant0=0.0, cant1=0.0):
        p0 = (self.x, self.z)
        p3 = (self.x + self.dx * length, self.z + self.dz * length)
        self.track(comment, p0, (0.0, 0.0), (0.0, 0.0), p3, roll(cant0), roll(cant1), speed, length)
        self.x, self.z = p3

    def arc(self, comment, radius, length, turn, speed, cant):
        # turn: 1 to the left, -1 to the right; split into pieces of up to 20 degrees
        angle = length / radius
        pieces = max(1, math.ceil(math.degrees(angle) / 20.0 - 1e-9))
        for i in range(pieces):
            a = angle / pieces
            nx, nz = self.left()
            cx, cz = self.x + turn * radius * nx, self.z + turn * radius * nz
            # the point and the heading turned by a around the centre
            rx, rz = self.x - cx, self.z - cz
            ca, sa = math.cos(turn * a), math.sin(turn * a)
            ex, ez = cx + rx * ca - rz * sa, cz + rx * sa + rz * ca
            edx, edz = self.dx * ca - self.dz * sa, self.dx * sa + self.dz * ca
            k = 4.0 / 3.0 * math.tan(a / 4.0) * radius
            text = comment if pieces == 1 else "%s, piece %d of %d" % (comment, i + 1, pieces)
            self.track(text, (self.x, self.z), (self.dx * k, self.dz * k), (-edx * k, -edz * k), (ex, ez),
                       turn * roll(cant), turn * roll(cant), speed, radius * a, radius)
            self.x, self.z, self.dx, self.dz = ex, ez, edx, edz

    def transition_in(self, comment, radius, length, turn, speed, cant):
        # y = x^3 / (6 R L) along the heading, to the side of the turn: the curvature goes from 0 to about 1 / R
        nx, nz = self.left()
        sx, sz = self.dx, self.dz
        side = turn * length * length / (6.0 * radius)
        ex = self.x + sx * length + nx * side
        ez = self.z + sz * length + nz * side
        slope = turn * length / (2.0 * radius)
        c2 = (-(sx * length + nx * 3.0 * side) / 3.0, -(sz * length + nz * 3.0 * side) / 3.0)
        self.track(comment, (self.x, self.z), (sx * length / 3.0, sz * length / 3.0), c2, (ex, ez), 0.0, turn * roll(cant), speed, length)
        norm = math.hypot(1.0, slope)
        self.dx, self.dz = (sx + nx * slope) / norm, (sz + nz * slope) / norm
        self.x, self.z = ex, ez

    def transition_out(self, comment, radius, length, turn, speed, cant):
        # the mirror of the one in: from the curvature of the arc to the straight it ends on
        angle = turn * math.atan(length / (2.0 * radius))
        ca, sa = math.cos(angle), math.sin(angle)
        bx, bz = self.dx * ca - self.dz * sa, self.dx * sa + self.dz * ca
        nbx, nbz = -bz, bx
        side = turn * length * length / (6.0 * radius)
        fx = self.x + length * bx - nbx * side
        fz = self.z + length * bz - nbz * side
        c1 = ((length * bx - nbx * 3.0 * side) / 3.0, (length * bz - nbz * 3.0 * side) / 3.0)
        c2 = (-length * bx / 3.0, -length * bz / 3.0)
        self.track(comment, (self.x, self.z), c1, c2, (fx, fz), turn * roll(cant), 0.0, speed, length)
        self.x, self.z, self.dx, self.dz = fx, fz, bx, bz


def roll(cant):
    return math.degrees(math.asin(cant / BASE))


def fmt(value):
    text = "%.6f" % value
    text = text.rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def main():
    line = Line()
    line.straight("a straight, 150 m at 100 km/h", 150.0, 100.0)
    line.transition_in("a transition of 60 m into R 600 m to the left, the cant ramp 0 - 60 mm along it", 600.0, 60.0, 1, 100.0, 60.0)
    line.arc("the arc of R 600 m, 100 m, the cant 60 mm: I 137 mm at 100 km/h", 600.0, 100.0, 1, 100.0, 60.0)
    line.transition_out("the transition out of it, 60 m, the cant ramp 60 - 0 mm", 600.0, 60.0, 1, 100.0, 60.0)
    line.straight("a straight of 28 m, shorter than 0.35 V", 28.0, 100.0)
    line.arc("R 400 m to the right with no transition, 80 m at 60 km/h", 400.0, 80.0, -1, 60.0, 0.0)
    line.straight("a straight of 10 m between reverse curves", 10.0, 60.0)
    line.arc("R 500 m to the left with no transition, 60 m at 60 km/h", 500.0, 60.0, 1, 60.0, 0.0)
    line.straight("a straight of 50 m, level", 50.0, 100.0)
    line.grade = 0.008
    line.straight("a straight of 50 m rising 8 per mille from a break of the grade", 50.0, 100.0)
    line.grade = 0.0
    line.straight("a straight of 30 m, level again, the cant ramp 0 - 90 mm along it, 3 mm/m", 30.0, 100.0, 0.0, 90.0)
    line.arc("R 1200 m to the left with no transition, 150 m, the cant 90 mm", 1200.0, 150.0, 1, 100.0, 90.0)
    line.straight("a straight of 60 m, the cant ramp 90 - 0 mm along it", 60.0, 100.0, 90.0, 0.0)
    line.straight("a straight of 100 m", 100.0, 100.0)
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "line.scn"), "w", newline="\n") as out:
        out.write("// a line for the golden report of the norms, written by make_line.py\n")
        out.write("\n".join(line.lines) + "\n")


if __name__ == "__main__":
    main()
