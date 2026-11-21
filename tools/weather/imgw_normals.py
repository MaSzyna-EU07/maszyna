#!/usr/bin/env python3
"""Makes climate/stations.txt, the table of the climate of the weather stations the simulator takes the
weather of a place and a month from (environment/climate.h), out of the public measurements of IMGW-PIB.

    python tools/weather/imgw_normals.py --out climate/stations.txt
    python tools/weather/imgw_normals.py --cpp environment/climatestations.cpp      (the table built into the simulator)

It downloads the monthly data of the synoptic stations for the years of the normal (1991-2020 by default)
from danepubliczne.imgw.pl, averages them month by month for every station and writes a row a month:

    station <name> <latitude> <longitude> <elevation>
    month <1..12> <tmin> <tmax> <humidity %> <wind m/s> <precipitation mm> <days with precipitation>
          <days with snow lying> <deepest snow cm> <days with fog> <days with a storm> <cloudiness oktas>
    rose <winter|spring|summer|autumn> <shares of the wind from N NE E SE S SW W NW>     (with --roses)

The archives are kept in --cache (imgw_cache by default), so a second run downloads nothing; --offline uses
only what is there (put the archives and the two *_format.txt files there by hand where the server can't be
reached). The columns are found by the names the format files of IMGW give them, not by their numbers; a
column which can't be found stops the script with the names it did find.

The files of IMGW don't hold the places of the stations: the ones below are approximate (a few kilometres),
good enough to find the nearest station. --coords gives a file of "name;latitude;longitude;elevation" rows
to add to them or to replace them.

The paths given (--out, --cpp, --cache, --coords) have to lead inside the folder the script is run in.

Source of the data: Instytut Meteorologii i Gospodarki Wodnej - Panstwowy Instytut Badawczy. The data of
IMGW-PIB were processed.
"""
import argparse
import csv
import io
import os
import re
import sys
import unicodedata
import urllib.request
import zipfile
from collections import defaultdict

BASE = "https://danepubliczne.imgw.pl/data/dane_pomiarowo_obserwacyjne/dane_meteorologiczne/"
MONTHLY = BASE + "miesieczne/synop/"
HOURLY = BASE + "terminowe/synop/"

# approximate places of the synoptic stations: latitude, longitude, elevation
STATIONS = {
    "BIALYSTOK": (53.11, 23.16, 148), "BIELSKO-BIALA": (49.81, 19.00, 396), "CHOJNICE": (53.71, 17.53, 164),
    "CZESTOCHOWA": (50.81, 19.09, 293), "ELBLAG-MILEJEWO": (54.22, 19.54, 189), "GDANSK-SWIBNO": (54.33, 18.93, 2),
    "GORZOW WIELKOPOLSKI": (52.74, 15.28, 71), "HEL": (54.60, 18.81, 1), "JELENIA GORA": (50.90, 15.79, 342),
    "KALISZ": (51.78, 18.08, 137), "KASPROWY WIERCH": (49.23, 19.98, 1991), "KATOWICE-MUCHOWIEC": (50.24, 19.03, 278),
    "KETRZYN": (54.07, 21.37, 106), "KIELCE-SUKOW": (50.81, 20.69, 260), "KLODZKO": (50.44, 16.61, 356),
    "KOLO": (52.20, 18.66, 115), "KOLOBRZEG-DZWIRZYNO": (54.16, 15.39, 3), "KOSZALIN": (54.20, 16.15, 32),
    "KOZIENICE": (51.56, 21.54, 123), "KRAKOW-BALICE": (50.08, 19.80, 237), "KROSNO": (49.71, 21.77, 326),
    "LEBA": (54.75, 17.53, 2), "LEBORK": (54.55, 17.76, 38), "LEGNICA": (51.19, 16.21, 122),
    "LESKO": (49.47, 22.34, 420), "LESZNO": (51.84, 16.53, 91), "LUBLIN-RADAWIEC": (51.22, 22.39, 238),
    "LODZ-LUBLINEK": (51.72, 19.40, 187), "MIKOLAJKI": (53.79, 21.59, 127), "MLAWA": (53.10, 20.36, 147),
    "NOWY SACZ": (49.63, 20.69, 292), "OLSZTYN": (53.77, 20.42, 133), "OPOLE": (50.63, 17.97, 165),
    "OSTROLEKA": (53.08, 21.56, 94), "PILA": (53.13, 16.75, 72), "PLOCK": (52.59, 19.73, 106),
    "POZNAN-LAWICA": (52.42, 16.83, 83), "PRZEMYSL": (49.80, 22.77, 279), "RACIBORZ": (50.06, 18.19, 205),
    "RESKO-SMOLSKO": (53.77, 15.42, 52), "RZESZOW-JASIONKA": (50.11, 22.04, 200), "SANDOMIERZ": (50.70, 21.72, 217),
    "SIEDLCE": (52.18, 22.24, 152), "SLUBICE": (52.35, 14.60, 21), "SNIEZKA": (50.74, 15.74, 1603),
    "SULEJOW": (51.35, 19.87, 188), "SUWALKI": (54.13, 22.95, 184), "SWINOUJSCIE": (53.92, 14.24, 6),
    "SZCZECIN": (53.40, 14.62, 1), "SZCZECINEK": (53.71, 16.75, 137), "TARNOW": (50.03, 20.98, 209),
    "TERESPOL": (52.08, 23.62, 133), "TORUN": (53.04, 18.60, 69), "USTKA": (54.59, 16.85, 6),
    "WARSZAWA-OKECIE": (52.16, 20.96, 106), "WIELUN": (51.21, 18.56, 200), "WLODAWA": (51.55, 23.53, 177),
    "WROCLAW-STRACHOWICE": (51.10, 16.88, 120), "ZAKOPANE": (49.29, 19.96, 855), "ZIELONA GORA": (51.93, 15.52, 192),
    "ZAMOSC": (50.70, 23.21, 212), "HALA GASIENICOWA": (49.24, 20.01, 1520), "GDANSK-PORT POLNOCNY": (54.40, 18.70, 2),
}

MONTH_DAYS = [31, 28.25, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
# what a month of the lowlands of central Poland has, where a station lacks an element
FALLBACK = {"humidity": 80.0, "wind": 3.5, "precipitation": 45.0, "wet_days": 9.0, "snow_days": 0.0, "snow_depth": 0.0,
            "fog_days": 3.0, "storm_days": 1.0, "cloud": 5.5}
# the columns: our name, the code the format file of IMGW gives the column, and the words its name has to hold where the code
# isn't there (folded to ascii, lower case)
DAILY_COLUMNS = {
    "tmax": ("TMXS", ("srednia maksymalna temperatura",)), "tmin": ("TMNS", ("srednia minimalna temperatura",)),
    "precipitation": ("SUMM", ("miesieczna suma opad",)), "snow_depth": ("PKSN", ("maksymalna wysokosc pokrywy",)),
    "snow_days": ("PSDN", ("liczba dni z pokrywa",)), "rain_days": ("DESD", ("liczba dni z opadem deszczu",)),
    "snowfall_days": ("SNID", ("liczba dni z opadem sniegu",)), "fog_days": ("MGLD", ("liczba dni z mgla",)),
    "storm_days": ("BURD", ("liczba dni z burza",)),
}
TERM_COLUMNS = {"cloud": ("NOS", ("zachmurzenie",)), "wind": ("FWS", ("predkosc wiatru",)), "humidity": ("WLGS", ("wilgotnosc",))}
HOURLY_COLUMNS = {"direction": ("", ("kierunek wiatru",)), "speed": ("", ("predkosc wiatru",))}
# the statuses of a value: not measured; no such phenomenon (a zero), which for the counts of days means the station doesn't
# observe the phenomenon at all
NO_MEASUREMENT = "8"
NO_PHENOMENON = "9"


def fold(text):
    """Lower case without the Polish letters, for comparing names."""
    text = text.replace("ł", "l").replace("Ł", "L")
    return "".join(c for c in unicodedata.normalize("NFKD", text) if not unicodedata.combining(c)).lower().strip()


def inside(path, option):
    """The path, made absolute; the script stops when it leads out of the folder the script is run in."""
    root = os.path.realpath(os.getcwd())
    full = os.path.realpath(os.path.join(root, path))
    if os.path.commonpath([root, full]) != root:
        sys.exit(f"{option} has to lead to a place inside {root}, not to {full}")
    return full


def fetch(url, cache, offline):
    """The bytes of a file of the server, kept in the cache; None when it isn't there."""
    path = os.path.join(cache, os.path.basename(url[len(BASE):].replace("/", "_")))
    if os.path.exists(path):
        with open(path, "rb") as file:  # NOSONAR: the cache went through inside(), which keeps it in the folder the script is run in
            return file.read()
    if offline:
        return None
    try:
        with urllib.request.urlopen(url, timeout=120) as response:
            data = response.read()
    except OSError as error:
        print(f"  can't get {url}: {error}", file=sys.stderr)
        return None
    os.makedirs(cache, exist_ok=True)  # NOSONAR: the cache went through inside()
    with open(path, "wb") as file:  # NOSONAR: the cache went through inside()
        file.write(data)
    return data


def format_names(text):
    """The columns a format file of IMGW lists, in their order, each (code, name): a line is the code of the column in capitals,
    then its name."""
    names = []
    for line in text.splitlines():
        match = re.match(r"^\s*([A-Z][A-Z0-9]*)\s+(\S.*)$", line.rstrip())
        if match:
            names.append((match.group(1), fold(match.group(2))))
    return names


def find_columns(names, wanted, what):
    """Of each wanted column: its index, the index of the column of its status after it (or None), and whether it counts days."""
    found = {}
    for key, (code, words) in wanted.items():
        index = next((i for i, (column, _) in enumerate(names) if code and column == code), None)
        if index is None:
            index = next((i for i, (_, name) in enumerate(names) if all(word in name for word in words)), None)
        if index is None:
            sys.exit(f"the format of the {what} data has no column for {key} ({code} {' '.join(words)}); it has:\n  " +
                     "\n  ".join(f"{column} {name}" for column, name in names))
        status = index + 1 if index + 1 < len(names) and names[index + 1][1].startswith("status") else None
        found[key] = (index, status, names[index][1].startswith("liczba dni"))
    return found


def read_rows(archive, prefix):
    """The rows of the csv files of a zip archive whose names start with the prefix."""
    with zipfile.ZipFile(io.BytesIO(archive)) as zipped:
        for name in zipped.namelist():
            if not os.path.basename(name).lower().startswith(prefix):
                continue
            text = zipped.read(name).decode("cp1250", errors="replace")
            yield from csv.reader(io.StringIO(text))


def value(row, column):
    """The number of a column of a row, None where it wasn't measured."""
    index, status, days = column
    if index >= len(row):
        return None
    flag = row[status].strip() if status is not None and status < len(row) else ""
    if flag == NO_MEASUREMENT or (days and flag == NO_PHENOMENON):
        return None
    try:
        return float(row[index].strip().replace(",", "."))
    except ValueError:
        return None


def monthly_archives(first, last):
    """The addresses of the archives of the monthly data: by five years up to 2000, by the year after."""
    urls = []
    for start in range((first - 1) // 5 * 5 + 1, 2001, 5):
        if start + 4 >= first and start <= last:
            urls.append(f"{MONTHLY}{start}_{start + 4}/{start}_{start + 4}_m_s.zip")
    urls.extend(f"{MONTHLY}{year}/{year}_m_s.zip" for year in range(max(first, 2001), last + 1))
    return urls


def gather(arguments):
    """Sums of every element for every station and month: sums[code][month][element] = [sum, count], and the names each station
    had: a station keeps its code over the years, its name changes (LODZ in some years, LODZ-LUBLINEK in others)."""
    formats = {}
    for kind in ("d", "t"):
        data = fetch(f"{MONTHLY}s_m_{kind}_format.txt", arguments.cache, arguments.offline)
        if data is None:
            sys.exit(f"the format file s_m_{kind}_format.txt isn't there (the cache: {arguments.cache})")
        formats[kind] = format_names(data.decode("cp1250", errors="replace"))
    daily = find_columns(formats["d"], DAILY_COLUMNS, "monthly (s_m_d)")
    term = find_columns(formats["t"], TERM_COLUMNS, "monthly (s_m_t)")
    sums = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: [0.0, 0])))
    names = defaultdict(set)
    for url in monthly_archives(arguments.first, arguments.last):
        archive = fetch(url, arguments.cache, arguments.offline)
        if archive is None:
            continue
        print(f"  {url.rsplit('/', 1)[-1]}")
        for prefix, columns in (("s_m_d", daily), ("s_m_t", term)):
            for row in read_rows(archive, prefix):
                if len(row) < 4 or not row[2].strip().isdigit() or not row[3].strip().isdigit():
                    continue
                year, month = int(row[2]), int(row[3])
                if not (arguments.first <= year <= arguments.last and 1 <= month <= 12):
                    continue
                station = row[0].strip()
                names[station].add(fold(row[1]).upper())
                for key, column in columns.items():
                    number = value(row, column)
                    if number is not None:
                        cell = sums[station][month][key]
                        cell[0] += number
                        cell[1] += 1
    return sums, names


def place_of(names, places):
    """The name a station is written under and its place: the fullest of its names with a known place, or a known place whose
    name starts with one of its names (WARSZAWA: WARSZAWA-OKECIE); (name, None) when none is known."""
    ordered = sorted(names, key=len, reverse=True)
    for name in ordered:
        if name in places:
            return name, places[name]
    for name in ordered:
        longer = sorted(known for known in places if known.startswith(name + "-"))
        if len(longer) == 1:
            return longer[0], places[longer[0]]
    return ordered[0], None


def roses(station_code, arguments):
    """The shares of the wind from the eight directions by the season, of the hourly data of a station; None without them."""
    data = fetch(f"{HOURLY}s_t_format.txt", arguments.cache, arguments.offline)
    if data is None:
        return None
    columns = find_columns(format_names(data.decode("cp1250", errors="replace")), HOURLY_COLUMNS, "hourly (s_t)")
    counts = [[0] * 8 for _ in range(4)]
    for year in range(max(arguments.first, arguments.last - 9), arguments.last + 1):
        archive = fetch(f"{HOURLY}{year}/{year}_{station_code}_s.zip", arguments.cache, arguments.offline)
        if archive is None:
            continue
        for row in read_rows(archive, "s_t"):
            if len(row) < 4 or not row[3].strip().isdigit():
                continue
            direction, speed = value(row, columns["direction"]), value(row, columns["speed"])
            # a calm has no direction
            if direction is None or speed is None or speed < 0.5 or not 0 < direction <= 360:
                continue
            season = (int(row[3]) % 12) // 3
            counts[season][int((direction + 22.5) // 45) % 8] += 1
    if any(sum(season) < 200 for season in counts):
        return None
    return [[count / sum(season) for count in season] for season in counts]


def mean(cells, key, least):
    cell = cells.get(key)
    return cell[0] / cell[1] if cell and cell[1] >= least else None


def station_rows(name, place, months, least):
    """The rows of a station, None when its temperatures aren't complete."""
    rows = [f"station {name.title().replace(' ', '_')} {place[0]:.2f} {place[1]:.2f} {place[2]:.0f}"]
    for month in range(1, 13):
        cells = months.get(month, {})
        tmin, tmax = mean(cells, "tmin", least), mean(cells, "tmax", least)
        if tmin is None or tmax is None:
            return None
        rain, snow = mean(cells, "rain_days", least), mean(cells, "snowfall_days", least)
        # a day may have both: the more frequent of the two, and half of the other
        wet = None if rain is None or snow is None else min(max(rain, snow) + 0.5 * min(rain, snow), MONTH_DAYS[month - 1])

        def element(key, fallback):
            number = mean(cells, key, least)
            return FALLBACK[fallback] if number is None else number

        rows.append("month {} {:.1f} {:.1f} {:.0f} {:.1f} {:.0f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}".format(
            month, tmin, tmax, element("humidity", "humidity"), element("wind", "wind"), element("precipitation", "precipitation"),
            FALLBACK["wet_days"] if wet is None else wet, element("snow_days", "snow_days"), element("snow_depth", "snow_depth"),
            element("fog_days", "fog_days"), element("storm_days", "storm_days"), element("cloud", "cloud")))
    return rows


def write_cpp(path, stations, first, last):
    """The table as the source file the simulator is built with (environment/climatestations.cpp)."""
    lines = [
        "/*", "This Source Code Form is subject to the", "terms of the Mozilla Public License, v.", "2.0. If a copy of the MPL was not",
        "distributed with this file, You can", "obtain one at", "http://mozilla.org/MPL/2.0/.", "*/", "",  # NOSONAR: the notice of the MPL as every source of the simulator has it, not an address fetched
        f"// climate normals of the synoptic stations of IMGW-PIB, {first}-{last}. made by tools/weather/imgw_normals.py --cpp of the",
        "// public monthly data of the stations: not to be edited by hand, made again when the normals change.",
        "// source of the data: Instytut Meteorologii i Gospodarki Wodnej - Panstwowy Instytut Badawczy; the data were processed",
        "", '#include "stdafx.h"', '#include "environment/climatestations.h"', "", "namespace climate", "{", "",
        "std::span<station_record const> builtin_stations()", "{",
        "\t// name, latitude, longitude, elevation; a row a month: tmin, tmax, humidity, wind, precipitation, its days, snow days, snow depth,",
        "\t// fog days, storm days, cloudiness",
        f"\tstatic constexpr std::array<station_record, {len(stations)}> records{{{{",
    ]
    for _, rows in sorted(stations):
        _, name, latitude, longitude, elevation = rows[0].split()
        lines.append(f'\t    {{"{name}", {latitude}f, {longitude}f, {elevation}.f, {{{{')
        for row in rows[1:13]:
            values = ", ".join((number if "." in number else number + ".") + "f" for number in row.split()[2:])
            lines.append(f"\t        {{{values}}},")
        lines.append("\t    }}},")
    lines += ["\t}};", "\treturn records;", "}", "", "} // namespace climate"]
    with open(path, "w", encoding="ascii", newline="\n") as file:  # NOSONAR: --cpp went through inside()
        file.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", default="climate/stations.txt", help="the table to write")
    parser.add_argument("--cache", default="imgw_cache", help="where the downloaded archives are kept")
    parser.add_argument("--offline", action="store_true", help="use only what the cache holds")
    parser.add_argument("--first", type=int, default=1991, help="first year of the normal")
    parser.add_argument("--last", type=int, default=2020, help="last year of the normal")
    parser.add_argument("--least", type=int, default=20, help="years a month needs to count")
    parser.add_argument("--roses", action="store_true", help="wind roses of the hourly data of the last ten years (a large download)")
    parser.add_argument("--cpp", help="also write the table as the source file of the simulator (environment/climatestations.cpp)")
    parser.add_argument("--coords", help='file of "name;latitude;longitude;elevation" rows of stations')
    arguments = parser.parse_args()
    # the files read and written stay inside the folder the script is run in (the root of the repository)
    arguments.out = inside(arguments.out, "--out")
    arguments.cache = inside(arguments.cache, "--cache")
    if arguments.cpp:
        arguments.cpp = inside(arguments.cpp, "--cpp")
    if arguments.coords:
        arguments.coords = inside(arguments.coords, "--coords")

    places = dict(STATIONS)
    if arguments.coords:
        with open(arguments.coords, encoding="utf-8") as file:  # NOSONAR: --coords went through inside()
            for row in csv.reader(file, delimiter=";"):
                if len(row) >= 4:
                    places[fold(row[0]).upper()] = (float(row[1]), float(row[2]), float(row[3]))

    print("monthly data:")
    sums, names = gather(arguments)
    if not sums:
        sys.exit("no data: the server can't be reached and the cache is empty")
    least = min(arguments.least, arguments.last - arguments.first + 1)
    lines = ["# climate normals of the synoptic stations of IMGW-PIB, {}-{}, made by tools/weather/imgw_normals.py".format(arguments.first, arguments.last),
             "# source of the data: Instytut Meteorologii i Gospodarki Wodnej - Panstwowy Instytut Badawczy; the data were processed"]
    written = 0
    stations = []
    for code, name, place in sorted((code, *place_of(names[code], places)) for code in sums):
        if place is None:
            print(f"  {name}: no place known, left out (see --coords)", file=sys.stderr)
            continue
        rows = station_rows(name, place, sums[code], least)
        if rows is None:
            print(f"  {name}: fewer than {least} years of temperatures, left out", file=sys.stderr)
            continue
        if arguments.roses:
            rose = roses(code, arguments)
            if rose:
                rows.extend("rose {} {}".format(season, " ".join(f"{share:.3f}" for share in shares))
                            for season, shares in zip(("winter", "spring", "summer", "autumn"), rose))
        stations.append((name, rows))
        written += 1
    for _, rows in sorted(stations):
        lines.extend(rows)
    os.makedirs(os.path.dirname(arguments.out) or ".", exist_ok=True)  # NOSONAR: --out went through inside()
    with open(arguments.out, "w", encoding="ascii", newline="\n") as file:  # NOSONAR: --out went through inside()
        file.write("\n".join(lines) + "\n")
    print(f"{written} stations written to {arguments.out}")
    if arguments.cpp:
        write_cpp(arguments.cpp, [(name, [row for row in rows if not row.startswith("rose")]) for name, rows in stations], arguments.first, arguments.last)
        print(f"and to {arguments.cpp}")


if __name__ == "__main__":
    main()
