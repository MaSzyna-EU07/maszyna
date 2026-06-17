# Format pliku EU7v2 (`.eu7v2`)

> Dokumentacja oparta na kodzie źródłowym z:
> - `scene/eu7/v2/` — format, emitter, loader, testy roundtrip
> - `scene/eu7/eu7_bake_parser.cpp` — headless bake scenariusza
> - `eu07-parser/include/eu07/scene/bake/` — pool bake, streaming terenu
>
> Stan: **czerwiec 2026**, wersja kontenera **v1** (`kVersion = 1`).

---

## Spis treści

1. [Przegląd ogólny](#1-przegląd-ogólny)
2. [Nagłówek pliku](#2-nagłówek-pliku)
3. [Struktura chunku](#3-struktura-chunku)
4. [Rodzaje pliku (`file_kind`)](#4-rodzaje-pliku-file_kind)
5. [Katalog chunków](#5-katalog-chunków)
6. [Typy podstawowe](#6-typy-podstawowe)
7. [STRS — tablica łańcuchów](#7-strs--tablica-łańcuchów)
8. [META — metadane modułu](#8-meta--metadane-modułu)
9. [INCL — include rekurencyjne](#9-incl--include-rekurencyjne)
10. [PLCE — placementy modułów wielokrotnego użytku](#10-plce--placementy-modułów-wielokrotnego-użytku)
11. [PROT + INST — prototypy i instancje modeli](#11-prot--inst--prototypy-i-instancje-modeli)
12. [SHPE — kształty (triangles)](#12-shpe--kształty-triangles)
13. [MESH — teren (tile)](#13-mesh--teren-tile)
14. [LINE — linie](#14-line--linie)
15. [Rekordy symulacji](#15-rekordy-symulacji)
16. [Bake headless i drzewo modułów](#16-bake-headless-i-drzewo-modułów)
17. [Ścieżki plików](#17-ścieżki-plików)

---

## 1. Przegląd ogólny

Plik `.eu7v2` to binarny format skompilowanego modułu scenerii:

- **Moduły wielokrotnego użytku** — osobny plik `.eu7v2` per `.inc` / `.scm`, placementy jako lean **PLCE**
- **Prototypy + instancje** — chunk **PROT** (deduplikacja meshów) + **INST** (transformacja w world-space)
- **Chunki pomijalne** — nieznany FourCC można pominąć po polu `size`
- **Little-endian** — deterministyczny zapis niezależny od endianness hosta
- **Centralna tablica STRS** — wszystkie stringi przez `string_id` (`uint32`)

Magic pliku: **`EU7C`** (FourCC `'E','U','7','C'`).

---

## 2. Nagłówek pliku

```
Offset  Rozmiar  Typ       Opis
------  -------  --------  -------------------------
0       4        uint32    Magic = 'EU7C'
4       2        uint16    Version (= 1)
6       2        uint16    file_kind (patrz sekcja 4)
8       4        uint32    Reserved (= 0)
12      4        uint32    Reserved (= 0)
16      ...      chunks    Strumień chunkowy (do EOF)
```

Nagłówek ma **16 bajtów**. Brak pola „rozmiar całego pliku”.

---

## 3. Struktura chunku

```
Offset  Rozmiar  Typ      Opis
------  -------  -------  -----------------------------------------
0       4        uint32   Chunk ID — FourCC little-endian
4       8        uint64   Payload size (tylko payload, BEZ nagłówka chunku)
12      ...      bytes    Payload
```

Pole `size` to rozmiar **samego payloadu** (`uint64`), bez nagłówka chunku.

Chunki są sekwencyjne do EOF. Nieznany chunk: `seek(payload_size)`.

FourCC — bajty w pliku w kolejności ASCII (np. `STRS` → `53 54 52 53`).

---

## 4. Rodzaje pliku (`file_kind`)

| Wartość | Nazwa      | Typowy źródłowy plik        | Opis                                      |
|---------|------------|-----------------------------|-------------------------------------------|
| `1`     | `sim`      | `.scn`, `.scm`, `.sbt`, `.ctr` | Moduł scenariusza / root                 |
| `2`     | `module`   | `.inc`                      | Moduł wielokrotnego użytku (trawa, drzewo…) |
| `3`     | `tile`     | (planowane)                 | Kafel 1 km — teren + instancje            |
| `4`     | `manifest` | (planowane)                 | Indeks kafli / modułów mapy               |

Emitter runtime (`eu7v2_emit_runtime.cpp`) ustawia:
- `.inc` → `file_kind::module`
- pozostałe rozszerzenia modułów → `file_kind::sim`

Loader akceptuje `sim` i `module` (`eu7v2_load.cpp`).

---

## 5. Katalog chunków

| FourCC  | Stała C++     | Typ pliku   | Opis                                              |
|---------|---------------|-------------|---------------------------------------------------|
| `STRS`  | `chunk::strs` | wszystkie   | Tablica stringów (zawsze pierwszy logicznie)      |
| `META`  | `chunk::meta` | sim/module  | Metadane modułu (first-init, placement params)    |
| `INCL`  | `chunk::incl` | sim/module  | Include rekurencyjne (submoduły `.eu7v2`)         |
| `PLCE`  | `chunk::plce` | sim/module  | Lean placementy → osobne moduły `.eu7v2`          |
| `PROT`  | `chunk::prot` | sim/module/tile | Prototypy modeli (mesh bez transformacji)   |
| `INST`  | `chunk::inst` | sim/module/tile | Instancje modeli (proto + transform)        |
| `MESH`  | `chunk::mesh` | tile/sim    | Teren — mesh z origin f64 + wierzchołki f32       |
| `SHPE`  | `chunk::shpe` | sim/module  | Kształty triangles/strip/fan (nie-teren)        |
| `LINE`  | `chunk::line` | sim/module  | Geometria linii                                   |
| `TRAK`  | `chunk::trak` | sim         | Tory                                              |
| `TRAC`  | `chunk::trac` | sim         | Przewody trakcyjne                                |
| `PWRS`  | `chunk::pwrs` | sim         | Źródła zasilania                                  |
| `MEMC`  | `chunk::memc` | sim         | Komórki pamięci                                   |
| `LAUN`  | `chunk::laun` | sim         | Wyzwalacze zdarzeń                                |
| `EVNT`  | `chunk::evnt` | sim         | Zdarzenia                                         |
| `SOND`  | `chunk::sond` | sim         | Dźwięki                                           |
| `DYNM`  | `chunk::dynm` | sim         | Pojazdy dynamiczne                                |
| `TRST`  | `chunk::trst` | sim/module  | Zestawy wagonowe                                  |
| `TRGR`  | `chunk::trgr` | sim         | Precomputed track graph (planowane)               |
| `SIDX`  | `chunk::sidx` | manifest    | Indeks sekcji przestrzennej (planowane)           |

Chunki o zerowej liczności (pusta tablica) **nie są zapisywane** przez emitter.

---

## 6. Typy podstawowe

| Typ w pliku | Rozmiar | Opis                                              |
|-------------|---------|---------------------------------------------------|
| `uint8`     | 1 B     | Bajt bez znaku                                    |
| `uint16`    | 2 B     | LE                                                |
| `uint32`    | 4 B     | LE                                                |
| `uint64`    | 8 B     | LE                                                |
| `int32`     | 4 B     | LE, ze znakiem                                    |
| `float32`   | 4 B     | IEEE 754                                          |
| `float64`   | 8 B     | IEEE 754 — pozycje world-space, originy mesh      |
| `string_id` | 4 B     | Indeks STRS; `0xFFFFFFFF` = brak (`kNoString`)  |

Współrzędne instancji modeli i placementów: **float64** (x/y/z).  
Wierzchołki mesh/shape: **float32** względem origin (f64).

---

## 7. STRS — tablica łańcuchów

```
uint32   count
Powtórzone count razy:
    uint32   length
    char[]   data (length bajtów, bez null-terminatora)
```

Indeks `0xFFFFFFFF` (`kNoString`) oznacza brak stringu.

---

## 8. META — metadane modułu

Layout wersjonowany (pole `layout_version`):

```
uint32   layout_version          (= 1)
uint32   first_init_count
uint8    has_terrain_chunk        (informacyjne)
uint8    has_pack_chunk           (informacyjne)
uint8    placement_origin_x       (numer parametru include, 0 = brak)
uint8    placement_origin_y
uint8    placement_origin_z
uint8    placement_rotation_y
```

---

## 9. INCL — include rekurencyjne

Submoduły wczytywane rekurencyjnie (np. zagnieżdżone `.scm`, tory, CTR):

```
uint32   count
Powtórzone count razy:
    uint32      source_line
    string_id   source_path        (.scm/.inc tekstowy)
    string_id   binary_path        (.eu7v2)
    uint32      param_count
    string_id[] parameters
    TransformRecord site_transform:
        uint32   origin_stack_count
        dvec3[]  origin_stack
        uint32   scale_stack_count
        dvec3[]  scale_stack
        dvec3    rotation            (stopnie, XYZ)
        uint32   group_depth
```

---

## 10. PLCE — placementy modułów wielokrotnego użytku

Pliki placement (flora, dekoracje) emitują lean rekordy wskazujące na **osobny** moduł `.eu7v2`:

```
uint32   count
Powtórzone count razy:
    string_id   module_path        (np. scenery/grass_l61/20.eu7v2)
    string_id   texture_override   (kNoString jeśli brak)
    float64     x, y, z            (world-space)
    float32     rotation_y         (stopnie)
    uint8       cell_id             (0xFF = brak)
```

**Bake:** unikalne `.inc` z placement file są bake'owane jako osobne moduły `file_kind::module`.  
**Runtime:** loader syntetyzuje include z PLCE (ładuje wskazany moduł z transformacją).

Typowy rozmiar: **~37 B / placement**.

Przykład: `204_trawky_ter.scm` (110k linii include) → `.eu7v2` ~4 MB (PLCE) + osobne `grass_l61/20.eu7v2` per unikalny `.inc`.

---

## 11. PROT + INST — prototypy i instancje modeli

### PROT — prototyp

```
uint32   count
Powtórzone count razy:
    string_id   model_file
    string_id   texture_file
    uint8       flags              (bit 0=transition, 1=is_terrain, 2=instanceable)
    float32     range_min
    float32     range_max
    uint32      light_state_count
    float32[]   light_states
    uint32      light_color_count
    uint32[]    light_colors
```

### INST — instancja

```
uint32   count
Powtórzone count razy:
    uint8       flags              (bit 0=has_scale, 1=texture_override, 2=has_node)
    uint32      proto              (indeks PROT)
    float64     x, y, z
    float32     ax, ay, az         (kąty Eulera, stopnie)
    uint8       cell_id
    [flags & 1]: float32 sx, sy, sz
    [flags & 2]: string_id texture_override
    [flags & 4]: node_record
```

Root scenariusza: modele z PACK bake są deduplikowane do PROT+INST (flattened world-space).

---

## 12. SHPE — kształty (triangles)

```
uint32   count
Powtórzone count razy:
    node_record     node
    uint8           translucent
    string_id       material
    lighting_block  (12 × float32: diffuse/ambient/specular RGBA)
    float64         ox, oy, oz     (origin world-space)
    uint32          vertex_count
    Powtórzone vertex_count razy:
        float32 px, py, pz       (względem origin)
        float32 nx, ny, nz
        float32 u, v
```

Typ węzła (`node.type`) niesie subtype: `triangles`, `triangle_strip`, `triangle_fan`.

Duże pliki terenu (same `node … triangles`) mogą być streamowane przy bake; shapes trafiają do **shape spool** na dysku, potem do chunku SHPE przy emit.

---

## 13. MESH — teren (tile)

Format terenu dla kafli 1 km (origin f64 + relative f32):

```
uint32   count
Powtórzone count razy:
    string_id   material
    uint8       translucent
    float64     ox, oy, oz
    uint32      vertex_count
    Powtórzone vertex_count razy:
        float32 px, py, pz, nx, ny, nz, u, v
```

---

## 14. LINE — linie

```
uint32   count
Powtórzone count razy:
    node_record
    lighting_block
    float32     line_width
    float64     ox, oy, oz
    uint32      vertex_count
    dvec3[]     vertices           (float64 × 3)
```

---

## 15. Rekordy symulacji

Chunki symulacji: `TRAK`, `TRAC`, `PWRS`, `MEMC`, `LAUN`, `EVNT`, `SOND`, `DYNM`, `TRST`.  
Wspólny nagłówek węzła: **`node_record`**. Stringi przez STRS. Pozycje torów/krzywych Béziera: **float64**.

Szczegóły pól: `scene/eu7/v2/eu7v2_records.h` (funkcje `write_*` / `read_*`).

---

## 16. Bake headless i drzewo modułów

CLI (binarny `eu07`, bez GUI):

```
eu07.exe --eu7v2-bake <sciezka.scn|.inc> [opcje]
```

| Opcja                    | Opis                                              |
|--------------------------|---------------------------------------------------|
| `--eu7v2-verify`         | Roundtrip: przeładuj `.eu7v2`, porównaj liczności rekordów |
| `--eu7v2-mem-limit-gb N` | Limit pamięci procesu (domyślnie 50 GB); włącza tryb spool |
| `--eu7v2-threads N`      | Wątki pool bake (0 = auto = liczba rdzeni)        |
| `--eu7v2-max-parse N`    | Równoległe parse (0 = auto)                       |
| `--eu7v2-heavy-parse-mb N` | Próg serial parse dużych plików (0 = wyłączony w trybie spool) |

**Przepływ bake scenariusza:**

1. Root `.scn` → enqueue drzewa modułów (dedup po ścieżce kanonicznej)
2. Pool workerów bake'uje moduły równolegle
3. Każdy `.scm`/`.inc` → osobny `.eu7v2` obok źródła tekstowego
4. Placement `.scm` → **PLCE** + enqueue unikalnych `.inc` jako `module`
5. Root emit na końcu (PACK → PROT+INST, includes z podmodułów)
6. Opcjonalnie verify batch wszystkich modułów

Implementacja: `scene/eu7/eu7_bake_parser.cpp`, `eu07-parser/.../bake_tree.hpp`.

---

## 17. Ścieżki plików

```cpp
binary_path_from_text("scenery/foo/bar.scm")  → scenery/foo/bar.eu7v2
binary_path_from_text("scenery/grass_l61/20.inc") → scenery/grass_l61/20.eu7v2
binary_path_from_text("scenery/foo/base.ctr") → scenery/foo/base.ctr.eu7v2
```

Rozszerzenia `.scm`, `.scn`, `.sbt`, `.inc` → `<stem>.eu7v2`.  
Inne (np. `.ctr`) → `<nazwa_pliku>.eu7v2` (bez utraty suffixu).

---

## Pliki implementacji

| Temat              | Plik                                              |
|--------------------|---------------------------------------------------|
| Format core        | `scene/eu7/v2/eu7v2_format.h`                   |
| Scene payloads     | `scene/eu7/v2/eu7v2_scene.h`                    |
| Sim records        | `scene/eu7/v2/eu7v2_records.h`                    |
| Emit RuntimeModule | `scene/eu7/v2/eu7v2_emit_runtime.cpp`           |
| Load runtime       | `scene/eu7/v2/eu7v2_load.cpp`                     |
| Test roundtrip     | `scene/eu7/v2/eu7v2_test.cpp`                     |
| Headless bake      | `scene/eu7/eu7_bake_parser.cpp`                 |
| Bake pool          | `eu07-parser/include/eu07/scene/bake/bake_tree.hpp` |

---

*Dokumentacja na podstawie kodu; stan na 17 czerwca 2026.*
