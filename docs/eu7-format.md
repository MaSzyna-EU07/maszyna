# Format pliku EU7B (`.eu7`)

> Dokumentacja oparta na kodzie źródłowym z:
> - `scene/eu7/` — runtime reader/loader (maszyna-fresh)
> - `parser/include/eu07/scene/binary/` — writer/reader (narzędzie CLI)
>
> Stan: **czerwiec 2026**, obejmuje wersje v4–v8.

---

## Spis treści

1. [Przegląd ogólny](#1-przegląd-ogólny)
2. [Nagłówek pliku i wersje](#2-nagłówek-pliku-i-wersje)
3. [Struktura chunku](#3-struktura-chunku)
4. [Kolejność i reguły chunkowania](#4-kolejność-i-reguły-chunkowania)
5. [Typy podstawowe i kodowanie](#5-typy-podstawowe-i-kodowanie)
6. [Slim Node — wspólny nagłówek węzła](#6-slim-node--wspólny-nagłówek-węzła)
7. [Transform Context](#7-transform-context)
8. [Packed vertex — pakowany wierzchołek](#8-packed-vertex--pakowany-wierzchołek)
9. [Lighting block — blok oświetlenia](#9-lighting-block--blok-oświetlenia)
10. [Catalog chunkowy — zestawienie](#10-catalog-chunkowy--zestawienie)
11. [STRS — tablica łańcuchów](#11-strs--tablica-łańcuchów)
12. [INCL — referencje do podmodułów](#12-incl--referencje-do-podmodułów)
13. [TRAK — tory](#13-trak--tory)
14. [TRAC — przewody trakcyjne](#14-trac--przewody-trakcyjne)
15. [PWRS — źródła zasilania trakcji](#15-pwrs--źródła-zasilania-trakcji)
16. [TERR — teren (terrain shapes)](#16-terr--teren-terrain-shapes)
17. [MESH — kształty geometryczne](#17-mesh--kształty-geometryczne)
18. [LINE — linie](#18-line--linie)
19. [MODL — instancje modeli 3D](#19-modl--instancje-modeli-3d)
20. [MEMC — komórki pamięci](#20-memc--komórki-pamięci)
21. [LAUN — wyzwalacze zdarzeń](#21-laun--wyzwalacze-zdarzeń)
22. [DYNM — pojazdy dynamiczne](#22-dynm--pojazdy-dynamiczne)
23. [SOND — dźwięki otoczenia](#23-sond--dźwięki-otoczenia)
24. [TRSE — zestawy wagonowe](#24-trse--zestawy-wagonowe)
25. [EVNT — zdarzenia](#25-evnt--zdarzenia)
26. [FINT — licznik first-init](#26-fint--licznik-first-init)
27. [PLAC — parametry umieszczenia include](#27-plac--parametry-umieszczenia-include)
28. [PIDX — indeks sekcji PACK (v7+)](#28-pidx--indeks-sekcji-pack-v7)
29. [PACK — instancje modeli per sekcja 1 km (v7+)](#29-pack--instancje-modeli-per-sekcja-1-km-v7)
30. [PROT — prototypy modeli (v8+)](#30-prot--prototypy-modeli-v8)
31. [Semantyka strumieniowania PACK w runtime](#31-semantyka-strumieniowania-pack-w-runtime)
32. [Historia wersji](#32-historia-wersji)
33. [Ścieżki plików i konwencje nazewnictwa](#33-ścieżki-plików-i-konwencje-nazewnictwa)

---

## 1. Przegląd ogólny

Plik `.eu7` (EU7B — **EU7 Binary**) to binarny format zapisu skompilowanego modułu scenerii symulatora EU07. Zastępuje tekst source (`*.scm`, `*.inc`, `*.scn`) jednym zoptymalizowanym plikiem binarnym gotowym do szybkiego wczytania przez silnik.

**Kluczowe cechy:**
- **Little-endian** dla wszystkich liczb całkowitych i zmiennoprzecinkowych
- **Chunki samoopisowe** — nieznane chunki mogą być pomijane (`seekg` po `chunk_size`)
- **Tablica łańcuchów (STRS)** — wszystkie stringi współdzielone przez indeks `uint32`
- **Strumieniowanie sekcji 1 km** — w plikach głównych scenariusza modele są dzielone na kafelki 1 km × 1 km (chunki PIDX + PACK), wczytywane asynchronicznie
- **Format skaluje się** od małych modułów include (kilka KB) do głównych scenariuszy (setki MB w PACK)

---

## 2. Nagłówek pliku i wersje

### Layout nagłówka

```
Offset  Rozmiar  Typ       Opis
------  -------  --------  -------------------------
0       4        char[4]   Magic = 'EU7B'  (0x45 0x55 0x37 0x42)
4       4        uint32    Version (patrz tabela poniżej)
8       ...      chunks    Strumień chunkowy (do EOF)
```

Nie ma pola "łączny rozmiar pliku" w nagłówku. Po nagłówku następują chunki aż do EOF.

### Tabela wersji

| Wartość | Stała (maszyna-fresh)   | Stała (parser CLI)        | Opis                                                           |
|---------|-------------------------|---------------------------|----------------------------------------------------------------|
| `1`     | —                       | `kVersionLegacy`          | Legacy — **nieobsługiwane** w CLI ani runtime                  |
| `2`     | —                       | `kVersionRuntimeF64`      | Skalary i Vec3 jako `double` (float64) na dysku               |
| `3`     | —                       | `kVersionRuntimeF32`      | Skalary i Vec3 jako `float32` na dysku                        |
| `4`     | `kEu7VersionV4`         | `kVersionRuntimeV4`       | Slim node + packed mesh (snorm16 normy + half-float UV)        |
| `5`     | `kEu7VersionV5`         | `kVersionRuntimeV5`       | Adds TERR z batched terrain per sekcja 1 km                    |
| `6`     | `kEu7VersionV6`         | `kVersionRuntime`         | Adds site transform w rekordach INCL                           |
| `7`     | `kEu7VersionV7`         | `kVersionRuntimeV7`       | Adds PIDX + PACK — modele scenerii per sekcja 1 km (streaming) |
| `8`     | `kEu7VersionV8`         | —                         | Adds PROT (prototypy) + nowy format sekcji PACK v8            |
| `9`     | `kEu7VersionV9`         | `kVersionRuntimeV9`       | PROT v9 (resolved_texture, pack_flags) + PACK sekcji v13      |

**Runtime maszyna-fresh obsługuje wersje 4–9.** Wersje 1–3 nie są obsługiwane przez żaden z dostępnych loaderów.

---

## 3. Struktura chunku

```
Offset  Rozmiar  Typ      Opis
------  -------  -------  -----------------------------------------
0       4        uint32   Chunk ID — FourCC little-endian (4 znaki ASCII)
4       4        uint32   Chunk total size — ŁĄCZNIE z nagłówkiem (≥ 8)
8       ...      bytes    Payload (rozmiar = chunk_total_size - 8)
```

> **Uwaga:** pole `chunk_total_size` **wlicza** 8 bajtów własnego nagłówka. Aby obliczyć rozmiar payloadu: `payload_size = chunk_total_size - 8`.

Strumień chunkowy nie ma terminator record. Parsowanie trwa do momentu napotkania EOF.

### FourCC — sposób kodowania

ID chunku jest 4-znakowym ASCII wpisanym w kolejności little-endian do `uint32`:

```cpp
constexpr uint32_t make_id4(char a, char b, char c, char d) {
    return (uint32(d) << 24) | (uint32(c) << 16) | (uint32(b) << 8) | uint32(a);
}
// Np. kChunkPack = make_id4('P','A','C','K')
// Bytes w pliku: 50 41 43 4B  →  odczytane jako uint32 LE = 0x4B434150
```

Przy odczycie człowiek widzi w hex-dumpie bajty w kolejności `P A C K`, co odpowiada oczytaniu `'PACK'` wprost.

---

## 4. Kolejność i reguły chunkowania

1. **STRS** musi wystąpić **przed** wszystkimi chunkiami używającymi indeksów string (w praktyce jest zawsze pierwszym chunkiem).
2. **PACK** jest **pomijany przy pełnym parse** — runtime zapamiętuje jedynie offset początku payloadu i rozmiar. Faktyczne dane są czytane on-demand przez `read_pack_section()`.
3. **PROT** musi wystąpić **przed** PACK (prototypy muszą być załadowane do pamięci zanim sekcje PACK będą rozwijane).
4. **PIDX** musi wystąpić przed próbą streamingu (runtime czyta PIDX przy załadowaniu modułu).
5. Nierozpoznane chunki są **ignorowane** — reader skacze do `chunk_start + payload_size`.

---

## 5. Typy podstawowe i kodowanie

Wszystkie typy całkowite i zmiennoprzecinkowe — **little-endian**.

| Typ w pliku | C++ typ docelowy | Rozmiar | Opis                                              |
|-------------|------------------|---------|---------------------------------------------------|
| `uint8`     | `uint8_t`        | 1 B     | Bajt bez znaku                                    |
| `int16`     | `int16_t`        | 2 B     | LE, ze znakiem                                    |
| `uint16`    | `uint16_t`       | 2 B     | LE, bez znaku                                     |
| `int32`     | `int32_t`        | 4 B     | LE, ze znakiem                                    |
| `uint32`    | `uint32_t`       | 4 B     | LE, bez znaku                                     |
| `uint64`    | `uint64_t`       | 8 B     | LE, bez znaku (lo-word najpierw)                  |
| `float32`   | `float`          | 4 B     | IEEE 754 single precision                         |
| `f64_disk`  | `double`         | 4 B     | **float32 na dysku** — wczytywany jako double w RAM |
| `Vec3`      | `glm::dvec3`     | 12 B    | 3× `f64_disk` (= 3× float32, razem 12 B)         |
| `snorm16`   | `float`          | 2 B     | int16 / 32767.0 → zakres [-1, +1]                 |
| `half16`    | `float`          | 2 B     | IEEE 754 half precision                           |
| `string_id` | `uint32_t`       | 4 B     | Indeks do tablicy STRS; `0xFFFFFFFF` = pusty      |

> **`f64_disk`**: Od wersji v3 wzwyż skalary (dystanse, kąty, napięcia itp.) i współrzędne Vec3 są zapisywane jako **float32** (4 B), mimo że runtime przechowuje je jako `double`. Patrz: `io::writeF64()` w `io.hpp` — wewnętrznie rzutuje do `float` przed zapisem.

---

## 6. Slim Node — wspólny nagłówek węzła

Każdy obiekt sceny (tor, model, dźwięk itp.) poprzedzony jest **slim node** — skompresowanym nagłówkiem z opcjonalnymi polami sterowanymi bajtem flag.

### Layout

```
Offset  Rozmiar  Typ        Opis
------  -------  ---------  -------------------------------------------
0       1        uint8      flags (bitmaska, patrz poniżej)
---  poniższe pola są OPCJONALNE, zależne od flags ---
+0      4        string_id  name (jeśli bit 0 set)
+0      4        f64_disk   range_squared_min (jeśli bit 1 set)
+0      4        f64_disk   range_squared_max (jeśli bit 2 set; domyślnie +∞)
+0     16        Vec3+f32   bounding sphere: center(Vec3) + radius(float32) (jeśli bit 3)
+0      4        uint32     group_handle (jeśli bit 4 set)
+0      ...      Transform  transform context (jeśli bit 5 set)
```

### Bitmaska `flags`

| Bit | Stała                    | Znaczenie                                           |
|-----|--------------------------|-----------------------------------------------------|
| 0   | `kNodeFlagHasName`       | pole `name` (string_id) jest obecne                 |
| 1   | `kNodeFlagHasRangeMin`   | pole `range_squared_min` (f64_disk) jest obecne     |
| 2   | `kNodeFlagHasRangeMax`   | pole `range_squared_max` jest obecne (inaczej +∞)   |
| 3   | `kNodeFlagHasBounds`     | bounding sphere: Vec3 center + float32 radius       |
| 4   | `kNodeFlagHasGroup`      | pole `group_handle` (uint32) jest obecne            |
| 5   | `kNodeFlagHasTransform`  | transform context jest obecny (patrz sekcja 7)      |
| 6   | `kNodeFlagNotVisible`    | węzeł domyślnie niewidoczny (`visible = false`)     |

### Bounding sphere

```
Vec3    center    (12 B)
float32 radius    (4 B)
```

Łącznie 16 B, obecne tylko gdy bit 3 set.

---

## 7. Transform Context

Kontekst transformacji lokacji węzła — stos offsetów i skal + obrót.

```
Offset  Rozmiar           Typ       Opis
------  ----------------  --------  ----------------------------------------
0       1                 uint8     origin_count (maks. 255)
1       12 × origin_count Vec3[]    origin_stack (offsety translacji)
+       1                 uint8     scale_count (maks. 255)
+       12 × scale_count  Vec3[]    scale_stack (skala)
+       12                Vec3      rotation (kąty Eulera w stopniach, XYZ)
+       1                 uint8     group_depth
```

Kolejność stosowania transformacji (w writer: `transform_point()`):
1. Obrót Y (o `rotation.y` stopni)
2. Skalowanie (`scale_stack.back()`)
3. Translacja (`origin_stack.back()`)

---

## 8. Packed vertex — pakowany wierzchołek

Używany w chunkach MESH i TERR (od v4).

```
Offset  Rozmiar  Typ      Opis
------  -------  -------  ------------------------------------------
0       4        float32  position.x
4       4        float32  position.y
8       4        float32  position.z
12      2        int16    normal.x jako snorm16  (/ 32767)
14      2        int16    normal.y jako snorm16
16      2        int16    normal.z jako snorm16
18      2        uint16   u jako half-float (IEEE 754 fp16)
20      2        uint16   v jako half-float
```

Łącznie: **22 bajty** na wierzchołek.

W chunkach LINE wierzchołki to **tylko pozycja** (Vec3 = 12 B; brak normalnych i UV).

---

## 9. Lighting block — blok oświetlenia

Opcjonalny blok oświetlenia materiału węzła (48 bajtów).

```
Offset  Rozmiar  Typ      Opis
------  -------  -------  ------------------
0       16       4×f32    diffuse  RGBA
16      16       4×f32    ambient  RGBA
32      16       4×f32    specular RGBA
```

Domyślne wartości (gdy brak bloku):
- `diffuse  = (0.8, 0.8, 0.8, 1.0)`
- `ambient  = (0.2, 0.2, 0.2, 1.0)`
- `specular = (0.0, 0.0, 0.0, 1.0)`

Blok jest poprzedzony znacznikiem `uint8` (`0` = brak, `!0` = blok obecny) w rekordach MESH i LINE.

---

## 10. Catalog chunkowy — zestawienie

| FourCC  | Stała C++        | Wersja min | Opis                                           |
|---------|------------------|------------|------------------------------------------------|
| `STRS`  | `kChunkStrs`     | v4         | Tablica shared strings (musi być pierwsza)     |
| `INCL`  | `kChunkIncl`     | v4         | Referencje do submodułów `.eu7`                |
| `TRAK`  | `kChunkTrak`     | v4         | Tory (Track)                                   |
| `TRAC`  | `kChunkTrac`     | v4         | Przewody trakcyjne (Traction)                  |
| `PWRS`  | `kChunkPwrs`     | v4         | Źródła zasilania trakcji                       |
| `TERR`  | `kChunkTerr`     | v5         | Teren (batched triangles per sekcja)           |
| `MESH`  | `kChunkMesh`     | v4         | Kształty geometryczne (triangles/strip/fan)    |
| `LINE`  | `kChunkLine`     | v4         | Linie (lines/strip/loop)                       |
| `MODL`  | `kChunkModl`     | v4         | Instancje modeli 3D (flat list)                |
| `MEMC`  | `kChunkMemc`     | v4         | Komórki pamięci (MemCell)                      |
| `LAUN`  | `kChunkLaun`     | v4         | Wyzwalacze zdarzeń (EventLauncher)             |
| `DYNM`  | `kChunkDynm`     | v4         | Pojazdy dynamiczne (Dynamic)                   |
| `SOND`  | `kChunkSond`     | v4         | Dźwięki otoczenia (Sound)                      |
| `TRSE`  | `kChunkTrset`    | v4         | Zestawy wagonowe (Trainset)                    |
| `EVNT`  | `kChunkEvnt`     | v4         | Zdarzenia (Event)                              |
| `FINT`  | `kChunkFint`     | v4         | Licznik obiektów first-init                    |
| `PLAC`  | `kChunkPlac`     | v4         | Parametry umieszczenia include                 |
| `PIDX`  | `kChunkPidx`     | **v7**     | Indeks sekcji 1 km → offset w PACK            |
| `PACK`  | `kChunkPack`     | **v7**     | Strumień modeli per sekcja 1 km               |
| `PROT`  | `kChunkProt`     | **v8**     | Prototypy modeli (współdzielone definicje)     |

---

## 11. STRS — tablica łańcuchów

Wszystkie łańcuchy tekstu w pliku są przechowywane w jednej tablicy i referowane przez `string_id` (indeks `uint32`). Specjalny indeks `0xFFFFFFFF` oznacza pusty string.

```
uint32   count
Powtórzone count razy:
    uint32   length (w bajtach, bez null-terminatora)
    char[]   data (length bajtów UTF-8/ASCII, bez null)
```

Każdy kolejny string ma numer indeksu 0, 1, 2, …

---

## 12. INCL — referencje do podmodułów

Przechowuje listę `include` wskazujących na inne pliki `.eu7` — submoduły scenerii.

```
uint32   count
Powtórzone count razy:
    uint32      source_line        — numer linii w pliku tekstowym źródłowym
    string_id   source_path        — ścieżka do pliku tekstowego (.scm/.inc)
    string_id   binary_path        — ścieżka do pliku .eu7
    uint32      param_count
    string_id[] parameters         — param_count × string_id (parametry include)
    [v6+]       TransformContext   site_transform  (nieobecne w v4/v5)
```

`site_transform` przechowuje pełny kontekst transformacji miejsca osadzenia (origin/scale/rotation) i jest używany do złożenia transformacji przy nakładaniu submodułu.

---

## 13. TRAK — tory

```
uint32   count
Powtórzone count razy:
    SlimNode    node
    uint8       track_type         — patrz enum Eu7TrackType
    uint8       category           — patrz enum Eu7TrackCategory
    float32     length             [m]
    float32     track_width        [m]
    float32     friction
    float32     sound_distance
    int32       quality_flag
    int32       damage_flag
    uint8       environment        — wartość enum + 1 (Unknown = 0, Flat = 1, ...)
    uint8       has_visibility     — 0 = brak, ≠0 = blok widoczności następuje
    [jeśli has_visibility]:
        string_id  material1
        float32    tex_length
        string_id  material2
        float32    tex_height1
        float32    tex_width
        float32    tex_slope
    uint32      path_count         (maks. 65536)
    Powtórzone path_count razy:
        Vec3      p_start
        f64_disk  roll_start        [stopnie]
        Vec3      cp_out             — punkt kontrolny wyjściowy Béziera
        Vec3      cp_in              — punkt kontrolny wejściowy Béziera
        Vec3      p_end
        f64_disk  roll_end
        f64_disk  radius             [m], 0 = prosta
    uint32      tail_count          (maks. 256)
    Powtórzone tail_count razy:
        uint8     code               — 1–18 = predefiniowane kw., 255 = custom
        [jeśli code==255]: string_id  custom_key
        string_id  value
```

### Enum Eu7TrackType (uint8)

| Wartość | Nazwa       |
|---------|-------------|
| 0       | Unknown     |
| 1       | Normal      |
| 2       | Switch      |
| 3       | Table       |
| 4       | Cross       |
| 5       | Tributary   |

### Enum Eu7TrackCategory (uint8)

| Wartość | Nazwa |
|---------|-------|
| 1       | Rail  |
| 2       | Road  |
| 4       | Water |

### Enum Eu7TrackEnvironment (uint8 w pliku = enum + 1)

| Wartość w pliku | Enum  | Nazwa     |
|-----------------|-------|-----------|
| 0               | -1    | Unknown   |
| 1               | 0     | Flat      |
| 2               | 1     | Mountains |
| 3               | 2     | Canyon    |
| 4               | 3     | Tunnel    |
| 5               | 4     | Bridge    |
| 6               | 5     | Bank      |

### Kody tail keywords (uint8)

| Kod | Słowo kluczowe |
|-----|----------------|
| 1   | `event0`       |
| 2   | `eventall0`    |
| 3   | `event1`       |
| 4   | `eventall1`    |
| 5   | `event2`       |
| 6   | `eventall2`    |
| 7   | `velocity`     |
| 8   | `isolated`     |
| 9   | `overhead`     |
| 10  | `vradius`      |
| 11  | `railprofile`  |
| 12  | `trackbed`     |
| 13  | `friction`     |
| 14  | `fouling1`     |
| 15  | `fouling2`     |
| 16  | `sleepermodel` |
| 17  | `angle1`       |
| 18  | `angle2`       |
| 255 | *custom*       |

---

## 14. TRAC — przewody trakcyjne

```
uint32   count
Powtórzone count razy:
    SlimNode    node
    string_id   power_supply_name
    uint8       material           — Eu7TractionWireMaterial (0=None,1=Copper,2=Aluminium)
    float32     nominal_voltage    [V]
    float32     max_current        [A]
    float32     resistivity_ohm_per_m
    f64_disk    resistivity_legacy
    string_id   material_raw       — oryginalny string materiału ("cu", "al", ...)
    float32     wire_thickness     [m]
    int32       damage_flag
    Vec3        wire_p1
    Vec3        wire_p2
    Vec3        wire_p3
    Vec3        wire_p4
    f64_disk    min_height         [m]
    f64_disk    segment_length     [m]
    int32       wire_count
    float32     wire_offset
    uint8       has_parallel       — 0 = brak, ≠0 = nazwa równoległego sekcji
    [jeśli has_parallel]:
        string_id  parallel_name
```

---

## 15. PWRS — źródła zasilania trakcji

```
uint32   count
Powtórzone count razy:
    SlimNode    node
    Vec3        position
    float32     nominal_voltage    [V]
    float32     voltage_frequency  [Hz]
    f64_disk    internal_resistance_legacy
    float32     internal_resistance  [Ω]
    float32     max_output_current   [A]
    float32     fast_fuse_timeout    [s]
    float32     fast_fuse_repetition [s]
    float32     slow_fuse_timeout    [s]
    uint8       modifier             — Eu7PowerSourceModifier (0=None,1=Recuperation,2=Section)
```

---

## 16. TERR — teren (terrain shapes)

Chunk TERR przechowuje trójkąty terenu. Może się pojawić wielokrotnie (po jednym per materiał lub per sekcja).

```
uint8       flags                  — bitmaska:
                                     bit 0: translucent
                                     bit 1: non-default lighting (blok lighting poniżej)
                                     bit 2: batched (grupowany per sekcja 1km)
string_id   material

[jeśli bit 1 set]:
    LightingBlock lighting          (48 B)

[jeśli bit 2 set — tryb batched]:
    uint32  batch_count
    Powtórzone batch_count razy:
        int32   section_x_coord    — współrzędna X sekcji (odczytane, ale ignorowane przez runtime)
        int32   section_z_coord
        uint32  vertex_count       — musi być wielokrotnością 3
        PackedVertex[] vertices    — vertex_count wierzchołków
[else — tryb legacy]:
    uint32  count                  — liczba trójkątów
    Powtórzone count razy:
        PackedVertex[3] vertices   — dokładnie 3 wierzchołki (1 trójkąt)
```

> **Uwaga:** tryb batched (bit 2) jest charakterystyczny dla v5+. Pola `section_x_coord` i `section_z_coord` są odczytywane przez reader, ale ich wartości są ignorowane w runtime maszyna-fresh (batche traktowane jako płaskie listy kształtów).

---

## 17. MESH — kształty geometryczne

```
uint32   count
Powtórzone count razy:
    uint8       subtype            — 0=triangles, 1=triangle_strip, 2=triangle_fan
    SlimNode    node               — typ węzła wyznaczony przez subtype
    uint8       translucent        — 0/1
    string_id   material_path
    uint8       has_lighting       — 0/1
    [jeśli has_lighting]:
        LightingBlock lighting
    Vec3        origin             — lokalny punkt odniesienia wierzchołków
    uint32      vertex_count
    PackedVertex[] vertices
```

Nazwa węzła (`node.node_type`) wyznaczona przez subtype: `"triangles"`, `"triangle_strip"`, `"triangle_fan"`.

---

## 18. LINE — linie

```
uint32   count
Powtórzone count razy:
    uint8       subtype            — 0=lines, 1=line_strip, 2=line_loop
    SlimNode    node
    uint8       has_lighting       — 0/1
    [jeśli has_lighting]:
        LightingBlock lighting
    float32     line_width
    Vec3        origin
    uint32      vertex_count
    Powtórzone vertex_count razy:
        Vec3    position           — TYLKO pozycja (brak normalnych/UV)
```

---

## 19. MODL — instancje modeli 3D

Używane w submodułach include. W plikach głównych z PACK chunk, MODL z podmodułów jest zazwyczaj pomijane (sterowane flagą `pack_scenery_active()`).

```
uint32   count
Powtórzone count razy:
    SlimNode    node
    uint8       is_terrain         — 0/1
    uint8       transition         — 0/1 (animowany przejazd)
    Vec3        location           — world-space, 3×float32
    Vec3        angles             — kąty Eulera (stopnie), 3×float32
    Vec3        scale              — skala XYZ, domyślnie (1,1,1)
    string_id   model_file         — ścieżka do pliku .e3d
    string_id   texture_file       — override tekstury (opcjonalnie pusty)
    uint32      light_count
    float32[]   light_states       — light_count wartości
    uint32      color_count
    uint32[]    light_colors       — color_count kolorów RGBA packed
```

### Aplikacja w runtime (maszyna-fresh)

Rekord MODL/PACK nie trafia przez parser tekstowy SCM. Ścieżka **instancingu GPU** (nie per-model drip):

1. `preload_unique_pack_meshes()` — jeden `GetModel()` na unikalny plik `.e3d` w całej sekcji
2. Dla każdego rekordu: `TAnimModel` + `LoadEu7()` (cache hit → szybkie `Init()`)
3. `update_instanceable_flag()` — modele bez świateł/animacji → `m_instanceable = true`
4. `Region->insert()` → `m_instancebuckets_opaque` → renderer: **`Render_Instanced()`**

PACK **nie** rejestruje instancji w `Instances` / `Hierarchy` (anonimowa sceneria, bez eventów po nazwie).

Deduplikacja meshów: **`TModelsManager`** (współdzielony `TModel3d`). GPU instancing: **`opengl33_renderer::Render_Instanced`** — jeden draw na submodel × N instancji tego samego mesha.

---

## 20. MEMC — komórki pamięci

```
uint32   count
Powtórzone count razy:
    SlimNode    node
    string_id   text
    f64_disk    value1
    f64_disk    value2
    string_id   track_name         — 0xFFFFFFFF jeśli brak
```

---

## 21. LAUN — wyzwalacze zdarzeń

```
uint32   count
Powtórzone count razy:
    SlimNode    node
    Vec3        location
    f64_disk    radius_squared     [m²]
    string_id   activation_key_raw
    int32       activation_key
    f64_disk    delta_time         [s], -1 = bez limitu
    string_id   event1_name
    string_id   event2_name
    int32       launch_hour        — -1 = bez warunku godzinowego
    int32       launch_minute
    uint8       has_condition      — 0/1
    [jeśli has_condition]:
        string_id  memcell_name
        string_id  compare_text
        f64_disk   compare_value1
        f64_disk   compare_value2
        int32      check_mask
    uint8       train_triggered    — 0/1
```

---

## 22. DYNM — pojazdy dynamiczne

```
uint32   count
Powtórzone count razy:
    SlimNode    node
    string_id   data_folder
    string_id   skin_file
    string_id   mmd_file
    string_id   track_name
    string_id   driver_type
    string_id   load_type
    string_id   coupling_params
    string_id   coupling_raw       — tekstowa reprezentacja sprzęgu (np. "3.automat")
    f64_disk    offset             [m od początku toru], -1 = auto
    int32       coupling           — wartość sprzęgu (0-3)
    int32       load_count
    float32     velocity           [km/h]
    uint8       has_destination    — 0/1
    [jeśli has_destination]:
        string_id  destination
    uint8       has_trainset_index — 0/1
    [jeśli has_trainset_index]:
        uint32     trainset_index
```

---

## 23. SOND — dźwięki otoczenia

```
uint32   count
Powtórzone count razy:
    SlimNode    node
    Vec3        location
    string_id   wav_file
```

---

## 24. TRSE — zestawy wagonowe

```
uint32   count
Powtórzone count razy:
    string_id   name
    string_id   track
    float32     offset             [m]
    float32     velocity           [km/h]
    uint32      assignment_count
    Powtórzone assignment_count razy:
        string_id  key
        string_id  value
    uint32      vehicle_count
    uint32[]    vehicle_indices    — indeksy do tablicy DYNM w module
    uint32      coupling_count
    int32[]     couplings
    uint32      driver_index       — indeks pojazdu-prowadzącego
```

---

## 25. EVNT — zdarzenia

```
uint32   count
Powtórzone count razy:
    string_id   name
    uint8       type               — Eu7EventType (patrz tabela)
    f64_disk    delay              [s]
    uint32      target_count
    string_id[] targets
    f64_disk    delay_random       [s]
    f64_disk    delay_departure    [s]
    uint8       ignored            — 0/1
    uint8       passive            — 0/1
    uint32      payload_count
    Powtórzone payload_count razy:
        string_id  key
        string_id  value
```

### Enum Eu7EventType (uint8)

| Wartość | Nazwa         |
|---------|---------------|
| 0       | AddValues     |
| 1       | UpdateValues  |
| 2       | CopyValues    |
| 3       | GetValues     |
| 4       | PutValues     |
| 5       | Whois         |
| 6       | LogValues     |
| 7       | Multiple      |
| 8       | Switch        |
| 9       | TrackVel      |
| 10      | Sound         |
| 11      | Texture       |
| 12      | Animation     |
| 13      | Lights        |
| 14      | Voltage       |
| 15      | Visible       |
| 16      | Friction      |
| 17      | Message       |
| 18      | Unknown       |

---

## 26. FINT — licznik first-init

Prosty licznik — ile obiektów wymaga obsługi pierwszego inicjowania.

```
uint32   first_init_count
```

---

## 27. PLAC — parametry umieszczenia include

Mapowanie parametrów wywołania include na składowe transformacji.

```
uint8   origin_x_param      — numer parametru dla origin.x (0 = brak)
uint8   origin_y_param      — numer parametru dla origin.y
uint8   origin_z_param      — numer parametru dla origin.z
uint8   rotation_y_param    — numer parametru dla rotation.y
```

Gdy wszystkie cztery pola są `0`, plik nie eksponuje parametryzowanego umieszczenia.

---

## 28. PIDX — indeks sekcji PACK (v7+)

Katalog mapujący sekcje 1 km × 1 km na offsety w payloadzie chunku PACK.

```
uint32   count
Powtórzone count razy:
    uint16   row            — indeks wiersza sekcji (siatka regionu)
    uint16   column         — indeks kolumny sekcji
    uint32   model_count    — liczba modeli w tej sekcji
    uint64   pack_offset    — offset w bajtach od początku payloadu PACK
```

**Uwagi:**
- `pack_offset` jest **względny** od początku payloadu PACK (nie od początku pliku).
- Sekcje bez modeli mogą mieć `model_count = 0` i być pominięte w PIDX.
- Indeksowanie sekcji: `row/column` odpowiadają siatce `kRegionSideSectionCount × kRegionSideSectionCount`, centrum siatki to `(0, 0)` w world-space.
- Runtime oblicza absolutny offset jako: `pack_payload_offset + entry.pack_offset`.

---

## 29. PACK — instancje modeli per sekcja 1 km (v7+)

Payload PACK jest **zbiorem sekcji** ustawionych sekwencyjnie wg offsetów z PIDX. Chunk PACK jest pomijany przy pełnym wczytaniu — reader zapamiętuje jedynie `pack_payload_offset`.

### Format sekcji — v7 (domyślny)

Każda sekcja to `model_count` rekordów `RuntimeModelInstance` (identyczny format jak rekordy MODL):

```
Powtórzone model_count razy:
    (pełny rekord RuntimeModelInstance — patrz sekcja 19 MODL)
```

Transformacja węzła (`node.transform`) jest **zerowana** po wczytaniu — dane w PACK są zawsze w world-space (transform był bake'owany przy generacji).

### Format sekcji — v8 (`kPackSectionFormatV8 = 1`)

```
uint8    section_format    — musi być = 1
uint32   solo_count        — modele pełne (bez prototypu)
uint32   inst_count        — instancje odwołujące się do PROT

Powtórzone solo_count razy:
    (pełny rekord RuntimeModelInstance, transformacja zerowana)

Powtórzone inst_count razy:
    uint32   proto_id       — indeks do tablicy PROT
    Vec3     location
    Vec3     angles
    Vec3     scale
    string_id  name
```

Instancje v8 są rozwijane przez `expand_prototype_instance()` łącząc dane z PROT z lokalizacją instancji.

### Format sekcji — v9 UMES (`kPackSectionFormatV9 = 2`)

Rozszerzenie v8/v7: przed payloadem modeli zapisana jest deduplikowana lista ścieżek mesh (`model_file`).

```
uint8    section_format    — musi być = 2
uint32   solo_count        — modele pełne (bez prototypu)
uint32   inst_count        — instancje odwołujące się do PROT (0 w bake v7)
uint32   unique_mesh_count
uint32[unique_mesh_count] string_id   — model_file (posortowane, unikalne; pomija puste/"notload")

Powtórzone solo_count razy:
    (pełny rekord RuntimeModelInstance, transformacja zerowana)

Powtórzone inst_count razy (gdy inst_count > 0):
    uint32   proto_id
    Vec3     location
    Vec3     angles
    Vec3     scale
    string_id  name
```

Sekcje v7 bez nagłówka (flat) pozostają czytelne — brak UMES, runtime skanuje instancje.

Runtime używa UMES do cold preload meshów zamiast liniowego skanowania wszystkich instancji w sekcji.

### Format sekcji — v10 CHNK (`kPackSectionFormatV10 = 3`)

Rozszerzenie v9: po UMES tabela sub-chunków (domyślnie 512 modeli/chunk przy bake).

```
uint8    section_format    — musi być = 3
uint32   solo_count
uint32   inst_count
uint32   unique_mesh_count
uint32[unique_mesh_count] string_id

uint32   chunk_count
Powtórzone chunk_count razy:
    uint32   chunk_model_count
    uint32   chunk_byte_offset   — offset od początku sekcji (względem PIDX pack_offset)

[payload chunków — solo modele, chunk_model_count rekordów każdy]
```

Runtime: worker czyta `read_pack_section_chunk_load(row, col, chunk_index)` z O(1) seek.
Sekcje v9 (format=2) i v7 flat pozostają kompatybilne (`chunk_count` implicit = 1).

### Format sekcji — v11 UTEX (`kPackSectionFormatV11 = 4`)

Rozszerzenie v10: po UMES deduplikowana lista ścieżek tekstur (`texture_file`), posortowana po częstotliwości.

```
uint8    section_format    — musi być = 4
uint32   solo_count
uint32   inst_count
uint32   unique_mesh_count
uint32[unique_mesh_count] string_id

uint32   unique_texture_count
uint32[unique_texture_count] string_id   — texture_file (pomija none/make:/@/#/.e3d)

uint32   chunk_count
Powtórzone chunk_count razy:
    uint32   chunk_model_count
    uint32   chunk_byte_offset

[payload chunków]
```

Runtime używa UTEX do prefetch/warm tekstur bez skanowania wszystkich instancji.

### Format sekcji — v12 PROT+INST (`kPackSectionFormatV12 = 5`)

Rozszerzenie v11: sekcja dzieli modele na **solo** (pełny MODL) i **inst** (kompaktowy rekord odwołujący się do globalnego chunku PROT). Chunki zawierają mix solo+inst; w każdym chunku najpierw solo, potem inst.

```
uint8    section_format    — musi być = 5
uint32   solo_count
uint32   inst_count
uint32   unique_mesh_count
uint32[unique_mesh_count] string_id

uint32   unique_texture_count
uint32[unique_texture_count] string_id

uint32   chunk_count
Powtórzone chunk_count razy:
    uint32   chunk_solo_count
    uint32   chunk_inst_count
    uint32   chunk_byte_offset

[payload chunków — solo_count pełnych MODL + inst_count rekordów inst rozłożonych po chunkach]
```

Rekord instancji (jak v8):

```
uint32   proto_id
Vec3     location
Vec3     angles
Vec3     scale
string_id  name
```

Bake v12 emituje chunk **PROT** przed **PACK** (wersja pliku 8). Runtime rozwija inst przez `expand_prototype_instance()` i `module.model_prototypes`.

Sekcje v11–v7 pozostają czytelne (fallback v12→v11→v10→flat).

---

## 30. PROT — prototypy modeli (v8+)

Wspólne definicje modeli (bez informacji o lokalizacji instancji). Muszą być obecne przed odczytaniem sekcji PACK v8.

```
uint32   count
Powtórzone count razy:
    SlimNode    node              — name/range/bounds jak MODL, ale BEZ location/angles/scale
    uint8       is_terrain
    uint8       transition
    string_id   model_file
    string_id   texture_file
    uint32      light_count
    float32[]   light_states
    uint32      color_count
    uint32[]    light_colors
```

Różnica względem MODL: brak pól `location`, `angles`, `scale` — te są zapisane per instancja w PACK.

---

## 31. Semantyka strumieniowania PACK w runtime

Gdy root scenariusz zawiera chunk PACK, silnik maszyna-fresh uruchamia asynchroniczny **sekcja-stream**:

### Parametry strumienia

| Parametr                   | Wartość          | Opis                                               |
|----------------------------|------------------|----------------------------------------------------|
| `kSectionSizeM`            | 1000 m           | Rozmiar sekcji                                     |
| `kInitialBootstrapRadius`  | 3 sekcje         | Promień wstępnego ładowania wokół pozycji startowej|
| `kStreamRadius`            | 9 sekcji         | Bieżący promień streamingu                         |
| `kMovementLookahead`       | 4 sekcje         | Prefetch w kierunku jazdy                          |
| `kMaxInFlightSections`     | 72               | Maks. sekcji w toku (backpressure)                 |
| `kMaxReadySections`        | 36               | Maks. gotowych sekcji w kolejce                    |
| `kBootstrapTimeoutMs`      | 120 000 ms       | Timeout bootstrapu (tylko ścieżka blokująca)      |

W **jazdzie** (`driver_mode`): **jedna sekcja PACK na klatkę** — `drain_section_stream()` → `apply_pending_section()` (cała sekcja naraz, bez fałszywych budżetów ms per model).

### Dysk vs wątek główny

| Etap | Wątek | API |
|------|-------|-----|
| Deserialize MODL z `.eu7` | Worker | `read_pack_section()` |
| Page cache `.e3d` | Worker | `prefetch_pack_models()` |
| `GetModel()` unikalnych meshów + instancje | **Main** | `insert_eu7_pack_models()` |
| GPU draw | **Main** | `Render_Instanced()` dla `m_instanceable` |

`TModel3d::LoadFromFile()` + `GfxRenderer` — **tylko main**. W sekcji: najpierw wszystkie unikalne meshe, potem tysiące lekkich `TAnimModel` trafiających do bucketów instancingu.

### Przepływ ładowania

```
1. read_module() → PIDX w RAM, PACK jako offset (seek)
2. init_section_stream() → worker pool
3. preload_section_stream() [loader, blokująco] → bootstrap + drain sekcji
4. W jazdzie (is_ready):
   kick_section_stream_bootstrap()  — enqueue, bez spin-wait
   drain_section_stream()           — jedna gotowa sekcja → apply_pending_section()
   update_section_stream(camera)
```

### Wątki robocze

Po `read_pack_section()`: `prefetch_pack_models()` → `PackSectionReady` w kolejce.

### Drain (wątek główny)

`apply_pending_section()` — **cała sekcja** w jednym `insert_eu7_pack_models()`:
1. `preload_unique_pack_meshes` — cold load tylko dla nowych `.e3d`
2. Pętla instancji → `Region->insert` → GPU instancing bucket

Jedna sekcja na wywołanie `drain_section_stream()` (jedna klatka = jedna sekcja, nie jeden model).

### Diagnostyka (`eu7_load_stats`)

| Pole | Znaczenie |
|------|-----------|
| `model_ms` / `models` | Czas i liczba `TAnimModel` z PACK |
| `pack_sections_loaded` / `pack_models` | Sekcje / instancje ze streamingu |

Overlay renderera: `inst-pool:` — ile instancji zakwalifikowanych do `Render_Instanced`.

### Pliki implementacji (runtime)

| Temat | Plik |
|-------|------|
| Streaming sekcji | `scene/eu7/eu7_section_stream.cpp` |
| Prefetch dysku | `scene/eu7/eu7_model_prefetch.cpp` |
| Instancje PACK | `simulation/simulationstateserializer.cpp` |
| `LoadEu7` + `m_instanceable` | `model/AnimModel.cpp` |
| `Render_Instanced` | `rendering/opengl33renderer.cpp` |
| Cache meshów | `model/MdlMngr.cpp` |

---

## 32. Historia wersji

| Wersja | Zmiany                                                                                           |
|--------|--------------------------------------------------------------------------------------------------|
| v1     | Format legacy — szczegóły nieokreślone, nieobsługiwany                                          |
| v2     | Skalary i Vec3 jako float64 (8 B) na dysku                                                       |
| v3     | Skalary i Vec3 jako float32 (4 B) na dysku — `writeF64()` → rzutuje do float32                 |
| v4     | **Slim node** (flagowany nagłówek węzła) + **packed vertex** (snorm16 normy + half16 UV)        |
| v5     | Chunk **TERR** z trybem batched (terrain per sekcja 1 km); TERR może być w osobnym pliku `.eu7` |
| v6     | Chunk INCL rozszerzony o **site transform** (pełny TransformContext per include)                 |
| v7     | Chunki **PIDX + PACK** — modele scenerii podzielone na kafelki 1 km, asynchroniczny streaming   |
| v8     | Chunk **PROT** (prototypy) + nowy format sekcji PACK z `solo_count` + `inst_count`              |
| v9     | **PROT v9** (resolved_texture, pack_flags, baked ranges) + **PACK v13** (mesh/tex indices, cell_id) |

---

## 33. Ścieżki plików i konwencje nazewnictwa

### Rozszerzenie

Pliki EU7B używają rozszerzenia `.eu7`. Identyfikacja pliku odbywa się przez:
1. Sprawdzenie rozszerzenia (`isSceneBinaryPath()`)
2. Sondowanie magic bytes `EU7B` na pozycji 0 (`probeSceneBinaryMagic()`)

### Konwersja ścieżek source → binary

- `*.scm` → `*.eu7` (te same katalog i stem)
- `*.inc` → `*.eu7`
- `*.scn` → `*.eu7`
- `*.sbt` → terrain `*.eu7` (odpowiednik binarny terenu)

Funkcja `binary_path()` w loaderze zamienia rozszerzenie przy tym samym stemie.

### Warianty pliku terenu

Plik `.eu7` może zawierać chunk TERR (teren). Loader sprawdza obecność TERR przez `probe_terrain_file()` (pełne wczytanie nagłówków).

Funkcja `is_scenario_terrain()` sprawdza czy obok scenariusza `<stem>.scn` istnieje `<stem>.eu7` z chunkiem TERR.

### Pliki include

Submoduły include są wczytywane jeden raz per ścieżka (deduplication przez `is_module_loaded()`). Ścieżka pliku `.eu7` includowanego jest wyznaczana przez `include_eu7_path()` — ten sam katalog co plik SCM/INC.

---

*Dokumentacja wygenerowana na podstawie kodu źródłowego; stan na 11 czerwca 2026.*
