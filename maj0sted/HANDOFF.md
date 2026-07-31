# HANDOFF — maj0sted

Stan prac i punkt zaczepienia dla kolejnej sesji (np. po przejściu na `claude` CLI).
Kod jest samoopisowy; ten plik to skrót "gdzie jesteśmy i co dalej".

## Czym jest projekt
Statyczna biblioteka **C++20** (DDD) do edytora torowiska. Czysty model domenowy,
bez zależności zewnętrznych. Namespace: `maj0sted::domain`.

**Uwaga (sandbox):** na tej maszynie wyłączone są nieuprzywilejowane user
namespaces, więc domyślny sandbox komend pada w preflight (komenda „nie zwraca
statusu”). Uruchamiaj polecenia powłoki **z wyłączonym sandboxem** (w Cursorze:
`required_permissions: ["all"]`). Wtedy `git`, `g++`, `node` działają normalnie.

**Uwaga:** `cmake` NIE jest zainstalowany. Weryfikacja odbywa się przez `g++ 14.2`.
Toolchain WASM: `emsdk` jest w `~/emsdk` (`source ~/emsdk/emsdk_env.sh` dodaje `em++`).

## Jak weryfikować (bez cmake)
```bash
cd /home/maj00r/Projects/maj0sted
FLAGS="-std=c++20 -Iinclude -Wall -Wextra -Wpedantic -Wconversion -Wshadow"
SRC="src/domain/*.cpp src/io/*.cpp src/render/*.cpp src/web/solve.cpp src/web/tile_cache.cpp src/app/editor_document.cpp"
for t in map_project niweleta plan_alignment parallelism fitting serialization render web tile_cache document; do
  g++ $FLAGS -Itests $SRC tests/test_$t.cpp -o /tmp/t_$t && /tmp/t_$t | tail -1
done
```
Wszystkie testy przechodzą, kompilacja bez ostrzeżeń. Oszczędnie: kompiluj tylko
zmieniony test z minimalnym zestawem źródeł (np. `tile_cache` wymaga tylko
`src/web/tile_cache.cpp`; `document` — `src/domain/*.cpp src/io/project_serializer.cpp
src/app/editor_document.cpp`). Szybki syntax-check: `g++ $FLAGS -fsyntax-only ...`.

Budowa WASM: `source ~/emsdk/emsdk_env.sh && cd web && ./build.sh`
(produkuje `web/maj0sted.js` + `.wasm`, oba w `.gitignore`).

## Model — co już jest (✓ = zaimplementowane + testy)

### Value Objects (`include/maj0sted/domain/value_objects/`)
- `Crs` — układ EPSG, domyślnie **EPSG:2180** (metry) ✓
- `CartesianPosition` — (x=easting, y=northing), metry, double ✓
- `Length` (≥0), `Radius` (>0) ✓
- `Azimuth` — radiany, **zgodnie z ruchem wskazówek zegara od północy (+y)**, znormalizowany [0,2π) ✓
- `NiweletaId`, `StraightId` — silne tożsamości ✓
- `TrackOffset` — dystans (>0) + `Side::Left/Right` ✓

### Agregaty
- `MapProject` — **projekt**: trzyma `Crs` (domyślnie EPSG:2180) + kolekcję niwelet
  (`add_niweleta`, `niwelety()`) ✓
- `Niweleta` (`domain/niweleta.hpp`) — id, nazwa, plan + profil. **Sama nadaje `StraightId`**:
  `add_straight(start,end,joint) -> StraightId`; `add_plan_element` mintuje id anonimowej prostej
  (i podbija licznik przy id wczytanym z pliku). `lengths_consistent()`.
  **`apply_fit(FitResult)`** — wpina wynik fitowania w plan: podmienia sąsiednie proste
  (entry, exit) na `entry → curve... → exit` stykami Tangent; proste zachowują id ✓

### Geometria planu (`domain/geometry/`)
- `PlanElement = variant<Straight, CircularArc, TransitionCurve>`
  - `Straight` — **zakotwiczony w XY (start/end)**, azymut i długość liczone; opcjonalny `StraightId`.
    Zasada: prosta trzyma linię nośną (pozycja+azymut), zmienia się tylko długość; krzywe to wypadkowa.
  - `CircularArc` — promień + kierunek + **opcjonalna** długość
  - `TransitionCurve` (klotoida) — długość + promienie końcowe (`optional`=∞) + kierunek
- `HorizontalAlignment` — lista kierunkowa + `JointContinuity{Tangent, AzimuthBreak}`. Reguły styku ✓:
  - sąsiednie proste Tangent: muszą się stykać i mieć równy azymut
  - różny azymut prostych → wymaga `AzimuthBreak` (tylko prosta-prosta, muszą się stykać, różne azymuty)
  - sąsiednie łuki o tym samym promieniu i kierunku → zabronione
  - styki z krzywą akceptowane "pending fit"

### Geometria profilu
- `ProfileElement = variant<Grade, VerticalCurve>`; `VerticalAlignment`. (Profil na razie nietknięty.)

### Równoległość (`domain/parallelism/`)
- `TrackParallelism { StraightId origin, StraightId parallel, TrackOffset }` — **referencje wprost po StraightId**;
  niezmienniki: oba id niepuste, różne ✓
- `ParallelismService`: `mark_parallel(originStraight, parallelStraight, offset)`, `are_parallel`,
  `derive_parallel_line`, `perpendicular_distance` ✓

### Fitowanie (`domain/fitting/`)
- `FitParameters { entry, exit, radius, optional transition_length }` — klotoida symetryczna lub sam łuk
- `CompoundFitParameters { entry, exit, radius1, radius2, split_station }` — **łuk koszowy**
  (miejsce podziału = kilometraż pierwszego łuku; `δ1 = split_station/R1`)
- `FitResult { entry(przycięta), curve[], exit(przycięta), tangent_in, tangent_out, direction }`
- `FittingService`:
  - `fit_between_straights` — klotoida–łuk–klotoida (lub sam łuk), przycina obie proste ✓
  - `fit_compound` — dwa łuki różnych promieni; **obie linie prostych bez zmian**, obie proste
    doprzycięte na długości; podział ustala `δ1`. Metoda stycznej pośredniej ✓

### Serializacja (`maj0sted::io`, `include/maj0sted/io/project_serializer.hpp`)
- `serialize(MapProject) -> std::string` / `deserialize(std::string) -> MapProject` ✓
- Format tekstowy, liniowy, bez zależności; liczby przez `to_chars`/`from_chars` (dokładny round-trip,
  niezależny od locale). Round-trip jest bajtowo stabilny (test to sprawdza).
- Zapisuje: CRS + niwelety (id, nazwa, plan z elementami i stykami, profil). **NIE** zapisuje relacji
  równoległości (patrz niżej — StraightId nie są globalnie unikalne).

### Rendering: dwie szyny
`src/web/editor.cpp` rysuje **dwie szyny w rozstawie 1500 mm** (`kHalfGauge=0.75`,
±750 mm od osi) zamiast samej osi. `offset_points` przesuwa linię osi wzdłuż
normalnej per-wierzchołek (różnica centralna, normalna lewa `(-dy,dx)`),
`push_rails` emituje lewą i prawą szynę zachowując `kind` (kolor). Uchwyty edycji
(węzły/końce prostych) w JS dalej rysują się z osi (model).

### Warstwa web (WASM) — biblioteka robi wszystko, JS tylko rysuje
Zasada: **cała logika jest w libce**; `web/` to cienki renderer + przekazywanie
wejścia. `web/maj0sted_bindings.cpp` (embind) to jedyny plik zależny od emscripten.

- **Kafle podkładu WMS** (`maj0sted::web`, `web/tile_cache.hpp/.cpp`) ✓
  - `TileGrid` — **jeden stały grid kafli 100 m × 100 m** w **EPSG:2180**
    (`kCellMetres=100`, **1024 px/kafel** → 100/1024 m/px, origin (0,0), jeden
    poziom `kLevel=0`, **bez piramidy per-zoom**); `tile_at`, `bbox`,
    `tiles_for_view`, `zoom_for_resolution` (argument zoom ignorowany, sygnatury
    zachowane dla boundary). Czysta, testowana natywnie.
  - **Retry:** nieudany fetch jest ponawiany **maks. `kMaxRetries=10` razy na
    kafel** (licznik `TileImage.attempts`); po przekroczeniu limitu kafel dostaje
    status `Failed`. Pod Emscripten retry jest odroczony async
    (`emscripten_async_call`, `kRetryDelayMs=1500`), bez blokowania/rekurencji;
    z wstrzykniętym fetcherem (testy) powtarza od razu.
  - `WmsConfig` + `wms_getmap_url` — WMS **1.1.1** GetMap (BBOX = minx,miny,maxx,maxy),
    presety Geoportalu (ortofoto/topo). Liczby przez `to_chars`.
  - `TileService` — cache + **fetch**: na trafienie zwraca z cache, na pudło sam
    wykonuje GetMap. Pod `__EMSCRIPTEN__` używa **Emscripten Fetch** (`-sFETCH`);
    natywnie/w testach przez wstrzykiwany `Fetcher` (bez sieci).     Callback
    `on_ready` budzi przerysowanie w JS.
  - **Proxy WMS (`web/serve.py`):** przeglądarka pobiera kafle **same-origin**
    (`/wms?...`), a lokalny serwer pobiera je serwer-serwer z Geoportalu. Powód:
    WAF Geoportalu **zwraca 404 dla User-Agentów zawierających „Mozilla"** (czyli
    dla prawdziwych przeglądarek), a przepuszcza proste UA (`curl`), więc bezpośredni
    `fetch`/`emscripten_fetch` z przeglądarki dostaje 404. Proxy używa UA `curl/8.5.0`.
    Frontend ustawia `Module.wmsSetConfig({base_url: origin+'/wms'})` w `finishBoot`.
    Libka nadal robi fetch (`emscripten_fetch`) — tylko na `localhost`.
    Uruchamianie: `python3 web/serve.py 8080` (zamiast `http.server`).
  - **Cache na dysku:** proxy odsyła udane kafle z `Cache-Control: public,
    max-age=31536000, immutable`, więc przeglądarka trzyma je w HTTP cache na dysku —
    po odświeżeniu strony kafle są brane z cache, bez ponownego pobierania. Błędy
    (`502`) idą z `no-store` (nie cache'ują się). Uwaga: cache `TileService` w C++ jest
    tylko w pamięci WASM (ginie przy refreshu) — trwałość zapewnia właśnie HTTP cache.
  - **Retry w proxy:** WAF Geoportalu bywa niestabilny (sporadyczne 404/reset nawet dla
    UA `curl`), więc proxy ponawia pobranie `WMS_PROXY_RETRIES=6` razy zanim zwróci 502;
    do tego dochodzi client-side retry libki (10×).
- **Trwałość projektu** (`maj0sted::app`, `app/editor_document.hpp/.cpp`) ✓
  - `EditorDocument` = model GUI (niezależne proste + parametry fitów) — bezstratny.
  - `serialize_document`/`deserialize_document` — **reużywa `io::serialize`** (osadza
    prawdziwy projekt maj0sted) + sekcja `editor` z prostymi i parametrami fitów.
  - `to_map_project`/`from_map_project` — konwersja do/z `MapProject` (best-effort,
    nie rzuca).
  - `save_project`/`load_project` — **I/O plikowe w libce** (`std::fstream` +
    `std::filesystem`), domyślna ścieżka `/data/maj0sted/project.m0s`.
- **Fit nieudany → „brak" (decyzja w libce):** `solve_project` (editor) zwraca per
  niweleta `polylines` **oraz `applied_fits`** — autorytatywną listę fitów, które
  faktycznie zaszły. Fit, którego nie da się policzyć (zły parametr/geometria, punkty
  styczne poza odcinkami prostych), jest **odrzucany w libce** i nie ma go w `applied_fits`.
  Frontend tylko lustrzanie przyjmuje tę listę (bez własnej reguły o porażce).
- **Boundary (embind, cienki):** `solveProject`, `wmsTilesForView`,
  `wmsZoomForResolution`, `wmsTileBytes`, `wmsSetOnReady`, `wmsSetConfig`,
  `saveProject`, `loadDefaultProject`.
- **Frontend (`web/index.html`):** rysuje podkład WMS pod geometrią (georeferencja
  EPSG:2180, trzyma się przy pan/zoom); `Ctrl+S` → `saveProject` + `FS.syncfs`;
  na starcie montuje **IDBFS** w `/data`, `syncfs(true)` i `loadDefaultProject`
  (fallback: demo). Flow budowy: prosta → krzywa (czeka jako „pending”, rysowana
  przerywaną) → kolejna prosta kotwiczy ją z obu stron i **dopiero wtedy** liczony
  jest fit.

## Konwencje geometryczne
- EPSG:2180, jednostka = metr; kierunek jazdy `d = (sin az, cos az)`; normalna lewa `(-cos az, sin az)`.
- Tolerancje: pozycja `1e-6 m`, azymut `1e-9 rad`.

## Znane ograniczenia
- **`StraightId` są lokalne per-niweleta** (licznik startuje od 1 w każdej niwelecie), więc NIE są
  globalnie unikalne. `TrackParallelism` referuje proste po samym `StraightId` — działa dla luźnych
  obiektów, ale w projekcie z wieloma niweletami trzeba by referować `(NiweletaId, StraightId)`.
  Do przemyślenia, jeśli równoległość ma być trwała/serializowana.

## Następne kroki (do decyzji / do zrobienia)
1. **Klotoidy w łuku koszowym** (między łukami i/lub na końcach). (NIE zrobione.)
2. Asymetryczne klotoidy w `fit_between_straights` (teraz tylko symetryczne). (NIE zrobione.)
3. **Rejestr relacji równoległości** + poprawka referencji na `(NiweletaId, StraightId)` i ich serializacja. (NIE.)
4. **Solver** ustawiający prostą równoległą na `derive_parallel_line` i przeliczający resztę. (NIE.)
5. Instalacja `cmake`/`ninja` i budowa przez CMake (na razie tylko `g++`).

## Preferencje pracy (ustalone z użytkownikiem)
- **Nie edytować `README.md`** (utrzymuje go użytkownik).
- Robić **tylko to, o co proszono** — bez dokładania funkcji z własnej inicjatywy.
- **Oszczędzać CPU**: kompilować tylko zmieniony test, `-fsyntax-only` do szybkiej walidacji,
  pełne testy na koniec/na żądanie; bez procesów w tle, przeglądarki i podagentów.
- Język domeny: **niweleta** = jednolity odcinek toru złożony z wielu krzywych w planie i profilu.

## Git
Repozytorium zainicjalizowane; pliki dodane do stage'a, **brak commitów** (commit dopiero na wyraźną prośbę).
