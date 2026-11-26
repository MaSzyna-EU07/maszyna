# Nowy format świata MaSzyny

Stan na 26.11.2026. Dokument projektowy: opisuje plan, nie stan kodu.

## 1. Cel i założenia

Nowy binarny format świata (`.eu07`) ma pozwolić na streaming scenerii kaflami, zmniejszyć zużycie RAM i VRAM oraz zmieścić całą Polskę w jednym układzie współrzędnych. Docelowo edytor zapisuje tylko ten format. W okresie przejściowym zapisuje oba: `.scn` i `.eu07`.

Założenia:

- **Streaming zamiast wczytywania wszystkiego.** W pamięci jest tylko to, co widać, plus zapas. Rozmiar scenerii nie zależy od RAM-u.
- **Dane gotowe dla GPU.** Bloki kafla kopiuje się wprost do buforów OpenGL 4.6 i Vulkan 1.4, bez konwersji wierzchołek po wierzchołku.
- **Jeden wspólny układ.** Scenerie z georeferencją wpadają we wspólny płaski układ wokół Warszawy, zdefiniowany na elipsoidzie WGS84. Można je łączyć, a zdjęcia lotnicze z dowolnego kraju przelicza się do niego przy imporcie.
- **Krzywizna Ziemi tylko w obrazie.** Dane zostają płaskie, a krzywiznę liczy shader.
- **Zgodność.** `.scn` jest czytany zawsze, tak jak dziś (zasady z `AGENTS.md`). Nowa funkcja ma przełącznik w `eu07.ini` i nie wymaga zmian w SVN.

Poza zakresem: format modeli `.e3d`, tekstur i pojazdów. Zostają bez zmian, a `.eu07` odwołuje się do nich po nazwie.

## 2. Stan obecny

Świat ogranicza dziś stała siatka 500 × 500 sekcji po 1 km, a nie precyzja: pozycje są w double, a region ma ±250 km (`scene/scene.h`, `EU07_SECTIONSIZE`, `EU07_REGIONSIDESECTIONCOUNT`). Gałąź `feat/gl46-migration` daje część potrzebnych narzędzi, ale geometria sceny dalej wczytuje się w całości.

**master** (sprawdzone na `fix/render-as-26.05`):

| Element | Stan | Co z tego bierzemy |
| --- | --- | --- |
| Pozycje | `glm::dvec3` w węzłach, `world_vertex` 44 B w RAM przed uploadem | rendering względem origin węzła |
| Siatka sekcji | gęsta tablica `std::array<basic_section*, 500²>` | do zastąpienia katalogiem kafli |
| Cache `.sbt` | tylko trójkąty i linie, wczytywany w całości, bez unieważniania (`basic_region::serialize/deserialize`) | do wycofania |
| Teren `.btf`/`.txtf` | katalog bloków sekcji, CRC źródła, doczytywanie z opóźnieniem (`scene/sceneterrain.*`) | wzorzec katalogu i unieważniania |
| Heightmapa `.tch` | bloki z tagami (`CHNK`, `HGHT`…), zstd, CRC, wątki `terrain_streamer` (`scene/heightmapterrain.h`) | wzorzec bloków, zstd już w projekcie |
| Wczytywanie | `state_serializer::deserialize_begin/continue`, wątek główny, co 200 ms | miejsce na ścieżkę `.eu07` |
| Zapis edytora | łatanie `.scn` po zakresach bajtów, `export_as_text` (`scene/scenelayerssave.cpp`) | zostaje dla `.scn`; brak interfejsu zapisu |
| Georeferencja | linia `//$g` czytana tylko przez edytor (`editor_mode::read_georeference`) | przenosimy do nagłówka |

**feat/gl46-migration** (522 commity przed master, PR #216):

| Element | Stan | Co z tego bierzemy |
| --- | --- | --- |
| Wierzchołek | `gfx::basic_vertex` 48 B (float) zakodowany na sztywno w `opengl46geometrybank.cpp` i `vulkan14scene.cpp` | dodajemy drugi układ, 28 B |
| Indeksy | zawsze uint32; kształty i tory w większości bez indeksów | indeksowanie przy kompilacji kafla |
| Bufory GL | strony-megabufory 4M wierzchołków z listą wolnych miejsc (`opengl46_geometry_page`) | kafle jako wycinki stron |
| Bufory Vulkan | VMA, osobne bufory na bank, upload na kolejce graficznej, kolejka transferowa nieużywana | podalokacja i kolejka transferowa |
| Culling | Hi-Z + multi-draw indirect tylko dla instancji (`rendering/common/instancecull.*`) | rozszerzamy na geometrię statyczną |
| Precyzja | origin w double minus kamera, wpieczony w `modelview`; `instance_record` względem origin kubełka | ten sam wzorzec z origin kafla |
| Materiały | bez bindless; push descriptors w Vulkanie | bez wymagania bindless |
| Wymagania | GL 4.6 core; Vulkan 1.4 + `multiDrawIndirect`; `drawIndirectCount` opcjonalne | nie dokładamy twardych wymagań |

## 3. Układ współrzędnych

Pozycje bezwzględne zapisujemy jako int32 w jednostkach 2⁻¹⁰ m (≈0,98 mm), co daje zasięg ±2097 km. Wewnątrz kafla 1024 m geometria jest we float32 względem środka kafla, z krokiem 0,06 mm.

**Jednostka i kafel.** Kafel ma 2²⁰ jednostek, czyli 1024 m. Dzięki temu indeks kafla i pozycja w kaflu to operacje bitowe, bez dzielenia i zaokrągleń:

```latex
t = p \gg 20, \qquad l = p \mathbin{\&} \mathtt{0xFFFFF}
```

| Wielkość | Typ | Zakres | Krok |
| --- | --- | --- | --- |
| pozycja w świecie (x, y, z) | int32, 2⁻¹⁰ m | ±2097 km | 0,98 mm |
| indeks kafla (tx, tz) | int16 | ±2048 kafli | 1024 m |
| geometria w kaflu | float32 względem środka | ±512 m (z zapasem do ±1 km) | 0,06 mm przy 512 m |
| wysokość | int32, 2⁻¹⁰ m | n.p.m. | 0,98 mm |

**Układ świata.** Układ to płaskie, wiernokątne odwzorowanie stereograficzne na elipsoidzie WGS84 (ETRS89), ze środkiem w Warszawie i skalą dobraną tak, by błąd był najmniejszy w promieniu ok. 500 km. Zerem układu jest stacja Warszawa Centralna (dokładne współrzędne do ustalenia), a oś z wskazuje tam prawdziwą północ. Krańce Polski leżą do ok. 500 km od Warszawy, a Berlin ok. 520 km, więc wszystko mieści się w ±2097 km z dużym zapasem. Kierunki zostają jak w `.scn`: x rośnie na zachód, z na północ, y w górę.

*Rysunek w wersji źródłowej dokumentu: mapa Europy w układzie świata. Kwadrat zasięgu int32 (±2097 km) wokół stacji Warszawa Centralna, dla porównania dzisiejszy region ±250 km, okręgi błędu skali 500 km (±0,8 m/km) i 800 km (3 m/km). Miasta i kontury lądów (Natural Earth 1:110m) przeliczone tym samym odwzorowaniem stereograficznym (k₀ = 0,9992, kula R = 6382 km).*

Kwadrat int32 sięga od Londynu i Paryża po Moskwę, od Oslo po Ateny. Madryt leży praktycznie na jego krawędzi, a Lizbona poza nim. Okręgi pokazują, gdzie błąd skali rośnie: cała Polska mieści się w 500 km i w ±0,8 m/km.

Źródło konturów lądów: [Natural Earth, 1:110m Land](https://github.com/nvkelso/natural-earth-vector/blob/master/geojson/ne_110m_land.geojson) (domena publiczna), przeliczone tym samym odwzorowaniem co miasta.

**Scenerie z georeferencją** (linia `//$g`) wchodzą do układu przez przesunięcie i obrót wokół pionu, dopasowane w środku scenerii. Ich geometria jest w PUWG 1992, którego siatka jest w Warszawie obrócona o ok. 1,6° względem układu świata. Długości się nie zmieniają, więc pociągi jeżdżą tak samo. Pominięta różnica skali obu odwzorowań przesuwa zdjęcia lotnicze na skrajach dużej scenerii najwyżej o kilka metrów. Kompilator może ją opcjonalnie uwzględnić. **Scenerie bez georeferencji** dostają własny środek i flagę w nagłówku. Działają tak samo, tylko nie da się ich złożyć z innymi.

**Dokładność georeferencji w danych.** Linie `//$g` w pctga podają środek scenerii w pełnych kilometrach: `PUWG1992` w 199 plikach, własna siatka `EU07-424` w 18 (Całkowo, Zwierzyniec, Drawinowo, czyli scenerie fikcyjne). Kilometr to za mało, żeby zdjęcia lotnicze pasowały. Dokładny środek ustawia się w edytorze względem ortofotomapy (edytor już to umie) i zapisuje w nagłówku w metrach. Scenerie z `EU07-424` dostają własny środek, bez miejsca na mapie. Liczy się tylko `//$g` z głównego `.scn`. Include'y mają czasem własne, sprzeczne linie, np. `linia181/wielun.scm` włączany do `linia61_osobowy1.scn` przez `origin 24000 0 48000`.

**Zdjęcia lotnicze i NMT z dowolnego kraju.** Każde źródło zostaje w swoim układzie: Polska EPSG:2180, Niemcy EPSG:25833, Czechy EPSG:5514 i tak dalej. Edytor trzyma kafle zdjęć w pamięci podręcznej w układzie źródła, jak dziś siatkę 256 m w PUWG (`editor/editorOrthophoto.*`, `editor/editorNmt.*`). Do układu świata przelicza je dopiero przy nakładaniu na teren, biblioteką PROJ (licencja MIT). Dzisiejsze proste mapowanie `easting = east − x` zastępuje pełna transformacja odwzorowań.

**Rendering względem kamery.** CPU liczy `kafel − kafel_kamery` i `pozycja − pozycja_kamery` w intach, dokładnie. Do bufora klatki trafia tylko wynik zamieniony na float: przesunięcie kafla względem kamery. Shader dodaje je do lokalnej pozycji wierzchołka. To ten sam wzorzec co dziś na gałęzi gl46: origin komórki minus kamera w `modelview` i `recordorigin` kubełka w `instance_record`. Zmienia się tylko źródło originu. Na GPU nie ma double.

W logice i fizyce pozycje pojazdów i torów mogą zostać w double. Int32 jest formatem zapisu i kluczem przestrzennym. Konwersja to jedno mnożenie przez 2⁻¹⁰, dokładne w double.

## 4. Krzywizna Ziemi i skala odwzorowania

Krzywiznę liczy vertex shader, a dane zostają płaskie. Na 5 km obiekt opada o 1,7 m, na 20 km już o 27 m, więc efekt widać przy dalekich widokach, horyzoncie i światłach.

```latex
y' = y - \frac{d_x^2 + d_z^2}{2 R_{\mathrm{eff}}}, \qquad R_{\mathrm{eff}} = \frac{R}{1 - k} \approx \frac{6371\ \mathrm{km}}{0{,}87} \approx 7323\ \mathrm{km}
```

Gdzie d to odległość wierzchołka od kamery w poziomie, a k = 0,13 to typowy współczynnik refrakcji.

| Odległość od kamery | Opadnięcie (z refrakcją, k = 0,13) |
| --- | --- |
| 2 km | 0,27 m |
| 5 km | 1,7 m |
| 10 km | 6,8 m |
| 20 km | 27 m |
| 50 km | 171 m |

**Dlaczego nie w danych:**

- Na 250 km od środka kierunek grawitacji różni się o 2,25°. Fizyka i dynamika liczą w płaszczyźnie z pionem w osi y.
- Pochylenia i profile torów są liczone w płaszczyźnie. Zakrzywione dane zmieniłyby każdy spadek.
- Istniejące `.scn` są płaskie. Ich geometria musi wejść do `.eu07` bez zmian.

**Wdrożenie w rendererze:**

- Ta sama poprawka w każdym przebiegu, który liczy pozycję wierzchołka: geometria, cienie, odbicia, cząsteczki. Inaczej cienie się rozjadą.
- Culling: sfera obiektu przesunięta w dół o opadnięcie jej najbliższego punktu, z zapasem. Tak samo w shaderach GPU cullingu (`instance_cull.comp`) i dla min/max y kafla.
- Przełącznik w `eu07.ini` (roboczo `gfx.earthcurvature`), domyślnie wyłączony, bo zmienia wygląd istniejących scenerii.

**Skala odwzorowania.** Odwzorowanie stereograficzne jest wiernokątne, a jego błąd skali rośnie jednakowo we wszystkie strony od Warszawy, z kwadratem odległości. Przy skali w zerze k₀ = 0,9992 (do ustalenia) błąd mieści się w ±0,8 m/km w promieniu 500 km, czyli na całej Polsce.

```latex
k(d) \approx k_0 \left(1 + \frac{d^2}{4R^2}\right), \qquad R \approx 6382\ \mathrm{km}
```

| Miejsce | Odległość od zera | Skala | Błąd długości | Obrót siatki względem północy |
| --- | --- | --- | --- | --- |
| Warszawa | 0 km | 0,9992 | −0,8 m/km | 0° |
| Kraków | 250 km | 0,9996 | −0,4 m/km | ok. 0,8° |
| Berlin | 520 km | 1,0009 | +0,9 m/km | ok. 6° |
| Kijów | 690 km | 1,0021 | +2,1 m/km | ok. 7° |
| Paryż, dla porównania | 1370 km | 1,011 | +11 m/km | ok. 14° |

**Przykład: mapa Warszawa – Berlin.** Kąty i promienie łuków są zachowane, więc geometria toru wygląda dobrze. Cała trasa wychodzi ok. 130 m krótsza niż w rzeczywistości, a czasy jazdy zmieniają się o mniej niż 0,1%. Przy Berlinie słupki kilometrowe rozjeżdżają się z długością toru o ok. 0,9 m na kilometr. Siatka mapy jest tam obrócona względem prawdziwej północy o ok. 6°, a słońce wschodzi ok. 30 minut później niż w Warszawie. Niebo liczone z jednego punktu (dziś `latitude` w `eu07.ini`) tego nie oddaje.

**Propozycja:**

- Kafel zapisuje skalę odwzorowania, obrót siatki i współrzędne geograficzne swojego środka. Niebo (słońce, księżyc, gwiazdy) liczy się z położenia kamery. Za przełącznikiem, bo zmienia wygląd.
- Korekta długości torów nie jest potrzebna dla map do ok. 700 km od Warszawy. Zostaje jako możliwość.
- Ponad ok. 800 km od Warszawy błąd skali przekracza 3 m/km. Taka sceneria (np. do Paryża) może dostać własne odwzorowanie przez definicję w nagłówku, ale nie łączy się wtedy ze sceneriami w układzie warszawskim bez przeliczenia.

## 5. Dwie warstwy: logika i obraz

Logika symulacji jest zawsze cała w pamięci, a warstwa wizualna przychodzi kaflami. AI, eventy i pojazdy działają na całej mapie, niezależnie od tego, gdzie jest kamera. Siatki, teren i modele są potrzebne tylko w zasięgu wzroku.

Zasada podziału: wszystko, do czego odwołuje się event, AI, pojazd albo inny obiekt po nazwie, należy do logiki. Wszystko, co tylko się rysuje albo gra, należy do kafla.

| Obiekt | Logika (zawsze w pamięci) | Kafel (streamowany) | Dziś |
| --- | --- | --- | --- |
| Tor `TTrack` | ścieżka, połączenia, prędkości, eventy, izolacje | siatka szyn, podkładów, podsypki | `simulation::Paths`, siatka w `basic_cell::create_geometry` |
| Sieć trakcyjna `TTraction` | przewód jako linia dla pantografu, zasilanie (`Powergrid`) | siatka drutów i lin | `simulation::Traction`, `Powergrid` |
| Eventy, memcelle, launchery | całość | — | `simulation::Events`, `Memory` |
| Model `TAnimModel` (semafor, tarcza) | stan: światła, animacje sterowane eventami | instancja modelu do narysowania | `simulation::Instances` |
| Kształty, linie, teren z trójkątów | — | całość | komórki regionu |
| Modele statyczne (drzewa, budynki) | — | instancje | `simulation::Instances` |
| Drogi (`Roads`, `Sweeps`) | — | gotowa siatka, dziś generowana przy każdym wczytaniu | `create_geometry` po wczytaniu |
| Dźwięki | globalne i sterowane eventami | pozycyjne dźwięki otoczenia | `simulation::Sounds` |

Pojazdy i rozkłady nie należą do formatu świata. To stan scenariusza.

**Model z dwiema częściami.** Semafor musi pamiętać stan świateł, nawet gdy jego kafel nie jest wczytany. Dlatego `TAnimModel` dzieli się na lekki stan w logice i instancję w kaflu. Instancja po wczytaniu czyta stan z logiki. Tak samo tor: ścieżka zostaje, a siatka przychodzi z kaflem.

**Indeks przestrzenny logiki.** Dziś komórki regionu trzymają wskaźniki do torów, sieci i memcelli (`m_paths`, `m_traction`, `m_memorycells`). W nowym układzie zastępuje je rzadka mapa klucz kafla → lista obiektów. Nie zależy od tego, czy kafel wizualny jest wczytany.

### Pozycje w logice: przegląd przypadków

Geometria statyczna może leżeć w pamięci jako int32 (2⁻¹⁰ m) albo kafel + float bez widocznej straty. Stan ruchomy (pojazdy, odległość na torze, kamera) zostaje w double albo kafel + przesunięcie z dokładnością poniżej milimetra. Sprawdzone na `fix/render-as-26.05`; na `feat/gl46-migration` te pola są takie same.

| Obiekt | Pola dziś | W pamięci docelowo | Dlaczego |
| --- | --- | --- | --- |
| Tor `TSegment` | 7 × dvec3 (168 B) + `TTrack::m_origin` | int32: 84 B; `vA`, `vB`, `vC` jako float względem `Point1`; obliczenia w double | statyczne; łączenia sprawdzane z tolerancją 2 cm |
| Odległość na torze `TTrackFollower::fCurrentDistance` | double | bez zmian (albo float, bo ograniczona długością odcinka) | przy 0,1 km/h krok fizyki to 0,28 mm; int zamroziłby powolny ruch |
| Sieć trakcyjna `TTraction` | 5 × dvec3 (120 B) | int32: 60 B albo kafel + float | kontakt pantografu do przepisania na różnice, bo dziś odejmuje iloczyny skalarne pozycji bezwzględnych |
| Pojazd `TDynamicObject` | `vPosition`, sprzęgi, `mMatrix` (double) | double albo kafel + double | różnice między klatkami i pojazdami poniżej 1 mm |
| `TMoverParameters::Loc` | 3 × double | double | szczelina sprzęgu liczona z `Loc`: 1 mm szumu to szum sił zderzaków |
| AI `TController` | dvec3, odległości double | kafel + float | używa tylko różnic; int dałby zerowe kroki przy małej prędkości |
| Eventy, memcelle, launchery | dvec3, promień do kwadratu | int32 | dokładność metrowa |
| Węzły sceny, `bounding_area` | dvec3 + float | int32 albo kafel | culling w metrach |
| `world_vertex` | dvec3 na każdy wierzchołek | znika: dane GPU w kaflu | największy zysk pamięci |
| Kamera `TCamera` | dvec3 | double albo kafel + float | wolny lot wymaga ruchu poniżej milimetra |
| Drogi, sweepy | dvec3 | int32 | statyczne |

**Do przeróbki albo decyzji:**

- **Nierówności toru.** `track_uneven::field` liczy fale z bezwzględnych x i z. Przeniesienie scenerii do układu świata zmieniłoby fale na każdym torze. Pole musi dalej dostawać współrzędne w układzie scenerii ze `.scn`, więc `.eu07` zapisuje przesunięcie i obrót scenerii.
- **Pantograf.** Wzór kontaktu przepisać na `dot(pPoint1 − pant0, vFront)`. To warunek każdego zwartego typu sieci.
- **Telemetria ruchu** (`utilities/motiontelemetry.cpp`) liczy prędkość i przyspieszenie z różnic pozycji pojazdu. Kwantyzacja 1 mm przy 60 Hz dawałaby szum do 3,6 m/s². To przesądza: pozycje pojazdów zostają w double.
- **Limit regionu ±250 km.** Z zerem w Warszawie część Polski wypada poza siatkę sekcji. Rzadki katalog kafli (etap R2) musi wejść przed przejściem na układ świata.

**Interfejsy zewnętrzne i zgodność:**

| Interfejs | Pozycje dziś | Co zrobić |
| --- | --- | --- |
| `.scn` | metry w układzie scenerii; sieć, memcelle i launchery zapisywane z 6 cyframi znaczącymi | zostaje w układzie scenerii; eksport w E3 z pełną precyzją |
| Pojazdy w scenariuszu | nazwa toru + przesunięcie | niezależne od układu, bez zmian |
| Blackbox `EU07BBX` v1 | double w układzie scenerii, float dla linii torów | nowa wersja z polem układu; v1 czytana jak dotąd |
| Sieć (multiplayer) | `command_data::location` jako float | 300–500 km od Warszawy to 2–3 cm, wystarcza do szukania w promieniu 50 m; nowa wersja protokołu wysyła względem kafla |
| Python, Lua, WASM | nie wystawiają pozycji | bez zmian |
| Telemetria UDP | nie wysyła pozycji | bez zmian, ale wymaga double w pojazdach |

Zysk pamięci w logice to głównie tory (168 → 84 B na odcinek), sieć (120 → 60 B na przęsło) i węzły (24 → 12 B). Największą pozycją jest jednak `world_vertex`, który znika całkiem.

## 6. Fizyka pojazdów, eventy i Lua

Fizyka i eventy należą w całości do warstwy logiki, więc działają na całej mapie, niezależnie od streamingu. Docelowo logikę scenerii opisuje Lua, a klasyczne eventy zostają jako format zgodności, czytany zawsze.

### Fizyka pojazdów

- **Wejście bez zmian.** Pojazdy nie są częścią formatu świata: scenariusz stawia je po nazwie toru i przesunięciu. `.eu07` tego nie zmienia.
- **Fizyka czyta tylko logikę:** geometrię toru (`TTrackFollower`, `TSegment`), sieć trakcyjną i zasilanie, nierówności toru oraz tarcie i napięcie zmieniane eventami. Wszystko to jest zawsze w pamięci, więc pociąg AI 200 km od kamery jedzie tak samo jak dziś.
- **Dokładność.** Stan pojazdu zostaje w double. W układzie świata liczby są większe (do ok. 500 km), ale double ma tam krok ok. 10⁻¹⁰ m, więc fizyka zachowuje się tak samo. Wyniki nie są jednak identyczne bit w bit z dzisiejszymi. Ma to znaczenie przy porównywaniu blackboxów i w grze wieloosobowej między różnymi wersjami gry, nie w obrębie jednej wersji.
- **Przy okazji:** kontakt pantografu i szczelina sprzęgu liczone z różnic pozycji zamiast z dwóch dużych liczb. W double to działa i dziś, ale tak jest stabilniej.
- **Modele pojazdów** (`.e3d`) wczytują się dziś razem z pojazdem. Daleki pociąg AI nie potrzebuje modelu w VRAM. Streaming modeli pojazdów to osobny, późniejszy krok, poza formatem świata.

### Fizyka w skali kraju

Dziś każdy pojazd liczy wszystko co 10 ms, w jednym wątku. Przy 1000 pociągów to się nie zmieści. Rozwiązanie ma trzy części: poziomy szczegółowości fizyki dla składów, równoległość po składach z deterministycznym zatwierdzaniem oraz tańsze AI. Warunek: poziom zależy od stanu wspólnego wszystkim graczom, nigdy od kamery.

**Stan dziś** (`application/drivermode.cpp`, `vehicle_table::update` w `vehicle/DynObj.cpp`):

- Krok fizyki ok. 10 ms (`m_primaryupdaterate`), bez górnego limitu liczby kroków na klatkę. Elektryka, hamulce i siły liczą się w każdym kroku, ruch po torze raz na paczkę kroków. Przez całą paczkę siły liczą się więc z pochylenia i łuku z jej początku, a odległość między pojazdami jest szacowana: szczelina sprzęgu to odległość z początku paczki plus 10× droga przejechana od tego czasu.
- Jedyny tryb uproszczony to uśpienie (`PhysicActivation`) dla stojących pojazdów bez kabiny. Lokomotywa prowadzona przez AI, EZT i SZT nigdy nie śpią. Uśpione pojazdy dalej szukają drutu i sąsiadów w każdej paczce.
- Wagony składu to osobne bryły połączone sprężynami, całkowane jawnie. Sprzęg zapisuje siłę do sąsiada, więc najmniejszą jednostką pracy równoległej jest skład, nie pojazd.
- AI skanuje 0,75–1,5 km toru co 0,2–0,5 s. Zajętość torów i kolejka eventów to stan globalny zapisywany w trakcie ruchu.
- Gra wieloosobowa wymaga fizyki identycznej bit w bit: klient porównuje sumę kontrolną pozycji wszystkich pojazdów. Wspólny generator losowy (`Global.random_engine`) jest używany w fizyce i AI, więc kolejność losowań też jest częścią stanu.
- Nie ma puli wątków. `train_table::updateAsync` tworzy nowe wątki w każdej klatce i tylko dla kabin graczy.

**Poziomy szczegółowości fizyki** (dla całego składu):

| Poziom | Kiedy | Model | Krok |
| --- | --- | --- | --- |
| F0 pełna | skład gracza, składy w promieniu 5 km od dowolnego gracza (dla dyżurnego od posterunku), składy w interakcji (sprzęganie, manewry, wspólny tor ze składem F0) | dzisiejszy `TMoverParameters` dla każdego pojazdu | 10 ms |
| F1 skład jako jedno ciało | pozostałe jadące składy AI | suma sił: trakcja z charakterystyki pojazdu, hamowanie z uproszczonego modelu przewodu (czasy napełniania i luzowania), opory, pochylenie z toru, pobór mocy z sieci | ok. 0,1 s |
| F2 rozkładowy | składy daleko od graczy i bez interakcji | ruch po profilu prędkości (ograniczenia, hamowanie przed sygnałem) liczonym z góry; wjazd na tor i zajętość w wyliczonych chwilach; nowy profil po zmianie sygnału lub zwrotnicy na trasie | zdarzeniowo |
| Uśpiony | stojące bez kabiny, także AI czekające na odjazd | bez liczenia; budzi go event, rozkład albo sąsiad | — |

F1 robi rząd wielkości mniej pracy niż F0: 10 razy rzadszy krok i jedno ciało zamiast kilku pojazdów. F2 nie ma kroku w ogóle. Dokładne liczby trzeba zmierzyć (niżej).

**Ruch drogowy (samochody).** Samochody jeżdżą w trybie F2, ale są tworzone tylko wokół kamery i nie wpływają na symulację pociągów.

- **Dziś:** punkty `spawn` i `despawn` dróg mają pule pojazdów (`world/RoadPoint.cpp`). W pctga nie ma jeszcze żadnego takiego punktu: samochody to tam pojazdy w składach na torach drogowych (track road, 45 tys. odcinków). Każdy samochód to pełny `TDynamicObject` z fizyką `TMoverParameters`, więc kosztuje tyle co wagon. Pula pomija dziś pociąg lokalnego gracza (`simulation::Train`), więc już teraz zależy od lokalnego komputera.
- **Docelowo:** auto to lekki obiekt kinematyczny. Jedzie po pasie z profilem prędkości (ograniczenia, łuki, auto przed nim), zatrzymuje się przed zamkniętym przejazdem i na światłach. Nie ma Movera, sprzęgów ani hamulców.
- **Streaming wokół kamery:** auta powstają na pasach w kaflach z pierścienia wokół kamery i znikają razem z kaflem. Liczba aut na pasie wynika z natężenia ruchu zapisanego w scenerii i z pory dnia. Model auta to zwykła instancja `.e3d`, rysowana instancjonowaniem.
- **Bez wpływu na pociągi:** to warstwa kosmetyczna, liczona lokalnie na każdym komputerze, poza lockstepem. Przejazd zamyka się według pociągów (logika), a auta tylko na to reagują. W grze wieloosobowej gracze mogą widzieć inne auta i niczego to nie psuje.
- **Wyjątek:** auto, które ma wejść w kolizję z pociągiem (scenariusz wypadku na przejeździe), musi być obiektem symulacji z logiki, nie z warstwy kosmetycznej.
- **Zgodność:** istniejące punkty `spawn` i `despawn` w `.scn` działają dalej. Kompilator zamienia je na źródła i ujścia ruchu, a nowe zachowanie idzie za przełącznikiem w `eu07.ini`.

**Przejścia między poziomami:**

- F1 → F0: stan każdego pojazdu odtworzony z F1. Wspólna prędkość, szczeliny sprzęgów w równowadze dla bieżącej siły, ciśnienia w cylindrach ze stanu hamulca składu, elektryka w stanie „jazda”. Odbywa się to poza zasięgiem wzroku gracza, więc drobny skok jest niewidoczny.
- F0 → F1: suma mas i sił, prędkość środka masy.
- Histereza: awans przy wjeździe w promień, spadek dopiero po wyjeździe poza większy promień i kilku sekundach stabilnej jazdy.
- Kryteria liczą się tylko z pozycji pociągów wszystkich graczy i stanu symulacji, który każdy klient zna z lockstepu. Wolna kamera bez pociągu może poprosić o F0 poleceniem, które idzie przez `command_relay` jak inne.

**Równoległość po składach:**

- Jednostką jest „wyspa”: skład plus wszystko, czego dotyka (sprzęty skład, pociąg w odległości kontaktu na tym samym torze). Wyspy wyznacza się raz na paczkę kroków.
- Faza A, równolegle: każda wyspa liczy swoje kroki. Skutki uboczne (zmiany zajętości, eventy do kolejki, pobór mocy, logi) trafiają do bufora wyspy, nie do stanu globalnego.
- Faza B, szeregowo: bufory zatwierdzane w ustalonej kolejności, według stałego id składu. Kolejka eventów, zajętość i sieć zasilania widzą ten sam wynik na każdym komputerze.
- Losowość: zamiast jednego wspólnego generatora strumień na pojazd, liczony z ziarna gry, id pojazdu i numeru kroku. Wynik nie zależy wtedy od kolejności wątków. Zmienia to konkretne wyniki losowań względem dzisiejszych (np. moment zerwania sprzęgu). Przyjęte na stałe, bez przełącznika. To świadomy wyjątek od zasady z `AGENTS.md`, że zmiana zachowania symulatora ma przełącznik w `eu07.ini`, więc PR musi to wyraźnie opisać.
- Napięcie w sieci: obciążenie podstacji sumowane w fazie B, napięcie dla następnej paczki z poprzedniej. Opóźnienie o jedną paczkę (ok. 50 ms) jest deterministyczne.
- Stała pula wątków dla symulacji, która zastępuje też tworzenie wątków co klatkę w `train_table::updateAsync`.

**Tańsze AI:**

- W F1 i F2 trasa jest skanowana raz i trzymana w pamięci. Ponowny skan następuje tylko po zmianie zwrotnicy albo sygnału na trasie (subskrypcja zamiast odpytywania).
- Długie bocznice pełne odstawionych wagonów: stojące składy jako jeden obiekt dla `find_vehicle`, żeby skan nie był liniowy w liczbie wagonów.
- Uśpione pojazdy nie szukają drutu ani sąsiadów w każdej paczce.

**Do naprawy przed streamingiem**, bo dziś symulacja zależy od tego, co jest narysowane albo kto gra lokalnie:

- Animacja iglic zwrotnic i obrotnic (`RaTrackAnimAdd`, `scene.cpp`) kończy się od razu, gdy komórka nie ma geometrii, a inaczej przez renderer. Ze streamingiem obecność geometrii zależy od kamery. Stan animacji musi więc iść z czasem symulacji w logice, a renderer tylko go pokazuje.
- `update_coupler_physics` wybiera składy względem pociągu lokalnego gracza (`simulation::Train`). W grze wieloosobowej ma liczyć względem pociągów wszystkich graczy.

Analiza sieci znalazła więcej miejsc, w których symulacja zależy od lokalnego komputera. Psują one powtarzalność także w grze jednoosobowej, więc też należą do P0:

- **`train_table::updateAsync`** (przy `async.trainThreads > 0`) zmienia stan symulacji z równoległych wątków bez kolejności, łącznie z losowaniem ze wspólnego generatora. Do zastąpienia wyspami i fazą B.
- **`TTrain::id()`** jest nadawane leniwie i bez blokady (`++prev_train_id`), więc numery składów mogą się różnić między uruchomieniami. Numer ma powstać przy tworzeniu składu.
- **Stan klawisza Ctrl** (`Global.ctrlState`) w `TTrain::Update` decyduje o powrocie nastawnika w każdym pociągu, nie tylko w pociągu gracza.
- **Launchery** są sprawdzane tylko w promieniu sekcji od kamery, także te wyzwalane o godzinie i co określony czas (`basic_region::update_events`). Kolejka czasowa i indeks przestrzenny w logice zastąpią ten warunek.
- **Ekrany Python** wysyłają polecenia do symulacji z wątku roboczego, bez blokady (`PyInt.cpp`). Ekrany Lua i WASM robią to już poprawnie, przez kolejkę na wątku głównym.

**Najpierw pomiar.** Licznik `sim_dynamics` obejmuje całą fizykę, a `sim_ai` i `sim_events` są zadeklarowane, ale nieużywane (`utilities/Timer.h`). Trzeba je podłączyć i dodać koszt na skład. Potem test obciążeniowy: sceneria z setkami składów AI w selftescie, z pomiarem czasu klatki i pamięci.

### Co uprościć w pełnej fizyce F0

F0 jest wzorcem, więc zmiany dzielą się na dwie grupy. Optymalizacje bez zmiany wyników można robić od razu. Zmiany przebiegu fizyki idą za przełącznikiem i z porównaniem przejazdów. Narzędzia już są: `physicslog_writer` zapisuje stan każdego pojazdu, a `Timer::set_delta_override` pozwala powtórzyć przejazd z tymi samymi krokami. W pierwszej grupie wynik ma być identyczny bit w bit.

**Bez zmiany wyników:**

| Zmiana | Dziś | Zysk |
| --- | --- | --- |
| Uśpione pojazdy naprawdę śpią | `update_traction`, `ComputeConstans` i `update_neighbours` idą dla każdego pojazdu w każdej paczce, także uśpionego | duży na stacjach i bocznicach pełnych wagonów; budzenie już jest (`switch_physics`) |
| Mieszanie hamulca ED/EP raz na paczkę | lider EIM przechodzi skład kilka razy w każdej paczce | mały do średniego w EZT i składach z EP |
| Rzeczy czysto wizualne poza symulacją | wycieraczki, lusterka, oświetlenie przedziałów, stukot osi liczone w `Update` dla każdego pojazdu | średni; najpierw sprawdzić, że symulacja nie czyta żadnej z tych wartości |
| Krótsza ścieżka dla wagonów bez napędu | ten sam `compute_movement_` co lokomotywa | do sprawdzenia, ile gałęzi elektryki dziś przechodzi wagon |
| Zwarte gorące dane | `TMoverParameters` to bardzo duża struktura; prędkość, siły i ciśnienia rozrzucone w pamięci | mniej chybień cache; duży refaktor, tylko po pomiarze |
| Tańsze kolejkowanie eventów | stojący pojazd na torze z eventami kolejkuje `m_events0all` w każdej paczce | zależnie od scenerii; sama semantyka „w każdej paczce” zostaje, bo scenerie mogą na niej polegać |

**Ze zmianą wyników** (za przełącznikiem):

| Zmiana | Dziś | Dlaczego warto |
| --- | --- | --- |
| Stały krok 10 ms z akumulatorem i interpolacją przy rysowaniu | krok to czas klatki podzielony na równe części, więc zależy od FPS (`drivermode.cpp`); stały krok był napisany, ale wyłączony, bo bez interpolacji drgał | fizyka niezależna od FPS, powtarzalne przejazdy i blackbox; podstawa pod wyspy i poziomy F0–F2 |
| Limit kroków na klatkę | bez limitu: przycięcie o 1 s to 100 kroków naraz i jeszcze dłuższa następna klatka | koniec z „spiralą” spowolnienia; nadrabianie rozłożone na kilka klatek |
| Podsystemy w różnym tempie | `compute_movement_` liczy przekaźniki, sprężarkę, drzwi, PLC i hamulce w każdym kroku 10 ms | siły, sprzęgi i przewód hamulcowy dalej co 10 ms; sprężarka, drzwi, ogrzewanie, światła i wolne przekaźniki co 50–100 ms |
| Sprzęgi rozwiązywane niejawnie dla całego składu | jawne sprężyny ze sztucznym usztywnieniem ×10, dlatego krok musi być mały | stabilne długie pociągi towarowe i możliwość dłuższego kroku; punktem wyjścia jest opcjonalny `couplerphysics.cpp` |

**Ograniczenie drogi na paczkę (ruch po torze).** Paczka ma do 5 kroków, czyli do ok. 50 ms. Przy 160 km/h skład przejeżdża w tym czasie ok. 2,2 m, przy 300 km/h ok. 4,2 m. Dopiero po paczce `Move` aktualizuje pozycję, pochylenie, łuk i odległości do sąsiadów (`update_neighbours`). Każda zmiana, która wydłuża paczkę albo krok (F1, stały krok, wolne podsystemy), musi więc mieć ograniczenie:

- **Droga, nie tylko czas.** Liczba kroków w paczce wynika z najszybszego obudzonego pojazdu, tak by żaden nie przejechał w paczce więcej niż ok. 1 m. Przy 160 km/h to 2 kroki, przy 360 km/h 1 krok, a przy postoju i manewrach 5 kroków, jak dziś. Próg do zmierzenia na sceneriach z ostrymi załomami profilu.
- **Odległości między pojazdami** po każdym ruchu po torze, nie tylko na początku paczki. Mnożnik 10× w szczelinie sprzęgu to dziś łatka na rzadką aktualizację. Przy ruchu co krok albo przy niejawnych sprzęgach może zniknąć, co zmienia dynamikę składu, więc tylko za przełącznikiem.
- **W F1 (krok ok. 0,1 s)** skład przejeżdża kilka metrów na krok. Siła z pochylenia liczy się wtedy ze średniej po przejechanym odcinku, a nie z punktu, a eventy na torach odpalają się w wyliczonej chwili przejazdu.
- **Eventy i zwrotnice** dalej wykrywa `TTrackFollower::Move`, który radzi sobie z kilkoma torami w jednym ruchu. Ograniczenie drogi zmniejsza jednak opóźnienie, z jakim pojazd „widzi” nowy tor.

To zmiana grupy B: inny podział na paczki daje inne liczby. Idzie razem ze stałym krokiem i z porównaniem przejazdów przez `physicslog`.

Kolejność: najpierw pomiar (P0), potem pierwsza grupa, potem stały krok z interpolacją. Bez niego reszta zmian, także poziomy F1 i F2, nie ma stabilnego punktu odniesienia.

### Przegląd kodu fizyki: konkretne miejsca

Najwięcej kosztuje to, że składy w ogóle nie zasypiają. Same wzory są tanie: hamulec jednego wagonu to według szacunku poniżej 1 µs na krok. Pełna fizyka liczy się jednak dla każdego pojazdu w każdym kroku, bo przewód hamulcowy, zawieszone polecenia i sprzęgi sterowania stale budzą całe składy. Przegląd statyczny z 26.11.2026, bez pomiaru czasu. Oznaczenia: **A** to wynik identyczny bit w bit, **B** to drobna zmiana liczb przy zachowanej fizyce.

**Największe zyski:**

1. Prawdziwe uśpienie składów: budzenie przy odejściu od stanu ustalonego zamiast przy dowolnym przepływie powietrza.
2. Polecenia wysyłane tylko przy zmianie wartości, zamiast co krok przez łańcuch porównań tekstów.
3. Wczesne wyjścia dla podsystemów, których pojazd nie ma albo które stoją (hamulec sprężynowy, sprzęgi zasilania, drzwi, zwolniony hamulec).
4. Wolne podsystemy liczone rzadziej (silnik spalinowy cieplnie, PLC, pompy, wentylatory).
5. Wyrzucenie z kroku symulacji rzeczy czysto wizualnych.

**Uśpienie i budzenie:**

| Miejsce | Co się dzieje | Poprawka | Grupa |
| --- | --- | --- | --- |
| `Mover.cpp:5100–5112`, `GetDVc` | sąsiad budzi się przy przepływie ponad 10⁻⁷ l; nieszczelności płyną zawsze (ok. 5·10⁻⁴ l na krok), więc zasilony skład nigdy nie zaśnie | budzenie, gdy ciśnienie odchodzi od stanu ustalonego, z tolerancją na stały przepływ uzupełniający nieszczelności | B |
| `Mover.cpp:5233` | EZT i SZT nie śpią nigdy | te same kryteria co dla innych pojazdów | B |
| `vehicle_table::update` | uśpiony pojazd w każdej paczce szuka drutu, liczy `ComputeConstans`, skanuje sąsiadów i bierze udział w `UpdateForce` | pominąć, gdy pozycja, tory i zwrotnice się nie zmieniły (znacznik wersji na torze) | A |
| nowy poziom „tylko powietrze” | pojazd zbudzony samym powietrzem liczy całą fizykę | liczy tylko hamulec i `BrakeForce`; pełna fizyka, gdy sąsiad ruszy albo spada siła hamowania na spadku | B |
| `BrakeReleaser`, `BrakeDelaySwitch`, `AlarmChainSwitch`, hamulec lokalny | wywołane lokalnie nie budzą pojazdu (budzi tylko `RunInternalCommand`) | budzenie przy każdej zmianie wejść; warunek bezpiecznego uśpienia | poprawka błędu |

**Polecenia zawieszone w każdym kroku:**

| Miejsce | Co się dzieje | Poprawka | Grupa |
| --- | --- | --- | --- |
| `RunInternalCommand` / `RunCommand` (`Mover.cpp:12954`) | polecenie, które nie zostało skasowane, przechodzi co krok przez do 53 porównań nazw | polecenie zamienione na enum przy ustawieniu | A |
| piasecznica EIM (`SandboxAuto`, `Mover.cpp:6783`) | co krok `SendCtrlToNext("Sandbox")`; sąsiad nie zmienia stanu, więc polecenie nigdy się nie kasuje | wysyłać tylko przy zmianie | A |
| hamulec EP w FVel6/FVE408 (`Mover.cpp:4947`) | co krok `SendCtrlToNext("Brake")` przez cały skład, z budzeniem każdego pojazdu i wyszukiwaniem w `std::map` | siła EP zapisana u odbiorców i liczona lokalnie | A |
| `MainsCheck` (`Mover.cpp:1894`) | gdy włączenie się nie udaje, `MainSwitch` wysyła polecenie do składu co krok | warunek przed wysłaniem | A |
| załadunek i wyładunek (`Mover.cpp:13415`) | przez cały czas wymiany co krok: parsowanie tekstu, pierwiastek, `ComputeMass` z wyszukiwaniem w `std::map<string>` | polecenie sparsowane raz, masa jednostki ładunku policzona przy przypisaniu | A |
| `SendCtrlToNext("EIMIC")` (`DynObj.cpp:3314`) | co paczkę kopiowanie tekstu i budzenie składu przy zmianie | typowane przekazywanie | A |

**Siły i elektryka (co krok, każdy pojazd):**

| Miejsce | Co się dzieje | Poprawka | Grupa |
| --- | --- | --- | --- |
| `BrakeForce` (`Mover.cpp:5466`) | tarcie klocków (`GetFC`, 2 wywołania wirtualne, 3× `exp`) liczone także przy zwolnionym hamulcu, potem mnożone przez 0 | pominąć przy K = 0; `exp(-Vel/25.7)` raz zamiast dwa (`friction.cpp:33`) | A |
| `GetTrainsetVoltage` / `GetTrainsetHighVoltage` | 6–12 wywołań na lokomotywę na krok, każde czyta dane sąsiadów | jedna wartość na początku przebiegu | A |
| `PowerCouplersCheck` (`Mover.cpp:1844`) | 3× na pojazd na krok, ok. 10 warunków za każdym razem; każdy wagon towarowy | wczesne wyjście bez sprzęgów zasilania; typ sprzęgu jako parametr szablonu | A |
| `UpdateSpringBrake` | każdy pojazd ma atrapę cylindra; `PF` z zerową powierzchnią dalej liczy pierwiastek | flaga „ma hamulec sprężynowy”, bez niej pominąć | A |
| `update_doors` (`Mover.cpp:9235`) | ok. 40 operacji na drzwi na krok w każdym wagonie | wczesne wyjście, gdy drzwi zamknięte i nic się nie zmienia | A |
| `TableInterpolation` na `std::map` | charakterystyki silnika spalinowego, przekładni hydraulicznej i EIM szukane w drzewie co krok; falownik EIM liniowo po 33 pozycjach | płaskie posortowane tablice z zapamiętanym indeksem | A |
| `Adhesive()` | do 3× z tymi samymi argumentami (`Mover.cpp:5305`) | raz; dla wagonu bez napędu i hamulca najpierw test `\|Fwheels\| > 0` | A |
| `dizel_Heat` (`Mover.cpp:8728`) | co krok, choć komentarz mówi „oryginalnie dwa razy na sekundę”; stan we `float` gubi drobne przyrosty przy 10 ms | co 0,1–0,5 s z sumowanym krokiem albo z akumulatorem double; tańsze i dokładniejsze | B |
| PLC (`ladderlogic.cpp`) | timery co 0,1 s, ale cały program co krok | program w tempie timerów | B |
| pompy, wentylatory, bateria, sprężarka, światła | stałe czasowe od sekund do minut | co 50–100 ms; zostają co 10 ms: siła trakcyjna, prąd, poślizg, sprzęgi, cylindry, przewód główny | B |

**Hamulce:**

| Miejsce | Co się dzieje | Poprawka | Grupa |
| --- | --- | --- | --- |
| `TReservoir::P()` (`hamulce.h`) | wirtualne, za `shared_ptr`; to samo ciśnienie czytane do 6× w jednym kroku | wartość zapamiętana w `Act()`, zbiorniki przez wartość, klasy `final` | A |
| `PF`, `PFVa`, `PFVd` | w osobnym pliku, LTO wyłączone | inline w nagłówku; wczesne wyjście przy zerowej powierzchni | A (na GCC możliwe FMA, testy złote dopuszczają 10⁻⁹) |
| `GetDVc` | przekrój przewodu liczony 2× na krok; przepływ każdego sprzęgu liczony z obu stron | przekrój przy wczytaniu (A); przepływ raz na sprzęg (B) | A / B |
| `TKE::GetPF` (`hamulce.cpp:2686`) | porównanie `typeid` co krok | flaga ustawiona w `Init` | A |
| `TNESt3::GetPF` | `CVs(BCP)` i `CelBCP` liczone po dwa razy | raz | A |
| `PLC()` w hamulcu | dzielenie co krok, choć zależy tylko od masy i ładunku | przy zmianie masy | A |
| przewód hamulcowy składu | jawne przepływy w kolejności `m_items`, nie składu; krok 10 ms na granicy stabilności (6–9 ms) | niejawne rozwiązanie całego składu (Thomas); nie daje dłuższego kroku, ale usuwa zależność od kolejności i od FPS | B |

**Sprzęgi, pętla pojazdów i AI:**

| Miejsce | Co się dzieje | Poprawka | Grupa |
| --- | --- | --- | --- |
| `couplerphysics.cpp:130`, `series()` | zawsze 60 kroków bisekcji, w każdym 2× pętla Newtona do 50 iteracji; dla różnych urządzeń (lokomotywa–wagon) zawsze; para liczona z obu stron | stop, gdy przedział się nie zmienia (A); Newton lub Brent startujący z poprzedniej siły, ok. 10× mniej obliczeń (B) | A / B |
| `CouplerForce` (`Mover.cpp:5558`) | siła pary liczona przez oba pojazdy; druga wartość nadpisuje pierwszą | raz na parę, zapis na oba końce | A |
| `update_neighbours` → `ABuFindObject` | każdy wolny koniec skanuje co najmniej 50 m toru, także uśpiony; k luźnych wagonów na bocznicy to O(k²) | pominąć bez zmian na torze (znacznik wersji); `CouplerDist` raz na parę | A |
| `update_traction` | pełne przeszukanie sieci w komórkach w każdej paczce dla przęseł równoległych, poza szerokością pantografu i bez drutu | pominąć dla uśpionego pojazdu w tej samej pozycji | A |
| mieszanie ED/EP (`DynObj.cpp:3316–3651`) | 4 alokacje wektorów i 6–8 przejść składu na paczkę; pętla `while(test)` do O(n²) | jeden bufor składu na paczkę; `FmaxEP` użyte ponownie | A |
| `update_coupler_physics` | co paczkę dwa `unordered_set`, `find` w pętli (O(n²)), odległości z pierwiastkiem | flagi z numerem epoki, porównanie kwadratów odległości | A |
| tabela prędkości AI | alokacja wektora na tor, liniowe `TableNotFound` na event, nowa tabela przy przycinaniu, `stable_sort` z buforem | bufory wielokrotnego użytku, usuwanie w miejscu | A |
| `QueueEvents` w `TTrackFollower::Move` (`TrkFoll.cpp:254`) | gałąź „zostaje na torze” bez warunku `m_events`, który ma linia 116 | dodać warunek | A |

**Rzeczy czysto wizualne liczone w kroku symulacji:** `mMatrix` (odwrotność macierzy w `Move`), kąty kół, wycieraczki, lusterka, światła przedziałów, przesunięcie ładunku (`update_load_offset` co krok), ekstrapolacja do rysowania. Można je liczyć przy rysowaniu, tylko dla widocznych pojazdów (A). Nie są czysto wizualne: kąt pantografu (kontakt z drutem), stukot osi (zapisuje przyspieszenia czytane przez AI i test uśpienia) i `fShade` (czyta AI). `update_exchange` i `toggle_lights` losują ze wspólnego generatora, ale przy strumieniach losowych na pojazd przestaje to mieć znaczenie.

**Logi:** `WriteLog` i `ErrorLog` zawsze formatują tekst (`std::format`) i biorą blokadę, a wyłączenie zapisu sprawdza dopiero wątek zapisujący (`Logs.cpp`). `ErrorLog(std::string)` zapisuje dwa razy. Gorące miejsca: „Bad traction” (`DynObj.cpp:3279`) potrafi trafiać co paczkę, „EVENT ADDED TO QUEUE” przy każdym evencie z kolejki. Sprawdzenie na początku `WriteLog` (A).

**Błędy znalezione przy okazji** (poprawka zmienia zachowanie, więc PR musi to opisać):

- **Hamulec sprężynowy:** klucze `ValveOnArea` i `ValveOffArea` z `.fiz` trafiają do zamienionych pól (`Mover.cpp:11780–11781`). Sprawdzone w pctga: wszystkie 261 pojazdów z tą sekcją mają obie wartości równe, więc dziś zamiana nie ma skutku i poprawka jest bezpieczna.
- **`PF1`:** warunek łagodnego obcięcia `sg < DPS` leży wewnątrz `sg > 0.5` i nie może być spełniony (`hamulce.cpp:114–116`). Przepływy LSt drgają wokół równowagi i nigdy nie dochodzą do zera, co też blokuje uśpienie.
- **Mieszanie ED/EP:** dzielenie przez zero, gdy wszystkie pojazdy są nasycone (`DynObj.cpp:3557`). Dziś bez skutku, bo wynik nie jest dodawany, ale do zabezpieczenia.
- **Szybkość sygnału hamowania** w przewodzie zależy od kroku, a krok od FPS. Stały krok 10 ms to naprawia.

### Dane pojazdów w pctga: co waży na fizykę

Przegląd 1487 plików `.fiz` z kopii roboczej SVN, czytanych regułami loadera z `Mover.cpp` (z rozwijaniem include'ów). Niczego w danych nie zmieniono, a poprawki danych wymagają osobnej zgody na commit w SVN.

**Jak często występuje to, czego dotyczą poprawki:**

| Co | Pojazdów | Znaczenie |
| --- | --- | --- |
| zawory z rodziny LSt (LSt, EP2, EStED, EP1) | 864 | błąd w `PF1` dotyczy większości taboru |
| nastawnik FVel6 / FVE408 (EP) | 190 / 4 | głównie EN57 i ED72; każdy obsadzony wysyła polecenie „Brake” przez skład co krok |
| silniki asynchroniczne (EIM) | 184 | 161 z piasecznicą, która włącza się automatycznie przy poślizgu (domyślnie, bez klucza w `.fiz`) |
| silniki spalinowe | 217 | 116 ma model cieplny, który dziś liczy się co krok |
| hamulec sprężynowy | 261 | wartości `ValveOnArea` = `ValveOffArea` wszędzie |
| KE z hamulcami tarczowymi | 35 | dotyczy zmiany `DiscFriction` w KE; wynik identyczny |

**Osobliwości danych:**

| Plik | Co jest nie tak | Skutek w grze |
| --- | --- | --- |
| `pkp/t448p_v2/t448.fiz.inc`, `cd/t478_12_v1/749.fiz.inc`, `749_tlumik.fiz.inc` | sekcja `Clima:` doklejona do linii `Light:` (`LMaxCurrent=227.8Clima: …`) | ustawienia prądnicy grzewczej (H\*) czytane jako część `Light:` i pomijane; 52 pojazdy przez include |
| `pkp/21xm_v1/218ma_b*.fiz`, `pkp/sa134_v1/218m*_b.fiz` (7) | `BuffCoupl2. CType=Screw` bez parametrów sprężyn | sprzęg z zerową sztywnością i skokiem; przy fizycznym modelu sprzęgów możliwe NaN (wniosek z kodu, nie testowane) |
| `pkp/ew58_v1/ew58-sb.fiz` | `BuffCoupl1.` bez `CType` | przedni sprzęg jako brak sprzęgu, sprężyny nieprzeskalowane (kB = 4200 zamiast 4,2·10⁶) |
| `pkp/eu04_v1/eu04-01.fiz`, `-04`, `-24` | `BrakeDelays=GPRR … BrakeDelays=GPR`; liczy się pierwsze, nieznane | nastawy hamulca = 0 |
| `river/zaglowka/zaglowka.fiz` | `CType=bare` małą literą | brak sprzęgu |
| `pkp/en57al_v1/*_s.fiz` i inne | sekcje powtórzone z różną treścią (Engine, Switches, Circuit, Clima) | liczy się ostatnia kopia |
| różne | klucze, których loader nie zna: `fcfuh` (22, impuls), `InactivaCabEmergencyBrake` (112), `type=` małą literą (6), `Comressor`, `DoorClosexDelay` | ustawienie pomijane |
| `pkp/pt31_v1/pt31.fiz` i 2 inne | sekcja `Steam:`, której loader nie ma | pomijana |
| 19 pojazdów `ESt4HBG300*` | nazwa zaworu spoza mapy, enum spada do `ESt3` | sam zawór budowany dobrze; kod sprawdzający `ESt4` widzi ESt3 (wniosek, nie sprawdzone) |

Poprawki tych plików zmieniają zachowanie pojazdów, więc idą osobno, z opisem w PR i commitem w SVN po zgodzie. Poprawki fizyki z tej sekcji nie wymagają żadnej zmiany danych.

**Kolejność:**

1. Pomiar: liczniki `sim_ai`, `sim_events` i koszt na skład, sceneria testowa z setkami składów.
2. Grupa A z tabel: same optymalizacje, porównanie przejazdów bit w bit przez `physicslog`.
3. Polecenia tylko przy zmianie i wczesne wyjścia. To jest jeszcze grupa A, a już odblokowuje część uśpień.
4. Stały krok z interpolacją.
5. Prawdziwe uśpienie i poziom „tylko powietrze”.
6. Wolne podsystemy rzadziej, niejawny przewód hamulcowy, szybszy solver sprzęgów.

**Postęp** (gałąź `perf/physics-f0-safe` od master, bez commita):

- Pomiar: czas AI (`sim_ai`) i eventów (`sim_events`) jest teraz liczony, a panel diagnostyczny pokazuje go razem z liczbą pojazdów, które fizyka liczy („vehicles awake: N of M”).
- Pierwsze poprawki A: wspólne `exp` w tarciu P10Bg i P10Bgu, rodzaj materiału w hamulcu KE ustalany raz w `Init` zamiast `typeid` co krok, przekrój przewodu liczony raz w `GetDVc`, bufory mieszania ED/EP bez alokacji w każdej paczce.
- Sprawdzenie: 639/639 testów, złote przebiegi nagrane na kodzie przed zmianą i porównane bit w bit (`EU07_UNIT_TESTS_GOLDEN_EXACT=1`), bez żadnej różnicy. Mieszanie ED/EP i liczniki leżą poza rdzeniem, który obejmują testy. Tam identyczność wynika z kodu (te same obliczenia, inne miejsce w pamięci), bez przejazdu w grze.

### Eventy w `.eu07`

- Sekcja logiki trzyma eventy jako rekordy według typu. W `world/Event.h` jest ich kilkanaście, od `updatevalues` do `message`. Nazwy są id z tablicy nazw. Kolejka, opóźnienia, warunki i launchery działają jak dziś (`event_manager`).
- Eventy zmieniające wygląd obiektów (`animation`, `lights`, `visible`, `texture`) zmieniają stan w logice. Kafel po wczytaniu czyta ten stan, tak jak semafor w sekcji 5.
- Launchery z promieniem działają na pozycjach z logiki, nie z kafli, więc streaming ich nie dotyczy.

### Lua dziś

- LuaJIT (Lua 5.1), opcja `WITH_LUA`, domyślnie włączona. Sceneria wczytuje skrypt dyrektywą `lua` w `.scn` (`state_serializer::deserialize_lua`).
- API `eu07.events` (`scripting/lua.cpp`): tworzenie, szukanie i wyzwalanie eventów, zajętość torów i odcinków izolowanych, odczyt i zapis memcelli, `dynobj_putvalues`, `random` i log. Event typu `lua` wywołuje funkcję Lua.
- Skrypty scenerii mają pełną bibliotekę standardową (`luaL_openlibs`), łącznie z `io` i `os`. Skrypty ekranów (`scripting/luaprogram.cpp`) działają już w piaskownicy, z limitem instrukcji.
- Obiekty trafiają do Lua jako surowe wskaźniki (`lightuserdata`).

### Lua docelowo

| Etap | Zakres | Zgodność |
| --- | --- | --- |
| L1 Pełne API | każdy typ eventu, memcelle, launchery jako callbacki (wjazd na tor, czas, promień), stan modeli, dźwięki | eventy działają jak dziś |
| L2 Lua w `.eu07` | skrypty scenerii w sekcji logiki jako tekst źródłowy; kod bajtowy LuaJIT nie jest przenośny między wersjami | dyrektywa `lua` w `.scn` działa dalej |
| L3 Konwerter | kompilator albo edytor zamienia klasyczne eventy na równoważny kod Lua; selftest porównuje wynik z oryginałem | `.scn` z eventami czytany zawsze |
| L4 Lua domyślnie | nowe scenerie piszą logikę w Lua, edytor ją edytuje | eventy dalej obsługiwane |

**Wymagania dla Lua w nowym formacie:**

- **Uchwyty zamiast wskaźników.** Obiekty przez id z tablicy nazw. Wskaźniki użyte jako klucze tabel dają kolejność `pairs` zależną od adresów, czyli różną na różnych komputerach.
- **Determinizm.** Gra wieloosobowa powtarza polecenia krok w krok, a blackbox odtwarza przejazd. Skrypt używa tylko czasu symulacji i `eu07.random` z ziarnem gry. `os.time`, `os.clock` i `math.random` są zablokowane albo podmienione.
- **Piaskownica jak w ekranach:** `string`, `math`, `table`, bez `io`, `os` i plików spoza scenerii, z limitem instrukcji na krok. Skrypty z dyrektywy `lua` działają domyślnie jak dziś, z pełną biblioteką. W pctga nie ma jednak ani jednego skryptu Lua scenerii, więc piaskownica może być domyślna dla wszystkich bez kosztu zgodności. Piaskownica obowiązuje nowe API od L2, a dla starych skryptów włącza ją przełącznik w `eu07.ini`.
- **Kolejność.** Callbacki Lua wykonują się w ustalonym miejscu kroku symulacji, w kolejności kolejki eventów, nigdy z innych wątków.
- **Pozycje w API**, jeśli dojdą, są w układzie scenerii, jak w `.scn`. Skrypt nie zależy wtedy od układu świata. Do przeliczenia służy osobna funkcja.
- **Streaming.** Lua nie dotyka danych kafli. Zmiany wyglądu idą przez stan w logice.

## 7. Format pliku `.eu07`

Jedna sceneria to jeden plik `.eu07`: nagłówek, tablice nazw, katalog kafli, sekcja logiki, bloki kafli i sekcja edytora. Katalog i logikę czyta się przy starcie. Każdy kafel to potem jeden odczyt z dysku.

```
[nagłówek]        'EU07' 'EWP1', wersja, flagi, georeferencja, jednostka, rozmiar kafla, liczba LOD
[spis sekcji]     offset i rozmiar każdej sekcji poniżej
[tablice nazw]    materiały, tekstury, modele, nazwy obiektów → dalej tylko uint32 id
[katalog kafli]   na każdy poziom LOD: wpisy posortowane kodem Mortona (tx, tz)
[sekcja logiki]   tory, sieć, eventy, memcelle, stan modeli — czytana w całości przy starcie
[bloki kafli]     wyrównane do 4 KB, każdy osobno skompresowany zstd
[sekcja edytora]  warstwy, include'y, komentarze, hashe źródeł — gra ją pomija
```

**Nagłówek:**

| Pole | Typ | Opis |
| --- | --- | --- |
| magic, typ | 2 × uint32 | `'EU07'`, `'EWP1'` (jak `EU07_FILEHEADER` w `.sbt` i `.btf`) |
| wersja | 2 × uint16 | główna i poboczna (sekcja „Wersjonowanie” niżej) |
| układ | tekst | definicja odwzorowania (np. tekst PROJ): stereograficzne wokół Warszawy na WGS84; pusta dla scenerii bez georeferencji |
| środek układu | 2 × float64, stopnie | szerokość i długość geograficzna zera (WGS84) i skala k₀; albo własny środek scenerii |
| jednostka, kafel | int8, uint8 | −10 (2⁻¹⁰ m), 20 (kafel 2²⁰ jednostek) |
| liczba LOD | uint8 | zwykle 3: L0, L1, L2 |
| hash źródeł | uint64 | hash `.scn` i wszystkich include'ów, do unieważniania |

**Wpis katalogu** ma 34 B, a z wyrównaniem do 4 B 36 B. Przy 10 000 kafli katalog zajmuje ok. 360 KB.

| Pole | Typ | Po co |
| --- | --- | --- |
| tx, tz | 2 × int16 | klucz kafla |
| LOD, flagi | 2 × uint8 | poziom szczegółów; flaga „pusty w tym LOD” |
| offset | uint64 | położenie bloku w pliku |
| rozmiar skompresowany, surowy | 2 × uint32 | odczyt i bufor na rozpakowanie |
| y min, y max | 2 × int32 | culling kafla, zanim się wczyta |
| CRC-32 | uint32 | sprawdzenie bloku po odczycie |

Kod Mortona układa sąsiednie kafle blisko siebie w pliku. Przy jeździe odczyty są więc prawie sekwencyjne, a wyszukanie kafla to wyszukiwanie binarne.

**Blok kafla** to lista kawałków z tagami, jak w paczkach `.tch`. Każdy kawałek ma tag, rozmiar i flagi. Nieznany tag czytnik pomija.

| Tag | Zawartość |
| --- | --- |
| `TINF` | origin kafla, skala odwzorowania, obrót siatki, współrzędne geograficzne środka, liczniki |
| `VPOS` | pozycje wierzchołków, float32 × 3 |
| `VATT` | QTangent i UV |
| `VUSR` | dane użytkownika vec4, tylko gdy są (jak `vertex_userdata`) |
| `IDX2`, `IDX4` | indeksy uint16 albo uint32 |
| `DRAW` | rekordy rysowania |
| `INST` | instancje modeli |
| `LGHT`, `SNDS` | światła i dźwięki otoczenia |
| `TERR` | odwołanie do paczki `.tch` albo siatka terenu |

Kompresja zstd, w ramkach po ok. 64 KB, żeby duży blok rozpakowywać równolegle. Biblioteka jest już w projekcie (heightmapa).

**Sekcja logiki** to rekordy według typu obiektu (tor, sieć, event, memcell…). Każdy rekord ma rozmiar na początku, więc starszy czytnik pomija pola, których nie zna. Nazwy są id z tablicy nazw.

**Sekcja edytora** zachowuje to, czego gra nie potrzebuje, a edytor tak: strukturę warstw i include'ów, parametry `.inc`, komentarze i nieznane tokeny węzłów, hashe plików źródłowych oraz ustawienia z linii `//$c`, `//$o`, `//$z`. Dopiero ona pozwala, by `.eu07` był kiedyś źródłem, a nie tylko cache (sekcja 10).

**Wersjonowanie.** Wersja poboczna dodaje tagi i pola, a stary czytnik je pomija. Wersja główna oznacza zmianę niezgodną. Gra odrzuca wtedy plik i kompiluje go od nowa z `.scn`, jeśli ten istnieje.

## 8. Dane GPU w kaflu

Wierzchołek ma 28 B zamiast 48 B, geometria jest indeksowana, a CPU nie trzyma kopii wierzchołków. Blok kafla po rozpakowaniu kopiuje się do buforów GPU bez przetwarzania.

**Wierzchołek kompaktowy** (`compact_vertex`) to drugi układ obok `gfx::basic_vertex`. Ten drugi zostaje dla modeli `.e3d` i ścieżki ze `.scn`.

| Atrybut | `basic_vertex` (dziś) | `compact_vertex` |
| --- | --- | --- |
| pozycja | float32 × 3, 12 B | float32 × 3 względem środka kafla, 12 B |
| normalna, tangent, znak bitangentu | float32 × 3 + float32 × 4, 28 B | QTangent, snorm16 × 4, 8 B |
| UV | float32 × 2, 8 B | float32 × 2, 8 B |
| **razem** | **48 B** | **28 B** |

- QTangent to kwaternion całego układu stycznych. Znak bitangentu siedzi w znaku w. Dekodowanie w vertex shaderze to kilkanaście instrukcji.
- UV zostaje we float32. Half traci precyzję przy dużych wartościach, a tory i teren mają tekstury powtarzane setki razy.
- Dane użytkownika (`vertex_userdata`, vec4) idą osobnym strumieniem, jak dziś na gl46, tylko tam, gdzie są.
- W backendach to nowy opis atrybutów: `GL_SHORT` znormalizowany w `opengl46geometrybank.cpp` i `VK_FORMAT_R16G16B16A16_SNORM` w `vulkan14scene.cpp`. Do tego wariant shaderów sceny w `shaders/ogl46/` i `shaders/vk14/`.

**Indeksy.** Kompilator spawa wierzchołki kształtów i torów i optymalizuje kolejność pod cache GPU (biblioteka meshoptimizer, licencja MIT, jako nowa zależność w ref/). Dziś są to w większości trójkąty bez indeksów. Zysk trzeba zmierzyć na prawdziwych sceneriach. Indeksy są uint16, gdy siatka ma mniej niż 65 536 wierzchołków, inaczej uint32.

**Rekord rysowania** (48 B, trafia wprost do SSBO):

| Pole | Typ |
| --- | --- |
| pierwszy indeks, liczba indeksów | 2 × uint32 |
| przesunięcie wierzchołków | int32 |
| materiał | uint32 (id z tablicy nazw) |
| sfera: środek względem kafla, promień | float32 × 4 |
| zasięg widoczności min, max | float32 × 2 |
| flagi (przezroczysty, cień, LOD) | uint32 |
| wyrównanie | 4 B |

Shader cullingu zamienia go na polecenie `DrawElementsIndirectCommand`. Shader sceny znajduje swój rekord przez `gl_DrawID`, tak jak dziś instancje (`MODEL_DRAW = pc.draw + gl_DrawID`).

**Instancje modeli.** Na dysku 32 B: id modelu, pozycja float32 × 3 względem kafla, obrót jako kwaternion snorm16 × 4, skala osobna dla każdej osi jako float16 × 3 i 2 B wyrównania. W pctga 326 tys. modeli ma skalę różną w osiach (scale sx sy sz w węźle). Przy wczytaniu rozwijają się do `instance_record` (80 B) z gl46, z origin kafla jako `recordorigin`. Istniejący GPU culling instancji działa bez zmian.

**Teren.** Heightmapa zostaje w paczkach `.tch`, a kafel tylko się do nich odwołuje (kawałek `TERR`). Teren z trójkątów ze `.scn` to zwykła siatka w kaflu. Heightfield rysowany wprost z tekstury wysokości na GPU to osobny, późniejszy krok.

**LOD i HLOD:**

- L0 to pełny kafel.
- L1 to ten sam kafel z połączonymi siatkami według materiału i uproszczoną geometrią, dla dalszego pierścienia.
- L2 to blok 4 × 4 kafli (4096 m) jako sylwetka na horyzont. Przy włączonej krzywiźnie robi największą różnicę.

**Pamięć na wierzchołek:**

| Gdzie | Dziś (master i gl46) | `.eu07` |
| --- | --- | --- |
| RAM, sekcje jeszcze nienarysowane | 44 B (`world_vertex` z double, do pierwszego rysowania) | 0 B |
| RAM po wysłaniu na GPU | 0 B | 0 B |
| VRAM | 48 B | 28 B, a po indeksowaniu mniej wierzchołków |
| 1 mln wierzchołków w VRAM | 48 MB | 28 MB |

## 9. Runtime w OpenGL 4.6 i Vulkanie

Wątki IO czytają i rozpakowują kafle, kopia idzie przez bufor pośredni, a culling i rysowanie robi GPU. Większość klocków już jest na `feat/gl46-migration`: trzeba je połączyć i rozszerzyć z instancji na geometrię statyczną.

Przepływ kafla od dysku do rysowania:

1. **Polityka streamingu:** pierścień wokół kamery, wyprzedzenie wzdłuż toru.
2. **Wątki IO:** odczyt bloku według katalogu, zstd, CRC-32.
3. **Bufor pośredni:** w GL persistent mapped, w Vulkanie pamięć upload z VMA.
4. **Kopia na GPU:** w GL `glCopyNamedBufferSubData`, w Vulkanie kolejka transferowa.
5. **Megabufory:** wycinki kafli i lista wolnych miejsc; wycinek zwalniany po fence.
6. **Culling i rysowanie:** compute z rekordami `DRAW` i Hi-Z, multi-draw indirect.

Wyładowanie kafla wraca od polityki streamingu do megabuforów i zwalnia jego wycinek.

Kafel jest widoczny dopiero, gdy jego kopia się s qkończyła, a wycinek wraca do puli dopiero po fence klatki, która go ostatnio rysowała.

| Element | Jest na gl46 | Do dodania |
| --- | --- | --- |
| Megabufory | GL: strony po 4M wierzchołków z listą wolnych miejsc (`opengl46_geometry_page`) | strony dla `compact_vertex`; Vulkan: duże bufory dzielone przez VMA virtual blocks zamiast buforów na bank |
| Upload | GL: `glNamedBufferSubData` przy pierwszym rysowaniu; Vulkan: staging na kolejce graficznej | GL: pierścień persistent mapped i `glCopyNamedBufferSubData`; Vulkan: istniejąca, nieużywana kolejka transferowa z timeline semaphore |
| Culling | compute z Hi-Z dla instancji (`instancecull.*`, `gfx.gpucull`) | rekordy `DRAW` geometrii statycznej w tym samym przebiegu |
| Polecenia rysowania | GPU wypełnia polecenia, liczbę zna CPU | `glMultiDrawElementsIndirectCount` (core w GL 4.6) i `vkCmdDrawIndexedIndirectCount` tam, gdzie jest `drawIndirectCount`; bez niego tryb jak dziś |
| Materiały | wiązanie na materiał, push descriptors | rekordy posortowane po materiale, jedno multi-draw na materiał; descriptor indexing w Vulkanie opcjonalnie; bez `ARB_bindless_texture` |
| Wątki | Vulkan: pipeline'y i tekstury w tle | pula IO dla kafli; tekstury kafla przez istniejący loader |

**Polityka streamingu:**

- Zasięg wczytania to zasięg rysowania plus jeden kafel. Wyładowanie następuje jeden kafel dalej, żeby kafle na granicy nie migały.
- Priorytet: odległość, kierunek kamery oraz trasa pociągu. Logika zna tory przed pociągiem, więc kafle wzdłuż trasy przychodzą z wyprzedzeniem. Przy 160 km/h nowy kafel wypada co ok. 23 s.
- Budżet uploadu na klatkę, jak w `terrain_streamer`. W RAM pamięć podręczna LRU skompresowanych bloków o rozmiarze z `eu07.ini`.
- Kafel listuje swoje tekstury. Brak tekstury nie wstrzymuje kafla: rysuje się z zastępczą, aż dojdzie właściwa.
- Start i przeskok kamery: pierścień wokół nowej pozycji wczytuje się od razu, przed pierwszą klatką.

**Tryb testowy: jak najmniej RAM-u.** Selftest edytora (`EU07_EDITOR_SELFTEST`) i testy jednostkowe wczytują tylko to, czego test używa, i mają twardy limit pamięci. `AGENTS.md` ostrzega, że pętla w selftescie potrafi zapełnić RAM całej maszyny.

- **Pierścień minimalny:** kafel pod kamerą i jego sąsiedzi. Bez pobierania z wyprzedzeniem wzdłuż toru, bez L1 i L2.
- **Bez pamięci podręcznej bloków** (0 MB). Plik `.eu07` jest mapowany tylko do odczytu (mmap), więc przeczytane bloki to strony pliku, które system może zwolnić, a nie prywatna pamięć procesu.
- **Tekstury:** najmniejszy mip albo zastępcze. Pełne tylko dla kafli w kadrze, gdy skrypt robi `screenshot`.
- **Logika:** domyślnie cała, bo jest potrzebna do symulacji. Testy samej geometrii albo edytora wczytują tylko kafle wskazane w skrypcie (nowe polecenie selftestu, roboczo `area`).
- **Twardy limit:** budżet w `eu07.ini` albo w zmiennej środowiskowej (roboczo `world.memorybudget`, `EU07_MEMORY_BUDGET`). Po przekroczeniu streaming przestaje wczytywać i zapisuje to w logu, a selftest kończy się błędem, zamiast rosnąć dalej.
- **Testy jednostkowe:** małe pliki `.eu07` z kilku kafli, budowane w teście w pamięci, bez GPU i bez danych gry.
- **Pomiar:** selftest wypisuje na koniec szczytowe zużycie pamięci, żeby regresje RAM-u było widać w CI.

### Co dziś obciąża renderer gl46

W dużych sceneriach najwięcej kosztuje CPU, a koszt rośnie z liczbą obiektów i submodeli, nie z liczbą trójkątów. Przegląd kodu `feat/gl46-migration`, bez pomiaru na scenerii.

- **Kilka przejść sceny na klatkę.** W dzień CPU przechodzi scenę 5 razy: 3 kaskady cieni, kolor i przezroczyste. W nocy dochodzą mapy cieni reflektorów i lamp. Zasięg rysowania to 2,5 km (`BaseDrawRange`). Vulkan rozkłada kaskady na wątki, ale pracy nie ubywa.
- **Koszt rośnie z submodelami.** Model bez instancjonowania (światła, animacje, eventy) to jedno wywołanie na każdy widoczny submodel. Do tego wiązanie materiału, zapis UBO i macierz z double. Model instancjonowany to jedno wywołanie na submodel i pasmo LOD, ale każda instancja w widocznych komórkach jest testowana i mnożona przez macierz na CPU w każdym przejściu.
- **Przezroczyste części nie są instancjonowane** (`basic_cell::insert` → `m_instancetranslucent`). Tekstura z alfa bez `opacity` w materiale oznacza część przezroczystą. Dotyczy to tylko submodeli z Opacity poniżej 100; drzewa z tree.t3d są instancjonowane, opis niżej.
- **GPU culling jest domyślnie wyłączony** (`gfx.gpucull no`). Z włączonym znika test i mnożenie na CPU dla instancji, a rysowanie idzie przez multi-draw indirect.
- **Cienie:** 3 kaskady 2048² do 250 m. Każdy materiał z teksturą idzie przez shader z `discard`, co wyłącza wczesny test głębokości dla większości rzucających cień (wniosek).
- **Wypełnienie:** ciężki shader forward (GGX, światła w klastrach, PCF, SSR, GTAO), MSAA 4×, gęste liście z alfa. Prepass głębokości, domyślnie włączony przez SSR i AO, częściowo to ogranicza.
- **Trójkąty** liczą się raz na instancję i przejście: w widoku głównym i w 3 kaskadach. Na współczesnym GPU to zwykle najtańsza część. Ważne są tylko modele z dużą liczbą trójkątów powtarzane blisko kamery, bo nie ma LOD siatek ani impostorów.
- **Przycięcia przy wjeździe w nowy obszar:** geometria i tekstury są wysyłane przy pierwszym rysowaniu, na wątku głównym.

**Drzewa i rośliny: zmierzone w grze.** Drzewa z `tree.t3d` są dziś instancjonowane i rysowane w przebiegu nieprzezroczystym z alpha-testem. Wcześniejszy wniosek, wyciągnięty z samego kodu renderera, był błędny.

- **Dlaczego:** loader `.t3d` (`Model3d.cpp`) daje submodelowi z `Map: replacableskin` i `Opacity: 100` flagę nieprzezroczystego (0x10), a nie bit skórki. `tree.t3d` ma `Opacity: 100.0`, więc kanał alfa skórki niczego nie zmienia. Przebieg nieprzezroczysty robi alpha-test z progiem 0,5.
- **Pomiar:** edytor z gałęzi gl46, kopia terenu i roślinności Wrzosów, widok z bliska. W kadrze 693 modele, z czego 645 instancjonowanych w 58 partiach; `inst-pool` 126 246 z 126 604; CPU przebiegu przezroczystego 0,15 ms. Klatka była ograniczona przez vsync (ok. 6,1 ms), więc pomiar mówi o liczbie wywołań, nie o FPS.
- **Kogo dotyczy kanał alfa skórki** (`is_translucent()`): tylko submodeli z `Opacity` poniżej 100. W modelach `.t3d` w pctga jest ich 27, głównie budynki, wobec 962 z `Opacity: 100`. Modeli `.e3d` nie sprawdzałem.
- **Kształty z `tree.inc`, `grass.inc`, `krzak.inc`** (węzły `triangles`) są scalane w komórkach i idą przez przebieg przezroczysty, bo ich materiały nie mają `opacity`. W kadrze to kilkadziesiąt scalonych kształtów, więc koszt jest mały. `opacity` w tych materiałach przeniosłoby je do prepassu, ale to zmiana danych w SVN.
- **Przełącznik `gfx.skinopacity`** (gałąź `feat/foliage-skin-opacity`, 659/659 testów) działał zgodnie z opisem, ale dotyczył tylko tych 27 submodeli, a dla roślinności nic nie dawał. Gałąź została porzucona 26.11.2026.

**Przy okazji: pełne Wrzosy w edytorze** wczytują się ok. 13 minut (kompilacja RelWithDebInfo) i pod koniec wczytywania przekraczają 20 GB RAM; proces został przerwany na tym limicie. Gdzie dokładnie ta pamięć idzie, nie zmierzono. To kolejny argument za danymi GPU w kaflach zamiast `world_vertex` z double w RAM.

**Jak to sprawdzić na scenerii.** Shift+F12 pokazuje czas CPU i GPU każdego przejścia, liczbę wywołań, submodeli i instancji. Pokazuje też, dlaczego modele nie trafiły do instancji (światła, animacja). Potem wystarczy porównanie z `gfx.gpucull yes` i z wyłączonymi cieniami. Duży spadek czasu klatki przy gpucull oznacza, że wąskim gardłem jest CPU.

**Co z tego dla formatu `.eu07`:** prefaby i instancje zamiast tysięcy modelów (sekcja 11), HLOD i impostory drzew w L1 i L2, GPU culling dla całej geometrii statycznej, upload kafli w tle oraz osobna ścieżka instancji dla przezroczystych liści (alpha-test zamiast sortowania).

## 10. Edytor: dwa formaty, docelowo jeden

Edytor zapisuje dwa formaty: `.scn` dla wydania 26.11 i Rainsteda oraz `.eu07` dla nowego silnika. Docelowo źródłem staje się `.eu07`, a `.scn` powstaje z eksportu na żądanie. Droga prowadzi przez trzy etapy.

| Etap | Źródło prawdy | Edytor zapisuje | Gra czyta | Warunek przejścia dalej |
| --- | --- | --- | --- | --- |
| E1 Kompilator | `.scn` | `.scn`, jak dziś | `.eu07`, gdy hash źródeł się zgadza; inaczej `.scn` i kompilacja w tle | oficjalne scenerie działają z `.eu07` |
| E2 Zapis podwójny | `.scn` | `.scn` (łatanie) i `.eu07` z regionu w pamięci | `.eu07` | `.eu07` z edytora taki sam jak z kompilatora |
| E3 Binarny jako źródło | `.eu07` | `.eu07`; `.scn` przez „Eksportuj do .scn” | `.eu07` | — |

**E1: kompilator.**

- Wczytuje `.scn` dzisiejszą ścieżką (`state_serializer`) i zapisuje region do `.eu07`. Jedna biblioteka, dwa wejścia: gra kompiluje w tle przy pierwszym wczytaniu, a narzędzie wiersza poleceń robi to dla paczek wydania.
- Unieważnianie jak w `.btf`: hash `.scn` i wszystkich include'ów, z listą plików w sekcji edytora. Zmiana dowolnego pliku oznacza kompilację od nowa.
- Zastępuje `.sbt`. Istniejące `.sbt` gra dalej czyta, gdy nie ma `.eu07`, ale przy włączonym `.eu07` nowych już nie zapisuje.

**Dystrybucja.** Oficjalne scenerie dostają gotowe `.eu07` w paczkach wydania (SVN, Steam, GOG), budowane narzędziem wiersza poleceń przy wydaniu. Gra kompiluje lokalnie tylko scenerie bez `.eu07`, scenerie zmienione przez gracza (hash źródeł się nie zgadza) i pliki starszej głównej wersji formatu. Dlatego `.scn` dalej jedzie w paczce obok `.eu07`.

**E2: zapis podwójny.**

- Dziś nie ma interfejsu zapisu: `export_as_text` to metoda wirtualna `basic_node`, a binarne `serialize_` węzłów logiki to głównie zaślepki. Potrzebny jest `node_writer` z dwiema implementacjami (tekst i binarna) albo pełne `serialize_` dla `TTrack`, `TTraction`, `TAnimModel`, `TMemCell`, `TEventLauncher`, `road_node`, `sweep_node`, eventów (`basic_event`) i `sound_source`.
- `.scn` dalej powstaje przez łatanie po zakresach bajtów. Niezmienione definicje zostają bajt w bajt, jak wymaga `AGENTS.md`.
- `.eu07` powstaje z regionu w pamięci, nie z łatania. Przebudowuje się tylko zmienione kafle, a niezmienione bloki kopiuje się z poprzedniego pliku. Zapis jest atomowy: plik tymczasowy, potem zmiana nazwy.
- Po zapisie hash źródeł w `.eu07` odpowiada świeżo zapisanym `.scn`, więc gra nie kompiluje go ponownie.

**E3: binarny jako źródło.**

- Edytor wczytuje `.eu07` wprost, razem z sekcją edytora: warstwami, include'ami, komentarzami i parametrami `.inc`.
- „Eksportuj do .scn” generuje tekst od nowa, z podziałem na pliki według warstw. To eksport, więc nie gwarantuje identyczności bajt w bajt ze starym plikiem.
- Szablony `.inc` z parametrami zostają w `.eu07` jako instancje szablonu (nazwa i parametry), rozwijane przy budowie kafli. Edytor zachowuje to powiązanie.
- Pliki `_ter.scm`, `.btf` i `.sbt` importuje się raz, do kafli.
- Gdy `.scn` zmieni się poza edytorem (Rainsted, ręczna edycja), hash się nie zgadza. Edytor pyta wtedy, czy wczytać `.scn`, czy `.eu07`. Nic nie jest nadpisywane po cichu.

Dane edytora trzymane dziś w osobnych plikach (`profile_store`, `infra_store`, `switch_store`, ustawienia z linii `//$c`, `//$o`, `//$z`) mogą w E3 przejść do sekcji edytora. Nie jest to warunek żadnego etapu.

## 11. Include'y i szablony

W grze include'y znikają: kompilator rozwija je do kafli i logiki, więc wczytanie `.eu07` nie parsuje żadnego pliku tekstowego. Strukturę include'ów zachowuje sekcja edytora, a szablony `.inc` zostają osobnymi plikami źródłowymi.

| Rodzaj | Dziś | W `.eu07` |
| --- | --- | --- |
| Podział scenerii na pliki (np. `_ter.scm`) | każdy plik to warstwa edytora, łatana po zakresach bajtów | warstwa w sekcji edytora; treść rozwinięta w kafle i logikę |
| Szablon `.inc` z parametrami | parametry podstawiane przy wczytaniu; edytor traktuje instancję jako całość (`include_instance`) | instancja: szablon, parametry, przekształcenie; wynik rozwinięcia w kaflach i logice |
| Plik dołączany przez kilka scenerii | wczytywany z każdą scenerią | zależność z hashem; zmiana przebudowuje każdą scenerię, która go używa |
| Losowy zestaw (`deserialize_random_set`) | wybór przy każdym wczytaniu | wszystkie warianty skompilowane jako grupa; wybór przy wczytaniu z ziarnem gry |
| Sieć trakcyjna z `tr/` i `tra/` | pomijana, gdy `LoadTraction` jest wyłączone | grupa „trakcja” w kaflach i logice, pomijana przez czytnik przy wyłączonej opcji |
| `origin`, `rotate`, `scale` wokół include'ów | stos przekształceń przy wczytaniu | zastosowane przy kompilacji; przekształcenie instancji w sekcji edytora |

**Grupy wariantów.** Losowe zestawy i trakcja to jedyne include'y, których wynik zmienia się przy wczytaniu. W kaflach i logice ich zawartość ma znacznik grupy i wariantu. Czytnik wybiera wariant raz, przy starcie scenariusza, ziarnem gry, więc w grze wieloosobowej wszyscy dostają ten sam. Kosztem jest większy plik, bo zapisane są wszystkie warianty.

**Zależności.** Sekcja edytora trzyma listę wszystkich plików źródłowych z hashami: `.scn`, każdy include, każdy `.inc` i pliki terenu. Narzędzie wydania buduje z tych list graf zależności i przebudowuje każdą scenerię, która używa zmienionego pliku. Gracz, który zmieni wspólny plik, dostaje lokalną kompilację przy następnym wczytaniu.

**Szablony w E3.** Gdy `.eu07` staje się źródłem, nie może kopiować treści szablonów, bo zmiana szablonu przestałaby się rozchodzić:

- Szablony `.inc` zostają osobnymi plikami, jak dziś.
- `.eu07` trzyma instancje (szablon, parametry, przekształcenie). Rozwinięcie w kaflach to dane pochodne, przebudowywane po zmianie szablonu.
- Eksport do `.scn` odtwarza dyrektywy `include` z parametrami, zamiast wklejać rozwiniętą treść.
- Podział scenerii na pliki staje się warstwami wewnątrz `.eu07`. Eksport tworzy z nich z powrotem osobne pliki.

**Nazwy.** Szablony budują nazwy obiektów z parametrów, np. nazwę semafora. Kompilator sprawdza kolizje nazw po rozwinięciu wszystkich instancji i zgłasza je w logu.

**Streaming.** Granice include'ów nie mają znaczenia dla kafli. Instancja na styku kafli dzieli się między nie, a jej eventy i memcelle trafiają do sekcji logiki.

**Szablony a Lua.** Szablon to dziś tekst z podstawianymi parametrami. Docelowo może nim być funkcja Lua, która generuje obiekty, np. semafor z liczbą komór jako parametrem. Kompilator wykonuje ją przy budowie `.eu07`, w tej samej piaskownicy co skrypty scenerii.

### Elementy wielokrotnego użytku

To, co powtarza się w scenerii, zapisuje się raz, a każde użycie to tylko odwołanie z przekształceniem. Na GPU rysuje się to instancjonowaniem, tak jak dziś modele.

| Element | Dziś | W `.eu07` |
| --- | --- | --- |
| Model `.e3d` (drzewo, słup, semafor) | jedna kopia modelu, instancje w kubełkach na model (gl46: `instance_record`, GPU culling) | bez zmian: kawałek `INST`, 32 B na instancję |
| Trójkąty z szablonu `.inc` (peron, przejazd, ogrodzenie) | trafiają do sekcji osobno dla każdej instancji (`basic_region::insert`) | prefab: geometria raz w bibliotece, instancje jak modele |
| Ta sama geometria bez szablonu | powielona | wykryta po hashu geometrii lokalnej i zamieniona w prefab |
| Logika szablonu (eventy, memcelle) | pełna kopia na instancję | definicja raz, z miejscami na parametry; instancja trzyma tylko wartości parametrów |
| Tory i sieć trakcyjna | osobna siatka na każdy odcinek | dziś siatka w kaflu; później generowana na GPU z danych logiki (sekcja 8) |

**Biblioteka prefabów.**

- Prefab to geometria względem własnego środka, z materiałami, LOD i sferą. Kawałek `INST` kafla odwołuje się do niego po id, tak jak do modelu.
- Kompilator sam wykrywa powtórzenia: rozwija szablon, liczy hash geometrii względem środka instancji i łączy identyczne. Parametry zmieniające geometrię (długość, tekstura) dają osobne prefaby, po jednym na zestaw wartości.
- Biblioteka może być w pliku scenerii albo w osobnym pliku wspólnym dla wielu scenerii (`.ewl`), z hashem w zależnościach. Wspólne elementy jadą wtedy w paczce raz.
- W VRAM prefab jest raz, z licznikiem odwołań: wczytuje się z pierwszym kaflem, który go używa, i znika z ostatnim.
- W L1 i L2 instancje prefabów mogą się łączyć w jedną uproszczoną siatkę kafla, bo z daleka liczy się liczba wywołań rysowania, a nie pamięć.

**Logika szablonów.** Eventy i memcelle szablonu mają nazwy budowane z parametrów, więc każda instancja potrzebuje własnych obiektów w pamięci. W pliku wystarczy jednak definicja raz i wartości parametrów na instancję. Obiekty powstają przy wczytaniu, co kosztuje trochę czasu startu, a oszczędza miejsce w pliku.

**Tory i sieć generowane na GPU.** Siatka toru to profil szyn, podkładów i podsypki przeciągnięty wzdłuż krzywej. Krzywa i tak jest w logice. Shader może ją więc wygenerować z id profilu i odcinka, bez zapisanej siatki. Podobnie druty sieci z geometrii przęsła. To osobny krok po R3, który usuwa największą część unikatowej geometrii z kafli.

Ile to daje, trzeba zmierzyć na prawdziwych sceneriach w R1: kompilator może policzyć powtórzenia, zanim prefaby wejdą do formatu.

## 12. Zgodność wstecz

`.eu07` jest opcjonalny. Bez niego albo przy wyłączonym przełączniku gra i edytor działają dokładnie jak dziś. Poniżej zasady z `AGENTS.md` przyłożone do tego projektu.

- **Stare pliki czytane jak dotąd.** `.scn` z include'ami, `.sbt`, `.btf`/`.txtf` i `.tch` zostają, bez zmiany słów kluczowych ani jednostek.
- **Przełączniki w `eu07.ini`.** Roboczo `world.binary` dla `.eu07` i `gfx.earthcurvature` dla krzywizny. Parsowane w `global_settings::ConfigParse*` i wypisywane przez `export_as_text` do logu. Krzywizna domyślnie wyłączona, bo zmienia wygląd.
- **Bez wymagań wobec SVN.** `.eu07` powstaje lokalnie z `.scn`. Paczki wydania niosą gotowe `.eu07`, `.scn` zostaje obok aż do E3. Kolejność: najpierw wydanie gry czytające `.eu07`, potem pliki w SVN.
- **Wydanie 26.11 nie czyta `.eu07`.** W E1 i E2 formatem wymiany i wydania zostaje `.scn`. E3 wchodzi dopiero wtedy, gdy wszystkie wspierane wydania czytają `.eu07`, a eksport do `.scn` zostaje na stałe.
- **Zapis `.scn` bez zmian.** Dalej łatanie po zakresach bajtów. `export_as_text` nie gubi więcej z edytowanego węzła niż dziś.
- **Obraz bez zmian.** Streaming rysuje te same siatki z tymi samymi zasięgami widoczności. L1 i L2 pojawiają się tylko dalej niż dzisiejszy zasięg rysowania albo za przełącznikiem.
- **Log.** Poprawne pliki nie dają nowych ostrzeżeń. Nieaktualny `.eu07` to informacja o kompilacji, nie błąd.
- **GCC i Sonar.** Odczyt i zapis jawnie little-endian przez `sn_utils::ls_*` i `ld_*`, bez `memcpy` całych struktur. `static_assert` na rozmiarach rekordów, które idą wprost do GPU.

### Dane scenerii w pctga: co musi przejść przez kompilator

Przegląd 124 scenerii i 5895 plików tekstowych z kopii roboczej SVN, tylko do odczytu. Kompilator `.eu07` korzysta z dzisiejszego parsera, więc wszystkie poniższe osobliwości przechodzą przez niego tak samo jak dziś. Liczby pokazują, co jest ważne dla formatu.

| Co | Ile | Znaczenie dla formatu |
| --- | --- | --- |
| zasięg torów | najwięcej ok. 127 km rozpiętości (Wrzosy), wszystko w ±130 km | mieści się w dzisiejszym regionie i z ogromnym zapasem w int32 |
| include'y | 1,2 mln dyrektyw, z czego 1,2 mln to instancje 2043 szablonów `.inc` | największy koszt dzisiejszego wczytywania; skompilowany plik go usuwa |
| najczęstsze szablony | `tree.inc` 452 tys., `grass.inc` 179 tys., `krzak.inc` 115 tys. | to właśnie prefaby i instancje modeli z sekcji 11 |
| modele ze skalą w trzech osiach | 326 tys. | rekord instancji ma trzy wartości skali |
| include'y z `tr/` i `tra/` | 62 tys. w 92 sceneriach | grupa „trakcja” jest potrzebna |
| losowe zestawy include'ów | 0 | grupy wariantów zostają w formacie, ale dziś nic ich nie używa |
| `rotate` w szablonach | 1058 szablonów obraca też w X lub Z; pozycje i tak obraca się tylko wokół Y | kompilator musi to odtworzyć dokładnie |
| nieskasowany `rotate` | np. `tr/sts160-kotwa-napr160.inc` (34 instancje), `wiejskie/animal.inc` | obrót przechodzi na kolejne węzły; wynik zależy od kolejności w pliku i tak ma zostać |
| teren | 1 plik `.sbt` (956 MB, linia 107), 14 plików `_ter.scm` (1,3 GB), 20 plików `.txtf` (581 MB), 2 heightmapy | import terenu to największa praca kompilatora |
| eventy | 162 tys., z czego 108 tys. `multiple` | konwerter na Lua (L3) ma realną skalę |
| losowość w eventach | 2645 `randomdelay`, 4402 warunki `probability` | losowanie w kolejności kolejki, w fazie B |
| skrypty Lua | 0 | piaskownica bez kosztu zgodności |
| punkty `spawn` ruchu drogowego | 0 | ruch drogowy F2 to nowa funkcja, nie migracja |
| tory bez nazwy (`none`) | 70 tys. | tablica nazw musi dopuszczać brak nazwy |
| powtórzone nazwy torów | 21 scenerii | kompilator zachowuje dzisiejszą regułę, który wygrywa, i loguje powtórzenia |
| komentarze w środku węzłów | 424 tys. w 100 tys. węzłów | sekcja edytora musi je zachować |
| znaczniki edytora `//$` | `//$e` 6690, `//$o` 2680, `//$d` 1033, `//$g` 217 i inne | jak wyżej |
| kodowanie i końce linii | głównie cp1250, 258 plików UTF-8, 3 z BOM; 219 plików tylko LF, 15 mieszanych, 1902 bez końcowego znaku nowej linii | łatanie `.scn` w E2 musi to zostawić bajt w bajt |

**Błędy w danych scenerii** (kompilator powinien je logować, poprawka to commit w SVN po zgodzie):

- `baltyk/mod/nowa_roslinnosc.scm`: liczba `239906.00` raz jako x trawy (linia 186), raz jako wysokość drzewa (linia 20165), czyli 240 km w górę. Prawie na pewno literówka, w tym samym pliku jest `2399.95`.
- `braniewo_szeroki.scn:24`: `terrain nmt/nmt100_warmaz_ter.scm`, a na dysku jest `.txtf`, więc teren jest po cichu pomijany.
- `linia053/scenariusz_os/l053_posers_debicawschod.scm`: 17 eventów z typem `0`.
- `linia61/l61_tory.scm:46585`: tor o długości `-118416.73`.
- `linia053_wrzosy/scm_wrzosy/9-flora.scm`: drzewa na wysokości 0 w rogach obszaru (np. `31374.7 0 -103064`).
- 20 brakujących plików w 227 include'ach, np. `kolejowe/barak-wsk.inc`, `6-9-9/l41594.inc`.
- `td/drogi.scm`: zastępcza georeferencja `//$g EU07-424 0 0`.

## 13. Plan wdrożenia

Format, kompilator i streaming kafli (R1, R2) można robić na master już teraz, bo nie zależą od renderera. Dane GPU (R3) czekają na merge `feat/gl46-migration`, planowany po wydaniu 26.11, z kamieniem milowym 27.04.

| Etap | Gdzie | Zakres | Bramka: warunek przejścia dalej |
| --- | --- | --- | --- |
| R1 Format i kompilator | master | nagłówek, katalog, sekcja logiki; binarne `serialize_`, kompilator do `.eu07` | stan logiki z `.eu07` taki sam jak z `.scn` |
| R2 Kafle i streaming | master, etap E1 | katalog zamiast siatki 500 × 500; kafle w tle, jeszcze `basic_vertex` | RAM nie rośnie z mapą; obraz bez zmian |
| R3 Dane GPU | po merge PR #216 (kamień milowy 27.04) | `compact_vertex`, indeksy, VMA; kolejka transferowa, culling `DRAW`, `IndirectCount` | mniej VRAM; FPS nie gorszy |
| R4 Zapis podwójny | edytor, etap E2 | `node_writer`, zapis `.eu07` z edytora; przebudowa tylko zmienionych kafli | `.eu07` z edytora = `.eu07` z kompilatora |
| R5 Kraj, krzywizna, E3 | po wydaniach czytających `.eu07` | krajowy układ, łączenie scenerii, HLOD; krzywizna w shaderze, `.eu07` jako źródło | dwie scenerie w jednym układzie; eksport `.scn` |

Każdy etap to osobna gałąź od master i osobny PR, zgodnie z `AGENTS.md`. Bramki sprawdzają testy jednostkowe (round-trip `.scn` → `.eu07` → eksport) i selftest edytora (`EU07_EDITOR_SELFTEST`) z porównaniem zrzutów ekranu, zawsze na kopii scenerii.

Etapy Lua (L1–L4, sekcja 6) idą równolegle. L1 nie zależy od formatu i może zacząć się od razu na master. L2 wymaga sekcji logiki z R1. L3 i L4 przychodzą po zapisie podwójnym (R4), bo konwerter i edycja Lua idą przez edytor.

Fizyka w skali kraju (sekcja 6) ma własne etapy:

- **P0:** pomiar i naprawa zależności symulacji od renderera i gracza lokalnego. Na master, przed streamingiem (R2).
- **P1:** pula wątków, wyspy i zatwierdzanie w fazie B, strumienie losowe na pojazd.
- **P2:** poziom F1, skład jako jedno ciało.
- **P3:** poziom F2, ruch rozkładowy.

P0 i P1 nie zależą od formatu. Bez P2 i P3 sceneria całej Polski z setkami pociągów AI nie ruszy.

Multiplayer idzie na koniec, po poprawie fizyki (P0–P3). Obecna sieć jest już przeanalizowana: master wysyła krok czasu i wykonane polecenia, a klient dołącza, odtwarzając całą sesję od początku. Wnioski dotyczące fizyki są w sekcji 6 (P0). Pełny projekt sieci powstanie jako osobna sekcja, gdy fizyka będzie gotowa.

## 14. Decyzje i otwarte pytania

Ustalone 26.11.2026:

| Temat | Decyzja |
| --- | --- |
| Układ świata | odwzorowanie stereograficzne na WGS84, zero na stacji Warszawa Centralna; zdjęcia lotnicze z każdego kraju przeliczane przy imporcie |
| Rozmiar kafla | 1024 m |
| Dystrybucja | gotowe `.eu07` w paczkach wydania; kompilacja u gracza dla scenerii bez `.eu07` i zmienionych |
| Siatki | meshoptimizer |
| Pliki | jeden `.eu07` na scenerię |
| Rainsted po E3 | wspierany przez eksport do `.scn` |
| Pozycje w logice | int albo inny zwarty typ, jeśli nie psuje dokładności; każdy przypadek osobno, z zachowaniem zgodności (sekcja 5) |
| Logika scenerii | docelowo Lua; klasyczne eventy czytane zawsze (sekcja 6) |
| Nazwa pliku | .eu07 |
| Niebo | liczone z położenia kamery, za przełącznikiem (sekcja 4) |
| Prefaby | wspólna biblioteka .ewl dla wielu scenerii (sekcja 11) |
| Pełna fizyka F0 | promień 5 km od gracza; dla dyżurnego od posterunku |
| Ruch rozkładowy F2 | tak, dla pociągów daleko od graczy (sekcja 6) |
| Tory i sieć na GPU | osobny etap po R3 |
| Losowanie w fizyce | osobny strumień na pojazd zawsze, bez przełącznika; wyjątek od zasady przełącznika z AGENTS.md, opisany w PR (sekcja 6) |

Zostaje do ustalenia:

- [ ] Dokładne współrzędne punktu zera (Warszawa Centralna) i skala k₀ (propozycja 0,9992).
- [ ] Jaki jest rozmiar sekcji logiki dla największych scenerii? Do zmierzenia, zanim uznamy, że logika zawsze mieści się w RAM.
- [ ] Ile wierzchołków oszczędza indeksowanie na prawdziwych sceneriach? Do zmierzenia w R1.
- [ ] Progi przejść między F0, F1 i F2 (promień F0 ustalony: 5 km).
