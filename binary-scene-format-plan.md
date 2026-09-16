# Binarny format scenerii pod streaming — plan działania

Dokument roboczy. Cel docelowy: format scenerii, który mapuje się do pamięci i trafia
do GPU bez transformacji danych, z niezależnym streamingiem terenu i obiektów,
gotowy pod renderer Vulkan.

Plan jest ułożony tak, że **każda faza daje wymierną korzyść samodzielnie**.
Fazy 0–4 nie wymagają żadnej decyzji o formacie binarnym ani o rendererze —
jeśli przedsięwzięcie stanie po drodze, to co zrobione, i tak zostaje na plusie.

## Pomiary wejściowe (2026-09-16)

Zmierzone na zbiorze `../pctga` (124 scenariusze, 3829 plików `.inc`, `scenery/` = 7,0 GB)
oraz na instalce Steam. Liczby są powodem, dla którego kolejność faz poniżej wygląda
tak, a nie inaczej.

Sceneria `l204` — 2,3 GB w 154 plikach `.scm`:

```
teren/         2,2 GB   (95% objętości)
citygml/        84 MB
deko/           15 MB
tory/          3,0 MB
```

Populacja węzłów w `l204`:

```
8 970 715  triangles     (99,8% wszystkich węzłów)
   11 258  track
    4 535  model
      664  line_strip
      474  dynamic
```

Postać danych terenu — jeden `node` na **jeden trójkąt**, ~220 bajtów ASCII:

```
node -1 0 none triangles grass_new
8812.0 35.55 -25152.0 -0.0280234 0.999585 -0.00665975 -307.251 -3449.75 end
8814.0 35.65 -25152.0 0.0115008 0.999892 -0.00910931 -307.501 -3449.75 end
8812.0 35.52 -25154.0 -0.0118979 0.999913 0.00561081 -307.251 -3450.0
endtri
```

Wnioski, które ustawiają plan:

1. **Teren dominuje wszystko.** 95% objętości i 99,8% węzłów sceny. Każdy trójkąt jest
   osobnym `shape_node` przechodzącym przez `basic_region::insert()` i `RaTriangleDivider`.
   Żadna inna optymalizacja nie zbliża się rzędem wielkości.
2. **Dane nie mają lokalności przestrzennej na dysku.** Kolejne trójkąty skaczą
   (x = 8812 → 17898 → 6398); w pierwszych 30 000 wierzchołków zakres x to 18 km.
   Streaming z takiego układu jest niemożliwy z konstrukcji — sortowanie przestrzenne
   jest warunkiem wstępnym, nie optymalizacją.
3. **Hybryda heightfield + overlay potwierdzona empirycznie.** Około połowy wierzchołków
   leży na regularnej siatce 2 m, druga połowa nie — baza jest siatką, nieregularne są
   wcięcia torowiska i dróg.
4. **Obiektów jest mało.** 4535 modeli i 11 258 torów na ok. 40 km linii. Prefaby i
   rezydencja assetów mają sens jako porządek w danych i podstawa batchowania, ale nie
   jako oszczędność pamięci — populacja jest o cztery rzędy wielkości mniejsza od terenu.
5. **Sekcja LOGIC jest darmowa.** Cała topologia to ułamek procenta danych, więc trzymanie
   jej rezydentnie w całości nie wymaga żadnego kompromisu.

Szacunek po zamianie bazy terenu na heightfield: 8,8 mln trójkątów na siatce 2 m to
ok. 4,4 mln próbek, czyli `R16` + splat `RGBA8` ≈ **26 MB zamiast 2,2 GB**. Nawet
z overlayem zostajemy trzy rzędy wielkości niżej.

## Pomiar fazy 0 (2026-09-16)

`braniewo_szeroki.scn` (wciąga `l204`), format tekstowy, RelWithDebInfo, clang 22.
Zebrane przełącznikiem `-loadprofile`, surowe dane w `loadprofile.txt`.

```
token loop (rest)      14.42 s     6.6 %
triangle import       125.36 s    57.2 %
triangle insert         3.63 s     1.7 %
  of which divider      0.23 s     0.1 %
other node types       75.61 s    34.5 %
total                 219.01 s

triangles            9 983 053
track                   12 439
model                   11 372
dynamic                    510
shape vertices      36 799 623
shapes divided             269   (extra pieces: 92 328)

pamięć rezydentna:  403 MB -> 9 547 MB   (delta 9 144 MB)
```

Co z tego wynika:

1. **Parsowanie trójkątów to 57% czasu ładowania.** Format binarny kasuje tę pozycję,
   nie zmniejsza.
2. **Pamięć boli bardziej niż czas.** 9,1 GB na 36,8 mln wierzchołków to 248 B na
   wierzchołek, przy danych wierzchołka rzędu 44 B (~1,6 GB łącznie). Pozostałe ~7,5 GB
   to narzut na 9,98 mln osobnych `shape_node` — ok. 960 B na węzeł (`std::string` nazwy,
   wektor, `bounding_area`, `lighting_data` = cztery `vec4` na trójkąt, origin, uchwyty).
   Struktura zarządzająca waży prawie pięć razy tyle, co dane, którymi zarządza.
3. **`RaTriangleDivider` był w tym planie przeszacowany.** 0,23 s, czyli 0,1%; całe
   wstawianie do regionu to 1,7%. Argumentem za rzadkim quadtree zostaje pamięć
   i lokalność przestrzenna, nie koszt dzielenia kształtów.
4. **Drugi cel ujawniony przez pomiar:** `other node types` to 75,6 s (34,5%) przy
   zaledwie 11 372 modelach — ok. 6,6 ms na model, czyli ładowanie `.e3d` i tekstur.
   Wzmacnia to fazę 6 (rejestr assetów), ale dopiero po terenie.

### Porównanie: istniejąca binarka `.sbt`, ta sama sceneria

W `pctga/scenery/` leżał gotowy `braniewo_szeroki.sbt` (1,2 GB); wystarczyło włączyć
`file.binary.terrain`. To jest bezpośredni pomiar tego, ile daje **samo zbinaryzowanie
obecnej struktury**, bez jej zmiany.

```
                        tekst        .sbt
token loop (rest)      14.42 s      7.18 s     (zawiera odczyt 1,2 GB i przebiegi Init*)
triangle import       125.36 s      0.00 s
triangle insert         3.63 s      0.00 s
other node types       75.61 s     83.81 s
total                 219.01 s     90.99 s
pamięć (delta)         9 144 MB    8 913 MB
```

Dwa wnioski, oba istotne dla kolejności faz:

1. **Czas: 2,4x szybciej.** Parsowanie znika co do zera, dokładnie jak zakładano.
2. **Pamięć: bez zmian (-2,5%).** I to jest najważniejsza liczba całego pomiaru.
   `.sbt` deserializuje się do **tych samych 10 mln obiektów `shape_node`**, więc 9 GB
   zostaje 9 GB. Zamiana tekstu na bajty nie rusza narzutu strukturalnego, bo narzut
   nie brał się z kodowania. Wygrana pamięciowa może przyjść **wyłącznie** ze zmiany
   struktury danych — czyli z heightfieldu (faza 2), nie z formatu jako takiego.
   Dla porządku wielkości: `.sbt` to 1,2 GB na dysku, szacowany heightfield ~26 MB.
3. **Po zdjęciu terenu dominuje ładowanie assetów.** W przebiegu `.sbt`
   `other node types` to 83,8 s, czyli 92% całości. Następnym wąskim gardłem po fazie 2
   będzie `.e3d` i tekstury — czyli faza 6, i to z mocniejszym uzasadnieniem, niż plan
   pierwotnie zakładał.

Wniosek dla całego przedsięwzięcia: **format binarny sam z siebie kupuje czas ładowania,
ale nie kupuje pamięci.** Kolejność faz (najpierw struktura terenu, potem kontener) jest
tym potwierdzona.

## Pomiar fazy 1: rozkład terenu (2026-09-16)

`tools/terraincook` na `scenery/l204/teren` (8 799 853 trójkąty, 2,2 GB tekstu).

```
-- rozkład, krok siatki 1,00 m
   na siatce (heightfield)   8 781 214   99,79 %
   poza siatką (overlay)        18 639    0,21 %
   spiętrzone próbki               595   (maks. różnica 2,80 m)

-- realna gęstość próbkowania: rozkład długości krawędzi
   < 4 m    52,58 %
   < 8 m    14,94 %
   < 16 m   18,27 %
   < 32 m   13,30 %
   >= 64 m   0,34 %

-- pokrycie wg rozmiaru komórki
    1 m   4 402 162 komórek     4,40 km2
    2 m   4 396 690 komórek    17,59 km2
    4 m   2 700 291 komórek    43,20 km2
    8 m   1 822 254 komórek   116,62 km2
   16 m   1 026 922 komórek   262,89 km2
   32 m     334 958 komórek   343,00 km2
   64 m      95 082 komórek   389,46 km2
```

### Rozstrzygnięcia

1. **Heightfield się broni.** 99,79% trójkątów leży na regularnej siatce, overlay to 0,21%
   i jest skupiony lokalnie (podgląd `overlay.pgm` jest praktycznie czarny, kilka plamek
   w jednym rejonie). Główne ryzyko fazy 1 — overlay rozlany po całej mapie — nie istnieje.
2. **Mosty i wiadukty to błąd zaokrąglenia.** 595 spiętrzonych próbek na 4,4 mln, czyli
   0,014%. Jedyne miejsca, których heightfield nie wyrazi, mieszczą się w garści trójkątów.
3. **Siatka bazowa to 1 m, nie 2 m.** Krok 2 m obejmuje tylko 25% wierzchołków.
4. **Ale teren NIE jest próbkowany co 1 m.** Wierzchołki mają całkowite współrzędne
   metrowe, natomiast odległości między sąsiadami to 2–4 m w połowie przypadków, z ogonem
   do 32 m. Teren jest zbiorem bloków o różnej gęstości, co widać wprost na `height.pgm`.
5. **Realna powierzchnia terenu to ~390 km², nie 4,4 km².** Liczba „pokrycie 4,40 km2"
   z pierwszego przebiegu liczyła unikalne próbki metrowe i dla rzadziej próbkowanych
   bloków liczyła ułamek ich powierzchni. Wartość zbiega dopiero przy komórce 32–64 m.

### Uczciwy koszt heightfieldu

Dla pokrycia ~390 km², sama mapa wysokości `R16`:

```
2 m     97,5 mln próbek    ~186 MB
4 m     24,4 mln próbek     ~47 MB
8 m      6,1 mln próbek     ~12 MB
```

Z mipmapami i splat mapą w połowie rozdzielczości, przy bazie 4 m, wychodzi rząd
**70–90 MB** — wobec 2,2 GB tekstu, 1,2 GB `.sbt` i 9 GB w pamięci. Wcześniejsze
oszacowanie „~26 MB" było zbyt optymistyczne, bo opierało się na zaniżonej powierzchni.
Rząd wielkości zysku się utrzymuje, ale to jest ~30x, a nie ~100x.

Otwarte: czy drobne próbkowanie (2–4 m) skupia się przy torze, czy rozkłada się losowo
po blokach. Jeśli przy torze — baza 8 m plus overlay korytarza wystarczy. Jeśli losowo —
trzeba bazy 4 m. Do zmierzenia w fazie 2, gdy narzędzie dostanie dostęp do osi toru.

### Gęstość terenu względem toru

Oś toru wyciągnięta z `tory/*.scm` (11 258 torów, 480,1 km cięciw), próbkowana co 8 m,
odległość liczona transformatą chamfer na siatce 16 m.

```
pasmo          trójkąty    śr. krawędź   drobne <4 m
< 25 m        3 389 721       2,75 m        91,8 %
< 50 m          621 774       5,13 m        51,5 %
< 100 m         638 680       6,80 m        32,3 %
< 200 m         823 895       8,03 m        30,4 %
< 400 m         965 577       9,59 m        30,0 %
< 800 m       1 056 106      12,23 m        20,8 %
< 1600 m        781 125      17,03 m         9,7 %
>= 1600 m       430 853      15,83 m        25,7 %
```

**Drobne próbkowanie jest przy torze i degraduje się monotonicznie.** 38,5% wszystkich
trójkątów terenu leży w pasie 25 m od osi — prawie cztery dziesiąte budżetu geometrii
idzie na wstęgę szerokości 50 m, a 390 km2 pola dostaje resztę.

Dwa ostatnie pasma odbijają lekko w górę; najpewniej to osobne dopracowane rejony daleko
od osi `l204` (stacje sąsiednich linii, obszar `citygml`). To 6% trójkątów, nie zmienia
obrazu, ale nie jest zweryfikowane.

Wniosek dla docelowego „drobno wszędzie": jest osiągalne, ale **dzisiejsze dane nie mają
szczegółu poza korytarzem**. Jednolita baza 2 m niczego nie pogorszy, natomiast poza
pasem 25 m będzie interpolacją, nie detalem. Realne zagęszczenie pola wymaga nowego
źródła wysokości — dla Polski dostępny jest publiczny NMT z ALS w siatce 1 m. I to jest
argument za heightfieldem niezależny od wydajności: **do heightfieldu można podmienić
źródło**, do zupy trójkątów nie.

Koszt jednolitej bazy 2 m dla pokrycia 390 km2: ~186 MB samych wysokości, z mipmapami
i splatem rzędu 250-300 MB. Jedna piąta dzisiejszego `.sbt`, a w pamięci bez znaczenia,
bo clipmapa trzyma tylko pierścienie.

### Ustalenia dla fazy 2 wynikające z pomiaru

1. **Jednolita baza 2 m**, bez wielorozdzielczości. Prostsze, zgodne z celem, mieści się
   w budżecie.
2. **Overlay korytarza generowany z geometrii toru**, nie konwertowany ze starego terenu.
   Siatka 2 m nie odda ostro skarpy 1:1,5, a na to gracz patrzy przez cały czas.
3. **Woda jako osobna warstwa** (patrz niżej).
4. **Rzadki katalog kafli**, nie gęsta tablica.

### Korekta do fazy 2

Faza 2 zakładała „jeden plik, stały `chunk_stride`, adres liczony arytmetycznie
z pozycji, bez katalogu". **To nie przejdzie na tych danych.** Pokryte ~390 km² leży
w zasięgu roboczym 51 x 46 km, czyli zajętość jest rzędu 16%, a przy pełnym bounding
boksie scenerii dużo gorsza. Gęsta tablica marnuje sześciokrotnie. Kafle terenu muszą
mieć **rzadki katalog** (kafel 256 m, tylko zajęte zapisane) — arytmetyczny offset
zostaje wewnątrz kafla, nie na poziomie całego pliku.

### Warstwa wody

Pełny bounding box danych to 206 km x 161 km, przy zasięgu roboczym 51 x 46 km, a sumaryczny
footprint trójkątów wychodzi na bezsensowne 19 600 km2. Sprawdzone: odpowiada za to
`woda_ter.scm` — 308 trójkątów, w tym kilka olbrzymich, rozciągniętych na całą mapę.
To nie są śmieci, tylko tafla wody, i nie jest to teren w sensie heightfieldu.

Wniosek dla fazy 2: **woda to osobna warstwa**, płaszczyzna z poziomem i materiałem,
nie próbki wysokości. Konwerter musi ją rozpoznawać i wydzielać przed liczeniem zasięgu,
bo inaczej jedna tafla rozciąga bounding box dziesięciokrotnie i psuje każdy szacunek
zajętości.

## Faza 2, krok 1: upieczony heightfield (2026-09-16)

`terraincook -cook` na `l204/teren`, krok siatki 2 m, kafel 128 próbek (256 m).

```
kafle                        7 531   (129 x 129 próbek, wspólna krawędź)
zajętość bounding boksu       12,3 %
próbki z danymi        116 855 308   93,24 % zapisanych
zakres wysokości       -3,96 .. 103,35 m  (0,002 m na jednostkę)
odrzucone zbyt duże            376   (krawędź > 200 m, tafla wody)
zdegenerowane                  880
spiętrzone                  32 882   (zachowana niższa powierzchnia)

plik surowy                  358,8 MB
plik po zstd -9              126,2 MB   (kompresja 1,5 s)

czas pieczenia                  20 s
szczyt pamięci                 1,7 GB
```

Porównanie do dzisiejszych formatów tej samej scenerii: 2,2 GB tekstu, 1,2 GB `.sbt`.
Surowy `.ehf` to 1/6 tekstu i 1/3,4 `.sbt`; po kompresji 1/17 i 1/10.

Korekta wcześniejszego szacunku: zapowiadałem „250-300 MB z mipmapami". Surowy plik
**bez** mipmap wyszedł 358,8 MB, bo doszła płaszczyzna materiału — bajt na próbkę, czyli
jedna trzecia pliku. Kompresuje się niemal do zera (92% terenu to jeden materiał), stąd
tak dobry wynik zstd, ale w postaci surowej trzeba ją liczyć.

Wizualna kontrola (`cooked.pgm`): ciągła powierzchnia z czytelnymi dolinami rzecznymi,
bez dziur i bez szwów między kaflami, zgodna kształtem z oryginałem.

Uwaga o spiętrzeniach: 32 882 przy pieczeniu, wobec 595 znalezionych wcześniej. To nie
sprzeczność - tamto liczyło tylko wspólne wierzchołki na siatce 1 m, a rasteryzacja
porównuje interpolowane powierzchnie, więc łapie każde nałożenie trójkątów, nie tylko
dzielony wierzchołek. 0,03% próbek.

### Następne kroki fazy 2

1. Kompresja per kafel w kontenerze (katalog ma już rozmiar, dochodzi rozmiar
   skompresowany i kod kodeka) - z 359 MB robi się ~126 MB bez zmiany logiki odczytu.
2. Mipmapy per kafel, pod poziomy clipmapy.
3. Strona silnika: loader `.ehf` i clipmapa pod OpenGL.
4. Overlay korytarza generowany z geometrii toru.

## Ślepa uliczka (sprawdzone, nie wracać)

**Launchery eventów jako listy 1D po torach.** Pierwotny plan stawiał to jako fazę
o najlepszym stosunku zysku do pracy, bo `basic_cell::update_events()` odpytuje je
promieniowo co klatkę i *wygląda* to na hot path. W danych hot patha nie ma:

```
65  eventlauncherów w całym zbiorze pctga (7 GB scenerii)
 0  z traintriggered
```

Większość z nich ma `radius = -1` i godzinę, czyli `IsGlobal()` — nie powinny w ogóle
trafiać do przeglądu przestrzennego. Przeniesienie ich na listy 1D byłoby optymalizacją
czegoś, co nie kosztuje. Pozostaje drobiazg do sprzątnięcia przy okazji: w pętli
`train_triggered` w `scene/scene.cpp` zmienna `radius` jest skalowana prędkością, ale
porównanie i tak idzie do `launcher->dRadius`, więc skalowanie jest martwe.

## Zasady przekrojowe

Obowiązują w każdej fazie, nie są osobnym etapem.

1. **Definicja jest niemutowalna, stan jest osobno.** Plik cooked jest read-only i
   mmapowany. Wszystko, co zmienia się w runtime, żyje w równoległych tablicach
   alokowanych przy ładowaniu. Bez tego konsekwentnie przestrzeganego nie da się
   mmapować niczego.
2. **Logika jest rezydentna, wizualia są streamowane.** Symulacja musi działać
   identycznie niezależnie od tego, co jest wczytane. Żaden element symulacji nie
   sprawdza rezydencji i żaden nie może się nie wykonać z powodu wyładowanego kafla.
3. **Przepływ jednokierunkowy.** Warstwa logiki zapisuje stan; warstwa wizualna
   czyta go przy ładowaniu. Nigdy odwrotnie.
4. **Nazwy rozwiązywane przy kompilacji.** W runtime są indeksy. Stringi lądują
   w opcjonalnej sekcji dla edytora, skryptów i debugowania.
5. **Determinizm.** Numeracja kanoniczna (sortowanie po hashu nazwy, nie po
   kolejności w plikach), kolejność aktualizacji po indeksie, losowość ze strumieni
   indeksowanych ID obiektu — nigdy z globalnego RNG.
6. **Brak pointerów w formacie.** Offsety względem początku chunka i indeksy tablic.

## Poza zakresem

**LOD symulacji fizyki.** Rozważany i odrzucony. Przejścia między poziomami muszą
być bezstratne w obie strony, a syntezowanie stanu pneumatyki przy awansie poziomu
jest źródłem błędów, których nie da się tanio wykryć (skład awansuje i wpada
w hamowanie nagłe). Koszt CPU po stronie AI leży w skanowaniu trasy, nie
w całkowaniu ruchu — adresuje to faza 4. Do tematu wracać tylko wtedy, gdy pomiar
z fazy 0 pokaże, że po fazie 4 fizyka nadal dominuje.

Podział pojazdu na rekord logiczny (rezydentny) i instancję wizualną (streamowaną)
zostaje — to nie jest LOD fizyki, tylko zasada 2.

---

## Faza 0 — pomiar terenu

**Cel.** Zmierzyć to, co pomiary wejściowe wskazały jako dominujące, zanim cokolwiek
zostanie przepisane. Zawężone względem pierwotnego zamysłu: nie cała ramka, tylko
ścieżka terenu.

**Zakres.**
- Czas ładowania `l204` rozbity na: parsowanie `.scm`, tworzenie `shape_node`,
  `RaTriangleDivider`, wstawianie do regionu, budowa banków geometrii.
- Zużycie pamięci: dane wierzchołków terenu osobno od struktur zarządzających
  (`shape_node`, komórki, sekcje).
- Liczniki populacji przy starcie: węzły wg typu, komórki niepuste, banki geometrii.
- Koszt ramki z grubsza, żeby wiedzieć, czy render terenu w ogóle jest problemem,
  czy problemem jest tylko ładowanie i pamięć.

**Gotowe, gdy.** Liczby są zapisane w repo i powtarzalne jedną komendą na `l204`.

**Ryzyko.** Żadne. Po poprzedniej pomyłce z launcherami — nie pomijać.

---

## Faza 1 — konwerter terenu (offline)

**Cel.** Rozstrzygnąć najwyższe ryzyko całego planu, zanim cokolwiek w silniku się
zmieni: czy zupę trójkątów da się rozłożyć na regularny heightfield plus wąski overlay.
Narzędzie samodzielne, silnik nietknięty.

**Zakres.**
- Wejście: pliki `teren/*.scm`. Wyjście: heightfield (`R16`) + splat + lista trójkątów
  nieprzystających do siatki.
- Wykrycie siatki bazowej (w `l204` jest to 2 m) i klasyfikacja każdego trójkąta:
  na siatce → heightfield, poza siatką → overlay.
- Raport: jaki procent trafia do overlaya, jak overlay jest rozłożony przestrzennie,
  ile waży po obu stronach.
- Wizualizacja albo eksport do czegoś oglądalnego — trzeba **zobaczyć**, czy overlay
  pokrywa się z torowiskiem i drogami, czy rozsypuje się po całej mapie.

**Gotowe, gdy.** Dla jednego odcinka `l204` znamy proporcję heightfield/overlay i mamy
pewność, że overlay jest wąskim pasem wzdłuż infrastruktury, a nie połową mapy.

**Ryzyko.** **Najwyższe w całym planie i celowo wyciągnięte na początek.** Jeśli overlay
okaże się rozlany po całej powierzchni, model heightfieldu upada i trzeba wrócić do
streamowanego meshu — ale wtedy wiemy to po tygodniu pracy nad narzędziem, a nie po
przepisaniu renderera.

---

## Faza 2 — teren w silniku: clipmapa pod OpenGL

**Cel.** Zastąpić 8,97 mln węzłów `triangles` heightfieldem o stałym budżecie pamięci.
Na razie pod OpenGL — format jest niezależny od backendu.

**Zakres.**
- Teren przestaje być meshem: chunk to `height R16` + `splat RGBA8` + maska materiałów.
  Mesh to stały grid albo pozycja pobierana z tekstury wysokości.
- Clipmapa: pierścienie o stałym rozmiarze wokół kamery, toroidalne adresowanie,
  aktualizacja tylko wchodzącego pasa. Geomorphing między poziomami w shaderze.
- Jeden plik, stały `chunk_stride`, **adres liczony arytmetycznie z pozycji**, bez
  katalogu. Konsolidacja dzisiejszego `terrain_streamer`
  (`editor/editorTerrainStreamer.cpp`, tysiące plików `chunk_X_Z.etc`) do tego jednego pliku.
- Warstwa **overlay** z fazy 1 jako osobna geometria wzdłuż infrastruktury; heightfield
  pod nią wygładzany przez konwerter.
- **Zapiekanie absolutnego Y instancji** z terenu w pełnej rozdzielczości. Runtime nigdy
  nie odpytuje terenu o wysokość — to warunek niezależności strumieni terenu i obiektów.
- Zgodność wysokości na krawędziach poziomów bitowo identyczna z konstrukcji (brak szwów
  z definicji, nie przez łatanie).

**Gotowe, gdy.** `l204` ładuje się z heightfieldu, zużycie pamięci terenu jest stałe
niezależnie od położenia, horyzont 20–30 km bez spadku wydajności, brak szwów.

**Ryzyko.** Średnie — po fazie 1 największa niewiadoma jest już zdjęta.

---

## Faza 3 — rozdzielenie definicji od stanu

**Cel.** Warunek wstępny mmapowania czegokolwiek. Efekt uboczny: ciągłe tablice
zamiast rozproszonych alokacji z wirtualnymi wywołaniami.

**Zakres.**
- `basic_event` (`world/Event.h`): `m_launchtime`, `m_inqueue`, `m_activator`,
  `m_ignored` wychodzą do równoległych tablic. `m_delay`, `m_delayrandom`, `m_name`,
  topologia zostają jako definicja.
- Memcelle (`world/MemCell.h`): wartości początkowe to definicja, bieżące to stan.
- Stan animacji obiektów (`TAnimContainer`) — faza animacji jako tablica, nie pole
  w obiekcie.
- Stan zwrotnic i torów.

**Gotowe, gdy.** Kolejka eventów przemiata ciągłą tablicę `launchtime[]`; żadna
struktura definicji nie jest zapisywana w runtime.

**Ryzyko.** Średnie — dotyka dużo miejsc, ale mechanicznie. Robić w krokach po
jednym podsystemie, z testem scenariusza T1/T3/T5 po każdym.

---

---

## Faza 4 — profil trasy i przepisany `scan_route`

**Cel.** Zlikwidować chodzenie po grafie obiektów sceny przy każdym skanowaniu
trasy. To jest właściwy koszt AI.

**Zakres.**
- Struktura per tor, w sekcji logiki: `vmax`, `gradient`, `curvature`, zakresy
  sygnałów i zmian limitu wzdłuż toru.
- `TController::scan_route` / `sSpeedTable` (`vehicle/Driver.h`) przepisane na
  przemiatanie ciągłej tablicy. W runtime dochodzi tylko to, co dynamiczne:
  wskazania sygnałów, zajętości, inne pojazdy.
- Zapieczony profil hamowania dla kategorii pociągu na odcinku — punkt rozpoczęcia
  hamowania odczytywany, nie całkowany co klatkę.
- Na tym etapie profil liczony przy ładowaniu; przeniesienie do cookera w fazie 7.

**Gotowe, gdy.** Pomiar z fazy 0 pokazuje spadek kosztu skanowania proporcjonalny
do liczby składów, przy niezmienionym zachowaniu AI na scenariuszach testowych.

**Ryzyko.** Średnie — dotyka zachowania AI, które jest trudne do testowania
regresyjnego. Wymaga zestawu scenariuszy referencyjnych z fazy 0.

---

---

## Faza 5 — walidator jako osobne narzędzie

**Cel.** Wychwycić błędy scenerii przy kompilacji zamiast w postaci cichego
milczenia w runtime. Działa jeszcze na formacie tekstowym, chodzi po tym samym
grafie, co przyszły cooker.

**Zakres.**
- Eventy: wiszące referencje do targetów i memcelli, launchery wskazujące na
  nieistniejące eventy, **eventy zadeklarowane za `FirstInit`**, niezgodność typów
  slotów memcella, eventy nieosiągalne, cykle `multi` bez opóźnienia.
- Rozkłady: nieistniejąca stacja, brak wskazanego peronu, peron krótszy niż skład,
  relacja prowadząca przez tor bez połączenia.
- Po fazie 4, na profilu trasy: **rozkład niewykonalny** — czas przejazdu A→B
  mniejszy niż fizycznie możliwy przy tych limitach, spadkach i tej masie.

**Gotowe, gdy.** Narzędzie przechodzi po scenariuszach referencyjnych i raportuje
z numerem pliku i wiersza; znane błędy w istniejących sceneriach są wychwycone.

**Ryzyko.** Niskie. Wartość dla twórców scenerii natychmiastowa i niezależna od
reszty planu.

---

---

## Faza 6 — rejestr assetów i prefaby

**Cel.** Wprowadzić pojęcie współdzielonego archetypu, zanim zabetonuje się je
w formacie binarnym. Weryfikacja na semaforach, bo są najliczniejsze i najbardziej
powtarzalne.

**Zakres.**
- `MdlMngr`: lookup po `asset_id` (hash logicznej ścieżki) zamiast po ścieżce.
- Tabela zależności: `{ pack_id, asset_id, version, content_hash }`.
- Prefab = asset + kanały animacji + definicja stanów + schemat memcella + światła
  + dźwięki. Kanały animacji kompilowane po **indeksach submodeli**, nie po nazwach
  (dziś `TAnimContainer` dowiązuje się po nazwie w runtime).
- Sceneria tekstowa zyskuje możliwość zapisu instancji prefabu zamiast modelu plus
  pięciu eventów.

**Gotowe, gdy.** Semafory S1/S5/S13, tarcze i semafory kształtowe dają się opisać
parametrami jednego prefabu. **Jeśli się nie dają — warstwa prefabu jest za cienka
i trzeba ją przeprojektować przed fazą 8.** To jest właściwy punkt próby.

**Ryzyko.** Średnie, ale to ryzyko projektowe, a nie wdrożeniowe — i lepiej je
ponieść tu niż po przejściu na binarkę.

### Wynik punktu próby

**Jeden prefab wystarcza — pod warunkiem, że jego parametrem jest lista wskazań,
a nie zestaw liczb.**

Szablony semaforowe różnią się liczbą zdarzeń od 5 do 44, ale liczba parametrów
jest w nich stała (sześć, jeden wyjątek ma siedem). Ta rozbieżność to cała
diagnoza: `(pN)` nie potrafi wyrazić „semafor z wskazaniami {S1, S2, S5, S13}",
więc każda kombinacja komór, drabinki, daszka i tekstury dostaje własny plik.
Stąd ponad tysiąc `.inc` w katalogu scenerii.

Zdarzenia nie są dowolne — wynikają ze wskazań mechanicznie:

```
dwukomorowy (9):      sem_info  s1 s2  sem_info_stop sem_info_vmax
                      sem_ligh1 sem_ligh2  uszk sem_ligh0
pięciokomorowy (37):  sem_info  s1..s5 s10..s13 ms2 m40 sz1
                      sem_info_stop/vmax/v40/v20/Shunt25/Shunt40
                      sem_distinfo_stop/vmax/v100/v40
                      sem_ligh1..13 lighs2 lighz1  uszk sem_ligh0
```

Na każde wskazanie przypada jeden event stanu, jeden event świateł i jedna
prędkość wpisywana do memcella; do tego stały rdzeń (`_sem_info`, `_uszk`,
`_sem_ligh0`) i opcjonalny blok `_distinfo`, gdy semafor powtarza tarczę.

**Co z tego zbudowano.** `scene/prefab.{cppm,cpp}` — rejestr definicji `.pfb`
i rozwijanie instancji. Sceneria zyskała token:

```
prefab ss2czy sem_probny 100 0 -100 90 tabl/a1 endprefab
```

Definicja opisuje wskazania, nie tekst:

```
prefab
 model sem/ps_2_l_cz_ks_ds_03.t3d
 chambers 2
 initial s1
 aspect s1 lights 1 0 velocity 0.0 0.0 endaspect
 aspect s2 lights 0 1 velocity -1 -1 endaspect
endprefab
```

Instancja rozwija się w tekst scenerii i wchodzi przez `injectString`, czyli tą
samą drogą co include — węzły i eventy powstają istniejącym parserem. Format
binarny skompiluje ją wprost, ale interfejs jest już ustalony: **publiczne są
wyłącznie eventy wskazań `<nazwa>_<wskazanie>`**, a światła, prędkości i memcell
należą do prefabu i wolno je zmienić.

Sprawdzone na `prefabtest.scn`: dziesięć eventów (rdzeń plus trzy na wskazanie),
model wczytany, semafor stoi na scenie. Odpowiednik `ss2czy.inc` ma dziewięć —
różnica bierze się z tego, że `.inc` współdzieli `_sem_info_stop` między
wskazaniem S1 a uszkodzeniem, a prefab nazywa prędkości po wskazaniach.

### Kształtowy — drugi test tej samej warstwy

Semafor kształtowy to przypadek, który miał ją rozerwać: `sk1.inc` ma sto wierszy,
dziesięć kanałów animacji na wskazanie, dźwięk przestawienia i trzy kanały światła.
**Nie rozerwał.** To wciąż ten sam kształt — na wskazanie jeden stan, jedne światła,
jedna prędkość — tylko z listą kanałów animacji doczepioną do wskazania:

```
aspect sr2
 lights -1 0 1
 lightdelay 0.5
 velocity -1 -1
 animate rotate ramie_gorne 0 -45 0 45
 ...
endaspect
```

Definicja urosła o `modelangle`, `attach` (kratownica i jej lod), `sound`,
`initiallights` i `lightmodes`. Wszystko to są cechy modelu, nie nowe pojęcia.

Przy okazji wyszło, że `.inc` dzielą animacje na grupy po osiem
(`_sem_anim_sr1` plus `_sem_anim_sr1cd`). `multi_event` trzyma `std::vector`, więc
ten limit dawno nie istnieje — prefab wystawia jedną listę. Dokładnie takiego
śmiecia warstwa prefabu ma nie przepuszczać dalej.

Sprawdzone na `prefabtest2.scn`: dwa semafory z jednej definicji, 72 eventy,
`sem_a` zostaje na „stój", `sem_b` po odpaleniu `sem_b_sr2` podnosi ramię.

**Czego to jeszcze nie pokrywa.** Kanały animacji idą po nazwach submodeli, tak jak
dziś — kompilacja po indeksach należy do formatu binarnego, bo dopiero tam jest
z czym indeks związać. Zostaje też `asset_id` zamiast ścieżki i tabela zależności
paczek.

---

---

## Faza 7 — cooker i kontener; sekcja LOGIC binarnie

**Cel.** Pierwsze realne wyjście binarne. Renderer nietknięty.

**Zakres.**
- Osobny target `tools/scn-cook`, wejście `.scn` (parser z `utilities/parser.h`),
  wyjście kontenera. Deterministyczny bit w bit.
- Kontener: `[header][directory][LOGIC][STRINGS][chunks…]`, wyrównanie 16 B,
  `static_assert` na rozmiarze każdej struktury, little-endian z markerem,
  `magic + version + hash schematu`; niezgodność = odmowa ładowania, nie migracja.
- Sekcja LOGIC: tory, trakcja, eventy, memcelle, wyzwalacze, profil trasy — płaskie
  tablice z indeksami zamiast pointerów. Znika `init_targets()` i `TrackJoin()`
  z runtime'u.
- Rozkłady w **osobnym** kontenerze (edytowane niezależnie, przeładowywalne na
  gorąco, ten sam teren obsługuje wiele scenariuszy).

**Gotowe, gdy.** Scenariusz ładuje się z binarnej sekcji LOGIC, zachowanie
identyczne, czas ładowania mierzalnie niższy.

**Ryzyko.** Średnie. Do rozstrzygnięcia przed startem: **relacja z VCS** — patrz
sekcja na końcu.

### Uwaga: na branchu `scenery-streaming` leży inny format binarny

`scene/scenerybinary_format.md` (wersja 10) opisuje **twin strumienia tokenów** —
każdy plik tekstowy kompiluje się do binarnego bliźniaka konsumowanego na poziomie
`cParser`, więc deserializer scenerii nigdy nie widzi formatu. Kod siedzi na
`scenery-streaming` (commit `6ba19dd6` i późniejsze), a sama specyfikacja przyjechała
tutaj z „Editor checkpoint" — dokument bez kodu, co jest mylące.

**To nie jest to samo co sekcja LOGIC i nie konkuruje z nią.** Twin skraca
tokenizowanie, zostawiając całą ścieżkę parsowania i budowania obiektów bez zmian.
LOGIC likwiduje samą tę ścieżkę: płaskie rekordy, indeksy zamiast nazw, brak
`init_targets()`. Można mieć oba naraz — twin dla tego, co zostaje tekstem (sceneria
w edycji), LOGIC dla tego, co jest upieczone.

Czego **nie** wolno zrobić: napisać drugiego kontenera na tokeny. Jeśli kiedyś
okaże się, że twin wystarcza, to LOGIC jest zbędny, a nie odwrotnie.

### Odstępstwo: kucharz siedzi w silniku, nie w osobnym targecie

Plan mówił `tools/scn-cook` czytający `.scn` parserem z `utilities/parser.h`.
Zrobione inaczej — `-cooklogic` piecze kontener z **załadowanego stanu**, bo cele
eventu, rodzeństwo i indeksy są produktem całego deserializera scenerii, a nie
parsera; osobne narzędzie musiałoby wyhodować jego drugą kopię. Pieczenie
z załadowanego stanu daje to, o co naprawdę chodzi: zapisane jest dokładnie to, co
produkuje ścieżka tekstowa. Determinizm bierze się z writera, nie z miejsca wywołania
— stringi internowane w kolejności pierwszego wystąpienia, rekordy w kolejności
menedżera, wyrównanie zerowane.

### Zrobione w tym kroku

- `scene/logicformat.h` — układ bajtów, bez typów silnika: `file_header`,
  `section_entry`, `string_entry`, `event_record`, `target_entry`, `static_assert`
  na każdym rozmiarze, `schema_hash` w nagłówku.
- `scene/logiccontainer.{cppm,cpp}` — writer i reader. Reader odrzuca wszystko, co
  nie zgadza się co do magic, wersji, hasha schematu, endianu, liczby sekcji lub
  rozmiaru pliku. Odmowa, nie migracja — to cache, zawsze można upiec od nowa.
- `simulation/logiccook.{cppm,cpp}` — `-cooklogic` piecze i **natychmiast czyta
  z powrotem**, porównując z eventami w pamięci: nazwa, rodzaj, opóźnienia, liczba
  i kolejność celów.

**Czego ten krok jeszcze nie robi.** Kolumna `node` w `target_entry` to `no_index`,
bo nie ma jeszcze sekcji węzłów, w której indeks miałby na co wskazywać —
więc `init_targets()` **nadal działa**. Ten krok stawia kontener, dyscyplinę schematu
i uprząż cook/verify, z których korzysta każda następna sekcja. Pola `reserved0`
w rekordzie nie udaję: flagi per-rodzaj leżą w podklasach i pojadą razem z resztą
ładunku, gdy będzie dla nich sekcja.

---

---

## Faza 8 — kafle wizualne i streamer obiektów

**Cel.** Streaming geometrii i instancji, niezależny od terenu.

**Zakres.**
- Rzadki quadtree po kodzie Mortona zamiast `EU07_CELLSIZE` 250 m /
  `EU07_SECTIONSIZE` 1000 m / `500×500` sekcji (95% dzisiejszych sekcji jest pustych,
  a stacja węzłowa dostaje ten sam budżet co pole).
- Chunki **posortowane po Mortonie** w pliku — ruch wzdłuż linii to odczyt
  sekwencyjny, prefetch to jeden `pread` na kilka kafli.
- Geometria GPU-ready: pozycje `int16` względem origin kafla (rozdzielczość ~4 mm
  przy kaflu 256 m, przy okazji likwiduje problem precyzji `dvec3`), normalne
  i tangenty oktahedralne, UV `unorm16`, indeksy 16-bit lokalne. Przepuszczone
  przez `meshoptimizer` + meshlety.
- LOD-y jako osobne chunki pod tym samym `tile_id` — pobranie lod3 bez czytania lod0.
- Instancje prefabów SoA, posortowane po `model_id`.
- Nagłówek kafla deklaruje zależności assetowe; kafel staje się widoczny dopiero
  gdy assety są rezydentne.
- Rezydencja assetów: refcount po kaflach + LRU, z flagą `pinned` dla małych
  i wszechobecnych (semafory, wskaźniki, słupki hektometrowe).
- Kompresja per chunk (zstd), nagłówek chunka nieskompresowany.
- **Dwie niezależne kolejki I/O i dwa budżety pamięci.** Teren ma twarde
  pierwszeństwo z wywłaszczeniem; pod presją pamięci dociskane są wyłącznie obiekty.
- Streamer najpierw na GL33 (`glBufferSubData`) — format jest już GPU-ready.

**Gotowe, gdy.** Przejazd całą scenerią bez hitchy, przy stałym suficie pamięci.

**Ryzyko.** Średnie. Najwięcej pracy, ale na tym etapie wszystkie decyzje
projektowe są już zweryfikowane wcześniejszymi fazami.

---

---

## Faza 9 — backend Vulkan

**Cel.** Konsumpcja tego samego formatu bez zmian w plikach.

**Zakres.**
- Dedykowana kolejka transferu, persistent-mapped ring staging 64–128 MB.
  Rozmiar chunka 0,5–4 MB (mniejsze topią się w barierach, większe dają hitch).
- Arenas przez VMA, `buffer_device_address` = baza areny + offset z katalogu, żeby
  deskryptory nie zmieniały się przy load/unload.
- Bindless (`VK_EXT_descriptor_indexing`), materiał jako indeks w SSBO.
- `vkCmdDrawIndexedIndirectCount`, culling meshletów w compute.
- Tekstury poza formatem sceny: KTX2 + BC7/BC5, streamowane niezależnie, mip-tail
  rezydentny. Sceneria trzyma tylko `texture_id`.
- Sparse binding rozważyć dopiero po pomiarze — komplikuje i bywa wolniejszy.

**Gotowe, gdy.** Ten sam plik scenerii działa na obu backendach.

---

---

## Do rozstrzygnięcia przed fazą 7: relacja z VCS

Dzisiejszy `.scn` jest diffowalny i cała społeczność na tym pracuje. Binarka to
zabija. Dwie drogi, trzeba wybrać świadomie:

- **Tekst jako źródło, binarka jako artefakt cookera** (jak shader → SPIR-V).
  Cooker deterministyczny, uruchamiany w CI. Rekomendowane.
- **Binarka jako jedyne źródło prawdy.** Wtedy obowiązkowo `scn-diff` / `scn-merge`
  i git driver — inaczej pierwszy merge dwóch osób na tej samej stacji kończy się
  utratą pracy.

## Kolejność w skrócie

Faza 0 mierzy. Fazy 1–2 zdejmują teren, czyli 95% objętości i 99,8% węzłów — i to one
decydują, czy całe przedsięwzięcie ma sens.
Fazy 3–5 są opłacalne niezależnie od formatu i od renderera.
Faza 6 to punkt próby dla modelu prefabu.
Fazy 7–9 wymagają podjętej decyzji o cookerze i o VCS.

Gdyby robić tylko jedną rzecz z tej listy: **faza 1**. Gdyby dwie: **1 i 2**.
