# Format modeli E4D (`.e4d`)

Następca E3D: w pełni binarny format modeli. Plik czyta się prawie bez przetwarzania, prosto do buforów OpenGL 4.6 i
Vulkana 1.4. Niesie węzły TRS, skinning, typowane komponenty i metadane węzłów oraz grup wierzchołków. Rozszerza się bez
podnoszenia wersji. Rekordy, rozmiary i offsety są w `model/e4d/e4d_format.h`, a `static_assert` pilnują, żeby kod i ten
opis się nie rozjechały.

Stan: wersja 1.0, etap 1 (format, czytnik, pisarz, konwersja z E3D/T3D). Gra jeszcze nie wczytuje `.e4d`; to etap 2.

## Zasady

- **Little-endian.** Rekordy leżą w pliku tak, jak w pamięci, i tak trafiają do GPU.
- **Spis sekcji (TOC).** Każdą sekcję znajduje się przez spis, a nie przez przejście po kolejnych kawałkach. Czytnik może
  wziąć tylko te sekcje, których potrzebuje.
- **Tablice z `stride`.** Sekcja z rekordami zaczyna się od `count` i `stride`. Nowa wersja dopisuje pola na końcu
  rekordu: stary czytnik bierze znany prefiks, a nowy daje brakującym polom wartości domyślne.
- **Sekcje wymagane i opcjonalne.** Nieznaną sekcję opcjonalną czytnik pomija, a narzędzia przenoszą ją bez zmian.
  Nieznana sekcja wymagana sprawia, że plik jest odrzucany z czytelnym komunikatem.
- **Wersja.** Wersja główna zmienia się tylko przy zmianie niezgodnej; czytnik odrzuca plik o wyższej wersji głównej.
  Wersja poboczna oznacza nowe pola i sekcje, które starszy czytnik pomija.
- **Plik to dane niezaufane.** Czytnik sprawdza każdy offset, rozmiar, indeks, rodzica, joint i odwołanie, zanim
  cokolwiek zostanie użyte (`e4d::validate`).
- **Semantyka w danych, nie w nazwach.** Role, strony pojazdu, warianty i sloty świateł są polami komponentów. Nazwy
  zostają, żeby dzisiejsze `.mmd` i eventy działały bez zmian.

## Nagłówek (64 B)

| Offset | Typ | Pole |
|---|---|---|
| 0 | char × 4 | magic `E4D\0` |
| 4 | u16 × 2 | wersja główna (1), poboczna (0) |
| 8 | u32 | flagi: bit 0 = kolejność indeksów zoptymalizowana |
| 12 | u32 | liczba sekcji |
| 16 | u64 | offset spisu sekcji |
| 24 | u64 | rozmiar pliku |
| 32 | u64 | FNV-1a 64 bajtów od offsetu 64 do końca pliku |
| 40 | u64 | hash pliku źródłowego (eksporter i konwerter wiedzą, kiedy budować od nowa) |
| 48 | 16 B | zarezerwowane, zera |

## Wpis spisu sekcji (32 B)

| Typ | Pole |
|---|---|
| u32 | FourCC |
| u32 | flagi: bit 0 = wymagana, bit 1 = zstd, bit 2 = kodek meshoptimizer (tylko strumienie wierzchołków i indeksów) |
| u64 | offset w pliku |
| u64 | rozmiar w pliku |
| u64 | rozmiar po rozpakowaniu |

Sekcje są wyrównane do 16 B, a strumienie wierzchołków i indeksów do 64 B. Sekcja może być skompresowana zstd. Każda
sekcja występuje najwyżej raz.

Strumienie wierzchołków (`VPOS`, `VATT`, `VSKN`, `VSKW`, `VUSR`, `VBND`) mogą być zapisane kodekiem wierzchołków
meshoptimizera (wersja 1), a strumienie indeksów (`IDX2`, `IDX4`) jego kodekiem sekwencji indeksów (`ref/meshoptimizer`,
licencja MIT). Kodek jest bezstratny co do bajtu, a rozpakowuje się kilka razy szybciej niż zstd na tych danych. Gdy sekcja ma
obie flagi, zstd pakuje wynik kodeka; czytnik najpierw rozpakowuje zstd (najwyżej do granicy rozmiaru kodeka), potem dekoduje.
Rozmiar po rozpakowaniu we wpisie spisu to zawsze rozmiar surowego strumienia. Flaga kodeka na innej sekcji odrzuca plik.

## Sekcje

`[T]` oznacza tablicę: `u32 count`, `u32 stride`, potem `count` rekordów co `stride` bajtów. `[S]` oznacza surowy
strumień bez nagłówka: liczba elementów to rozmiar sekcji podzielony przez rozmiar elementu.

| FourCC | Wym. | Postać | Zawartość |
|---|---|---|---|
| `STRS` | tak | — | `u32 count`, `u32 blob_size`, `count + 1` offsetów `u32`, blob UTF-8. String `i` to bajty `[offset[i], offset[i+1])`. |
| `NIDX` | nie | [T] 16 B | indeks nazw: `u64` FNV-1a 64 nazwy w ASCII małymi literami, `u32` węzeł, `u32` 0; posortowane po hashu, potem po węźle |
| `NODE` | tak | [T] 80 B | węzły |
| `XFRM` | nie | [T] 80 B | `u32` węzeł, 12 B zera, macierz 4×4 `f32` kolumnami: przekształcenie węzła, którego TRS nie opisze (ścinanie) |
| `MESH` | tak | [T] 16 B | `u32` pierwszy prymityw, `u32` liczba, `i32` skóra (−1), `u32` flagi (0x1: siatka używa `VUSR`) |
| `PRIM` | tak | [T] 48 B | prymitywy (rekordy rysowania) |
| `VPOS` | tak | [S] 12 B | pozycja `f32 × 3` |
| `VATT` | tak | [S] 16 B | QTangent `snorm16 × 4`, UV0 `f32 × 2` |
| `VSKN` | nie | [S] 8 B | jointy `u8 × 4`, wagi `unorm8 × 4` (suma 255) |
| `VSKW` | nie | [S] 16 B | jointy `u16 × 4`, wagi `unorm16 × 4` (suma 65535); zamiast `VSKN`, gdy skóra ma ponad 256 jointów |
| `VUSR` | nie | [S] 16 B | dane użytkownika `f32 × 4` (dzisiejsze `userdata`) |
| `VBND` | tak, gdy jest | [S] 4 B | przypisanie wierzchołka scalonego prymitywu: `u8` joint skóry prymitywu, `u8` slot światła, `u8` rola w slocie (0 brak, 1 zapalony, 2 zgaszony), `u8` 0. Pisarz oznacza go jako wymagany, bo bez niego scalone prymitywy rysują się źle |
| `IDX2` | nie | [S] 2 B | indeksy `u16`, lokalne względem `base_vertex` prymitywu |
| `IDX4` | nie | [S] 4 B | indeksy `u32`, jw. |
| `MATL` | tak | [T] 80 B | sloty materiałów |
| `SKIN` | nie | [T] 16 B | `u32` pierwszy joint w `SKJT`, `u32` liczba, `i32` węzeł korzenia, `u32` 0 |
| `SKJT` | nie | [T] 64 B | `u32` węzeł, 12 B zera, odwrotna macierz bindu 3×4 `f32` (cztery kolumny po trzy) |
| `VGRP` | nie | [T] 32 B | grupy wierzchołków |
| `VGIT` | nie | [S] 4 B | elementy grup: wierzchołki albo pary (pierwszy, liczba) |
| `VGWT` | nie | [S] 1 B | wagi `unorm8` elementów grup, które je mają |
| `CMPT` | nie | [T] 20 B | komponenty |
| `CDAT` | nie | — | dane komponentów |
| `BNDS` | nie | [T] 40 B | granice: `f32 × 3` min, `f32 × 3` max, sfera `f32 × 4`; jeden rekord na węzeł (geometria węzła w jego przestrzeni), ostatni dla całego modelu |
| `EDIT` | nie | [T] 16 B | `u32` ścieżka pliku źródłowego (string), `u32` narzędzie (string), 8 B zera |

Wszystkie strumienie wierzchołków (`VPOS`, `VATT`, `VSKN`/`VSKW`, `VUSR`, `VBND`) mają tyle samo elementów: jeden na wierzchołek
pliku. Strumień skóry obejmuje więc też wierzchołki siatek sztywnych (z zerami; zstd je zjada). Do GPU trafiają tylko
zakresy siatek skinowanych.

### Węzeł (80 B)

| Offset | Typ | Pole |
|---|---|---|
| 0 | u32 | nazwa (string) |
| 4 | i32 | rodzic, −1 dla korzenia |
| 8 | u32 | koniec poddrzewa: indeks za ostatnim węzłem poddrzewa |
| 12 | u32 | flagi: bit 0 animowany, bit 1 ukryty, bit 2 przekształcenie w `XFRM`, bit 3 bez własnego przekształcenia (submodel E3D bez macierzy; TRS to tożsamość) |
| 16 | f32 × 3 | translacja |
| 28 | f32 × 4 | obrót, kwaternion x y z w |
| 44 | f32 × 3 | skala |
| 56 | i32 | siatka, −1 bez siatki |
| 60 | u32 × 2 | komponenty węzła: pierwszy, liczba |
| 68 | f32 × 2 | kwadrat odległości od kamery: od, do (0 = bez ograniczenia) |
| 76 | u32 | `iFlags` submodelu E3D, z którego węzeł powstał; 0 w modelu nowym |

Węzły leżą w porządku DFS: rodzic przed dziećmi, a poddrzewo zajmuje ciągły zakres `[węzeł, koniec poddrzewa)`. Dzieci
jednego rodzica są w kolejności rysowania. Macierze świata liczy się więc jedną pętlą od początku tablicy. Przekształcenie
węzła to T · R · S, chyba że ma flagę `XFRM`.

### Prymityw (48 B)

| Offset | Typ | Pole |
|---|---|---|
| 0 | u32 | pierwszy indeks w `IDX2`/`IDX4`; bez indeksów: pierwszy wierzchołek liczony od `base_vertex` |
| 4 | u32 | liczba indeksów; bez indeksów: liczba rysowanych wierzchołków |
| 8 | u32 | `base_vertex` w strumieniach |
| 12 | u32 | liczba wierzchołków prymitywu |
| 16 | u32 | materiał (slot `MATL`) |
| 20 | u32 | węzeł |
| 24 | i32 | skóra, −1 dla sztywnego |
| 28 | u32 | flagi: bity 0-3 topologia (wartości GL: 0 punkty, 1 linie, 2 pętla, 3 łamana, 4 trójkąty, 5 pas, 6 wachlarz); 0x10 indeksowany; 0x20 indeksy w `IDX4`; 0x40 przebieg nieprzezroczysty; 0x80 przebieg przezroczysty; 0x100 bez cienia; 0x200 sztywne jointy z `VBND`; 0x400 sloty świateł z `VBND`; bity 16-23 LOD |
| 32 | f32 × 4 | sfera w przestrzeni węzła |

### Prymitywy scalone

Dziś każda ruchoma część i każda lampa (`Light_On##` / `Light_Off##`) to osobny submodel, a więc osobny draw call w
każdym przebiegu. Model scenerii ze światłem albo animacją nie jest przy tym instancjonowany. E4D pozwala złożyć części
kilku węzłów o tym samym materiale w jeden prymityw:

- **Sztywne jointy (0x200).** Każdy wierzchołek wskazuje w `VBND` joint skóry prymitywu (`skin` ≥ 0), z wagą 1.
  Wierzchołki leżą w pozie bindu, tak jak w skinningu, a macierz to `świat(joint) · odwrotny bind(joint)`. Vertex shader
  bierze ją z bufora póz instancji sam, bez przebiegu compute. Prymityw nie potrzebuje `VSKN`. Skóra ma najwyżej 256
  jointów; większą grupę narzędzie dzieli na kilka prymitywów.
- **Sloty świateł (0x400).** Wierzchołek wskazuje slot (`u8`, pole `slot` komponentu `LSLT`) i rolę: zapalony (świeci i
  jest widoczny, gdy slot się świeci) albo zgaszony (widoczny, gdy slot jest zgaszony). Stan slotu leży w buforze stanów
  instancji, więc model ze światłami da się instancjonować. Slotu, którego model nie ma, shader nie zapala.
- **Węzły zostają.** Scalenie przenosi tylko geometrię; węzły i ich nazwy zostają w `NODE`, więc `GetFromName`, `.mmd` i
  eventy działają dalej. Ukrycie węzła (`visible`) to zerowa macierz jego jointu w buforze póz.
- Sfera prymitywu obejmuje pozę bindu. Runtime dla prymitywu z ruchomymi jointami bierze granice instancji.

Łączy narzędzie (`optimize`), nie konwerter: łączy węzły nieanimowane i bez komponentów, a węzły, do których coś odwołuje
się po nazwie, zostawia jako jointy. Części przezroczyste, ze skórkami wymiennymi innymi niż reszta oraz z innym zakresem
widoczności nie są łączone.

### Wierzchołek

- Pozycja `f32 × 3` względem originu modelu.
- QTangent: kwaternion obrotu układu (T, B, N), w którym B = N × T. Kwaternion ma w ≥ 1/32767. Gdy bitangent jest
  odwrócony (lustrzane UV, `tangent.w < 0` w `basic_vertex`), cały kwaternion jest zanegowany, więc w < 0. Dekodowanie:
  N = trzecia kolumna, T = pierwsza, B = N × T · znak(w).
- UV0 `f32 × 2`; half traci precyzję przy teksturach powtarzanych setki razy.

Rozmiar: 28 B (sztywny), 32 B (w prymitywie scalonym, z `VBND`), 36 B (skinowany), wobec 48 B `basic_vertex`. Przebieg
cieni czyta tylko `VPOS` (i `VBND` w prymitywach scalonych).

### Materiał (80 B)

| Offset | Typ | Pole |
|---|---|---|
| 0 | u32 | nazwa `.mat` albo tekstury (string); brak = tylko kolory |
| 4 | i32 | skórka wymienna −1..−4, 0 = stała |
| 8 | u32 | flagi |
| 12 | f32 | próg selfillum (`fLight` z E3D) |
| 16 | f32 × 4 × 4 | ambient, diffuse, specular, emission |

### Grupa wierzchołków (32 B)

`u32` nazwa, `u32` siatka, `u32` pierwszy element w `VGIT`, `u32` liczba elementów, `u32` flagi (0x1 elementy to
pary (pierwszy, liczba), 0x2 wagi w `VGWT`), `u32 × 2` komponenty grupy (pierwszy, liczba), `u32` 0. Indeksy wierzchołków
są w strumieniach pliku (bezwzględne).

## Komponenty

Rekord w `CMPT` (20 B):

| Offset | Typ | Pole |
|---|---|---|
| 0 | u32 | typ (FourCC) |
| 4 | u16 | wersja danych |
| 6 | u8 | rodzaj celu: 0 model, 1 węzeł, 2 siatka, 3 grupa wierzchołków |
| 7 | u8 | flagi |
| 8 | u32 | indeks celu |
| 12 | u32 | offset danych w `CDAT` (wyrównany do 4) |
| 16 | u32 | rozmiar danych |

Komponenty jednego węzła leżą obok siebie, a węzeł wskazuje swój zakres; tak samo grupa wierzchołków. Nieznany typ gra
pomija, a narzędzia przenoszą go bez zmian. Nowa wersja typu dopisuje pola na samym końcu danych, za listami, więc
starszy czytnik czyta swoją część od początku i pomija resztę.
Odwołania wewnątrz danych to indeksy: string, węzeł albo komponent; `0xFFFFFFFF` = brak.

| Typ | Dane (wersja 1) |
|---|---|
| `PROP` | `u32 count`, potem rekordy 24 B: `u32` klucz (string), `u8` typ (0 bool, 1 int, 2 float, 3-5 vec2-4, 6 string, 7 węzeł, 8 komponent), 3 B zera, 16 B wartości |
| `SOCK` | gniazdo, 36 B: `u32` rola (string), `u8` strona (0 brak, 1 przód, 2 tył), `u8` 0, `u16` indeks, pozycja `f32 × 3`, obrót `f32 × 4` (w przestrzeni węzła) |
| `JOIN` | joint, 36 B: `u8` rodzaj (0 stały, 1 zawias, 2 kulisty, 3 przesuw, 4 śruba), `u8` czy ograniczony, `u16` 0, `u32` rola, oś `f32 × 3`, min, max, spoczynek, skok śruby (m na obrót) |
| `SPRG` | sprężyna, 52 B: `u8` tryb (0 follow, 1 stretch, 2 aim, 3 wahadło), `u8` ruch boczny, `u16` 0, kotwica A (węzeł, gniazdo), kotwica B (węzeł, gniazdo), oś `f32 × 3`, długość swobodna, skok min, max, długość wahadła, tłumienie wahadła |
| `CHAN` | łańcuch (wąż, kabel, miech), 32 B: `u8` rodzaj (0 wąż hamulcowy, 1 zasilający, 2 kabel, 3 miech, 4 inny), `u8` 0, `u16` liczba jointów, `i32` skóra, min. promień gięcia, sztywność, masa głowicy, długość sztywnej głowicy, gniazda końców A i B; potem węzły jointów `u32 × n` i długości segmentów `f32 × (n − 1)` |
| `VARI` | grupa wariantów: `u32` nazwa, `u16` liczba stanów, `u16` stan domyślny; stany po 12 B (`u32` nazwa, `u32` pierwszy, `u32` liczba węzłów), potem lista węzłów `u32` |
| `CHNL` | kanał sterowania: `u32` nazwa, `f32` min, max, wartość początkowa, `u16` liczba powiązań, `u16` 0; powiązania po 16 B: `u8` rodzaj (0 joint, 1 klip, 2 warianty, 3 slot światła), 3 B zera, `u32` komponent celu, `f32` skala, `f32` przesunięcie (wartość celu = kanał · skala + przesunięcie) |
| `LSLT` | slot światła: `u16` slot, `u16` 0, `f32` stan początkowy (kodowanie z eventu `lights`), kolor `f32 × 3` (ujemny: kolor źródeł), `u16` liczba węzłów zapalonych, zgaszonych, źródeł `LGHT`, `u16` 0; potem te trzy listy `u32` |
| `LGHT` | źródło światła, 52 B: `u8` rodzaj (0 punkt, 1 reflektor), `u8` flagi (0x1 shadeself, 0x2 bez cieni, 0x4 soft falloff, 0x8 tylko oświetlenie, 0x10 aureola), `u16` 0, kolor `f32 × 4`, natężenie, promień, aureola od, do, `i32` typ zaniku, promień zaniku, cos stożka, cos plamy |
| `EMIT` | emiter, 16 B: `u8` rodzaj (0 dym, 1 para, 2 iskry), 3 B zera, kierunek `f32 × 3` |
| `MREF` | model wpięty w węzeł, 12 B: ścieżka (string), gniazdo (komponent), skórka (string) |
| `COLL` | bryła, 48 B: `u8` kształt (0 prostopadłościan, 1 kula, 2 kapsuła), 3 B zera, `u32` rola, środek `f32 × 3`, obrót `f32 × 4`, półwymiary `f32 × 3` |
| `LANM` | animacja E3D (`TAnimType`): `u32` |
| `LTXT` | wyświetlacz tekstu E3D (`TP_TEXT`); dzieci węzła to znaki nazwane swoim znakiem. Bez danych. |
| `STAR` | gwiazdy E3D (`TP_STARS`): `u32 count`, potem kolory RGB `u32` (jak w E3D), po jednym na wierzchołek siatki węzła |
| `LMAT` | materiał węzła bez geometrii (punkt świetlny, rotator), który E3D trzyma w każdym submodelu: `u32` slot `MATL`. Z niego światło bierze próg zapalenia (`fLight`), kolory i materiał |

## Konwersja z E3D i T3D

`eu07 -e4d convert <wejście.e3d|.t3d> <wyjście.e4d> [dynamic]` wczytuje model istniejącym loaderem (jak `-e3d`) i zapisuje
go w E4D:

- Submodel staje się węzłem. Kolejność dzieci i nazwy zostają, więc `GetFromName`, `.mmd` i eventy działają po staremu.
- Macierz submodelu jest rozkładana na TRS. Gdy TRS jej nie odtwarza (ścinanie), pełna macierz idzie do `XFRM`.
- `iFlags`, zakres widoczności i kolory zostają w węźle i w slocie materiału.
- `TP_FREESPOTLIGHT` → `LGHT`; `Light_On##` / `Light_Off##` → `LSLT`; `smokesource_*` → `EMIT`; animacja `anim:` → `LANM`;
  `TP_TEXT` → `LTXT`; `TP_STARS` → `STAR`.
- Indeksy są zapisywane jako `u16`, gdy prymityw ma mniej niż 65 536 wierzchołków.

Pozostałe polecenia: `eu07 -e4d dump <plik.e4d>` (zawartość jako tekst) i `eu07 -e4d validate <plik.e4d>` (sprawdzenie
pliku wraz z hashem zawartości).

## Zgodność wstecz

E3D i T3D są czytane jak dotąd. Gra zacznie wczytywać `.e4d` w etapie 2, i tylko wtedy, gdy plik istnieje, za
przełącznikiem w `eu07.ini`. Obecne wydanie nie czyta `.e4d`.
