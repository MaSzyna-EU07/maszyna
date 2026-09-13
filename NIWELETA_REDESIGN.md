# Odcinki niwelety — jak to powinno być zrobione

Dokument projektowy, nie opis stanu. Siostrzany do `TURNOUT_REDESIGN.md`, który
w §2.2 i §3.2 nazwał ten problem i odłożył go. To jest ten odłożony krok.

Teza: model nie umie powiedzieć rzeczy najprostszej — **„tu zaczyna się łuk”**.
Wszystko inne jest skutkiem.

## 1. Co jest dziś

Fakty, nie opinie:

- **Krzywa nie jest bytem.** Gap *i* jest zawsze między prostą *i* a *i+1*.
  Twardo założone w `align_gap_fits` (przepisuje fity na gęstą tablicę
  `straights-1`), w `editor.cpp` (`fits(n-1)`, prosta przycinana wyłącznie przez
  dwa sąsiadujące fity), w `basket_draft.cpp` i w `planpanel.cpp`.
- Stąd **wyjątki**: `hidden` prosta-prowadnica, tryb 5, i `BasketDraft` —
  czwarta reprezentacja ciągu (obok `NiweletaSpec`, `FitResult` i `RibbonRequest`),
  która przy commicie musi **dorobić sztuczną 20-metrową prostą wyjściową**, żeby
  kosz miał na czym stanąć.
- **Referencje pozycyjne.** `GapFit::gap`, `rel_niw`, `rel_str`, `frog_of`,
  `Junction::through/branch` to indeksy w wektorze; wstawienie prostej wymaga
  ręcznego przenumerowania w dwóch pętlach. Mocne `StraightId`/`NiweletaId`
  istnieją w `domain/value_objects/` i **nie są używane**.
- **Dwa źródła prawdy o współrzędnych.** Dokument trzyma to, co narysowane;
  solver zwraca `rendered_straights` i `pinned`, a host musi wpisać `pinned` z
  powrotem do dokumentu — inaczej „dokument i rysunek się rozjeżdżają”.
- **Solver zjada intencję.** Nieudany fit wraca jako `nullopt` (każdy `throw`
  z `FittingService` jest połykany przez `catch (...)`) i znika z `applied_fits`.
  Bez powodu, bez liczby. `basket_draft` musi robić próbne `solve_project`
  i sprawdzać, czy jego fit „przeżył”.
- **Brak porządku rozwiązywania.** Stała pętla 3 przebiegów relaksacji
  i sztywne dwa przejścia w `solve_layout`. Rozjazd na odnodze rozjazdu nie ma
  prawa się zbiec.
- **Druga, martwa warstwa.** `domain::Niweleta` + `HorizontalAlignment` +
  `MapProject` + cały profil: host nie zna ani jednego z tych typów,
  `to_map_project` fabrykuje placeholderowe łuki R=300 tylko po to, żeby przejść
  walidację złącz, a reader i tak przewija ten blok bez czytania.
- `Junction::pre_blade/blade_angle/blade_length` nie są zapisywane ani czytane —
  po `load` zerują się.

## 2. Jak to robią inni

- **IFC 4.3** — `IfcAlignmentHorizontal/Vertical/Cant`, każdy jako ciąg
  `IfcAlignmentSegment`: `StartDistAlong`, `SegmentLength`, promień na początku
  i na końcu, typ `LINE|CIRCULARARC|CLOTHOID`. Styczność jest **regułą walidacji
  między segmentami**, nie własnością reprezentacji.
- **LandXML** — `CoordGeom` = `Line|Curve|Spiral`; profil osobno (`ProfAlign`:
  `PVI|CircCurve|ParaCurve`). Poziom i pion to dwa niezależne ciągi nad wspólną
  stacją.
- **Bentley OpenRoads** — dwa tryby autorstwa nad **jednym** modelem elementów:
  *by PI* (wierzchołki i promienie — nasze „proste i luki”) oraz *by Element*
  (dokładanie odcinków — nasz `BasketDraft`). Element niesie *rules*: styczny do
  X, przez punkt P, offset od Y. To jest brakujące ogniwo.
- **Civil 3D Rail, „Diverted Profiles”** — geometria odnogi jest **wyprowadzana**
  z toru zasadniczego w rozjeździe, nie autorowana niezależnie i doklejana.
- **railML** — wszystko parametryzowane pozycją wzdłuż toru, nigdy indeksem
  elementu.

Wniosek wspólny: **odcinek (k0, k1, L) jest jednostką, stacja jest parametrem,
styczność jest własnością reprezentacji, a „jak ten odcinek jest wyznaczony” jest
osobnym, jawnym polem.** Kernel do tego już mamy: `geometry::layout_segment`.

## 3. Model docelowy

Płasko, w `editor::plan`, Id jako mocne typy.

### 3.1. Odcinek jest bytem

```cpp
enum class Kind { Line, Arc, Clothoid };

using Determination = std::variant<
    FixedLength,   // długość podana wprost
    ToLine,        // biegnij aż dojdziesz do tej prostej kierunkowej
    ToPoint,       // przez ten punkt
    Remainder      // pochłoń resztę odchylenia
>;

struct Element { ElementId id; Kind kind; double radius; int hand; Determination det; };
```

Odcinek zaczyna się tam, gdzie kończy się poprzedni — **pozą**, nie
współrzędnymi. Styczność przestaje być regułą do sprawdzania i staje się
własnością zapisu. Znikają `JointContinuity`, `validate_joint`,
`pin_branch_to_frog`, `rendered_straights` i cały dryf.

### 3.2. Prosta kierunkowa oddzielona od toru

```cpp
struct ConstructionLine { LineId id; double x1,y1,x2,y2; Binding binding; };
```

Dziś `StraightSpec` jest jednocześnie prostą kierunkową i kawałkiem toru — to
jest źródło dwóch prawd o współrzędnych. Prosta kierunkowa nigdy nie jest torem;
torem jest `Element{Line, ToLine{id}}`. `hidden` przestaje być potrzebne:
prowadnica to linia, na której nie leży żaden element.

### 3.3. Kotwica i porty — graf zamiast przypinania

```cpp
struct Track { TrackId id; std::string name; Anchor anchor; std::vector<Element> elements; };
using Anchor = std::variant<AtLine, AtPort, AtPose>;
```

`frog_of` → `AtPort`. Znikają `place_frog_straight`, `LayoutSolution::pinned`
i zapis solvera do dokumentu.

### 3.4. Rozjazd: typ + umiejscowienie

`TURNOUT_REDESIGN.md` §3.3, teraz odblokowane — krzyżownica jest portem:

```cpp
struct TurnoutType      { name, crossing_n, radius, length, pre_blade, blade_angle, blade_length };
struct TurnoutPlacement { id, type, on, station, hand, facing };
```

Naprawia przy okazji niezapisywane liczby iglicy: idą do katalogu, egzemplarz
trzyma nazwę typu.

### 3.5. Solve — czysta funkcja, po Id, z diagnostyką

```cpp
struct Solution { map<TrackId,SolvedTrack>; map<TurnoutId,TurnoutGeom>; vector<Diagnostic>; };
Solution solve(const Document&);
```

1. **Nic nie wraca do dokumentu.** Host rysuje `Solution` i w nią trafia.
2. **Nieudany fit to diagnostyka, nie cisza.** Geometria do tego odcinka zostaje
   położona, łańcuch urywa się i jest rysowany jako kikut, a użytkownik dostaje
   powód i liczbę.
3. **Kolejność z DAG-u.** Zależności (kotwice, `ToLine`, bindingi, rozjazdy) dają
   graf; sortowanie topologiczne zastępuje 3 przebiegi relaksacji i sztywne dwa
   przejścia. Cykl → diagnostyka, nie śmieci.

### 3.6. Dwa tryby autorstwa nad jednym ciągiem

To jest właściwy zysk. Dokładanie odcinek po odcinku i fitowanie przestają być
dwoma światami (`BasketDraft` kontra `GapFit`) i stają się dwoma sposobami
wypełnienia pola `Determination`.

**Odcinek po odcinku** (`FixedLength`, `ToPoint`) — start z kotwicy, każdy
kolejny odcinek styczny do pozy końcowej poprzednika. Trzy rzeczy dziś
niewyrażalne wychodzą same:

- **łuk wprost z krzyżownicy** — bez prostej KR, bez prowadnicy;
- **łuk po łuku** bez prostej między nimi;
- **ciąg, który się nie domyka** — bez sztucznej prostej wyjściowej.

Każdy dołożony odcinek idzie od razu do dokumentu, nie do bufora szkicu. Cofanie
to usunięcie ostatniego elementu. Podgląd przestaje kłamać: dziś `draft_to_ribbon`
liczy kąt każdego łuku jako `L/R`, a `fit_compound` daje ostatniemu łukowi resztę
odchylenia — to są **różne krzywe**.

**Fitowanie** (`ToLine`, `Remainder`) — między dwoma odcinkami `ToLine` łańcuch
z dokładnie jednym `Remainder`:

| chcesz | łańcuch |
|---|---|
| sam łuk (tryb 1) | `Arc{R, Remainder}` |
| łuk z klotoidami (tryb 2) | `Clot{R,L}` → `Arc{R, Remainder}` → `Clot{∞,L}` |
| łuk koszowy (tryb 3) | `Arc{R₁,L₁}` → `Clot` → … → `Arc{Rₙ, Remainder}` |
| nawrót 180° (tryb 4) | ten sam łańcuch — nawrót wynika z geometrii |
| S-krzywa | `Arc{R,+1,L}` → `Arc{R,-1, Remainder}` |

Tryby `GapFit` 0–4 znikają: to były nazwy dla pięciu kształtów łańcucha.

**Mieszanie w jednym ciągu** — dziś nie ma tego w ogóle, bo wszystko między
dwiema prostymi musi być jednym `GapFit`:

```
anchor  port turnout=3 KR
elem 11 arc  190 -1 len 40      ← odcinek po odcinku, prosto z krzyżownicy
elem 12 clot 600 -1 len 30
elem 13 arc  600 -1 rem         ← domknięcie fitem…
elem 14 arc  400  1 rem         ← …dwoma łukami, bo start jest sztywny…
elem 15 line 0    0 toline 7    ← …na prostą kierunkową 7
```

**Stopnie swobody zamiast wyjątków.** Domknięcie to dwa równania: wylądować na
prostej i być do niej stycznym. Łańcuch wchodzący *wzdłuż prostej* może się po
niej przesuwać — to jeden stopień swobody — więc potrzebuje dokładnie **jednego**
`Remainder`; to jest każdy dzisiejszy fit. Łańcuch wychodzący ze **sztywnej pozy**
(kotwica, zwykle krzyżownica) przesuwać się nie może, więc potrzebuje **dwóch**.
Łańcuch, który nie domyka się na nic — **zera**.

Solver to liczy i mówi wprost: za mało → „przewiązany, brakuje N stopni swobody”;
za dużo → „niedookreślony, podaj długość jednego z łuków”; `Remainder ≤ 0` → ile
odchylenia było do rozdania i ile zużyły odcinki o zadanej długości.

Uwaga na marginesie: odchylenie bierzemy z krótkiej gałęzi `atan2`, więc łuk
domykający nigdy nie skręci dalej, niż narożnik naprawdę się załamuje. Jazda
dookoła — nawrót powyżej 180°, pętla — jest czymś, co się **kładzie** zadanymi
długościami, nie czymś, co fit ma po cichu za nas wybrać.

### 3.7. Format `.m0s` v2

Jawny numer wersji zamiast przemianowywania tagów, wszystko po Id, bez doklejania
tokenów na koniec linii. Bez zgodności wstecz.

## 4. Co zostaje bez zmian

Warto powiedzieć wprost, bo to nie jest przepisywanie wszystkiego:

- `geometry::layout_segment` — jedyny ewaluator odcinka, zostaje.
- `domain::lay_turnout`, `Turnout`, `TurnoutGeometry` — zostają.
- Matematyka fitowania (`solve_chain`, `fit_compound`) — zmienia się interfejs
  (powód zamiast `throw`/`nullopt`), nie liczby.
- `ParallelismService::project_onto_offset_line` — jako `Binding::Parallel`.
- `RailRenderer`, `TrackRenderer`, kafle, georeferencja, eksport `.scn` — bez
  związku z modelem.

Do wyrzucenia jest **klej**: przypinanie, prowadnice, tryby jako wyjątki,
`BasketDraft` i martwa warstwa agregatowa.

## 5. Poza zakresem

**Niweleta w profilu (pion).** `VerticalAlignment`, `ProfileElement`, `Grade`,
`VerticalCurve` istnieją, ale nic ich nie liczy, `.m0s` ich nie niesie, a
`scn_export` i tak wpisuje `y=0.2` i `roll=0` dla każdego węzła — mimo że format
`.scn` niesie `z` na obu końcach segmentu i `roll1`/`roll2` interpolowane
liniowo. Lecą jako martwy kod i wracają jako osobny temat: ten sam kształt
ciągu, o wymiar niżej, nad tą samą stacją.
