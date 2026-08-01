# Rozjazd — kontrakt geometrii dla runtime i modeli

Jak edytor przekazuje rozjazd, a runtime ustawia z tego elementy. Podstawą są
**krzywe Béziera** (kubiczne). To jest warstwa graniczna: wewnątrz edytor liczy
łuki i klotoidy (kolejowo dokładne), a na wyjściu przybliża je Bézierami z
zadaną tolerancją. Runtime nie zna „typów" rozjazdów — dostaje instancję i sadzi
po niej elementy.

## Zasada

Runtime **nie bazuje na szablonie** (Rz 1:9 R190 itd.). Szablony to tylko preset
w edytorze, który wypełnia liczby. Każdy rozjazd to instancja o dowolnej
geometrii (własny skos, promień, przesunięty początek, łuk koszowy w środku).
Runtime dostaje **rozwiązaną geometrię** i z niej wszystko wyprowadza.

## Układ i jednostki

- metry; krzywe w XY (płaszczyzna planu),
- rozstaw toru 1435 mm → półrozstaw **0,7175 m**,
- pozy podawane jako pozycja + jednostkowy kierunek `(hx, hy)` (styczna, „w
  kierunku jazdy"). Normalna lewa = `(-hy, hx)`,
- każdy element modelu ma origin w punkcie wstawienia, **+X = kierunek jazdy**,
  **+Y = w lewo**, **+Z = w górę**. Runtime wstawia go podając tylko pozę —
  żadnego per-model obrotu.

## Instancja rozjazdu

```
Turnout {
    side          : Left | Right          // strona odgałęzienia (mirror)
    crossing_mark : n                      // skos 1:n  -> dobór odlewu krzyżownicy
    rail_profile  : "49E1" | "60E1"        // -> dobór iglic / profilu szyny
    gauge         : 1.435

    PR : Pose      // początek rozjazdu: styczny do toru zasadniczego
    KR : Pose      // koniec (krzyżownica): kierunek = tor zasadniczy obrócony o alfa
    crossing_angle : alfa = atan(1/n)      // radiany

    blade_angle      : beta                // kąt nagięcia iglicy (kąt zwrotnicowy)
    pre_blade_length : d                   // odcinek przediglicowy: prosty, PR -> ostrze iglicy

    through_axis  : [Bezier...]            // tor zasadniczy przez rozjazd (zwykle prosty)
    diverging_axis: [Bezier...]            // przediglicowy -> iglica(prosta @ beta) -> luk -> prosta -> KR
}

Pose   { x, y, hx, hy }                    // pozycja + jednostkowa styczna
Bezier { p0, p1, p2, p3 }                  // kubiczna, punkty kontrolne w XY (metry)
```

Tor zasadniczy nie jest dzielony — `through_axis` jest podany dla kompletności
(runtime często ma już tor główny). Rozjazd wnosi przede wszystkim
`diverging_axis` + osprzęt + podrozjazdnice spinające oba tory.

## Początek odnogi: odcinek przediglicowy + iglica

Odnoga **nie zaczyna się łukiem**. To zwrotnica sieczna z iglicą prostą:

1. **odcinek przediglicowy** — prosta w kierunku toru zasadniczego (kąt 0), od
   `PR` do ostrza iglicy. Tu rozjazd jest jeszcze równoległy do toru głównego;
   tu siedzi napęd.
2. **iglica** — **prosta odcięta pod kątem `beta`** („prosta dogięta do
   prostego"). Odnoga odchodzi od toru zasadniczego dopiero tutaj, o kąt nagięcia
   iglicy `beta`.
3. **łuk** — promień `R`, skręca od `beta` do kąta krzyżownicy `alfa`, czyli o
   **`alfa − beta`** (nie o całe `alfa`).
4. **prosta krzyżownicowa** — pod kątem `alfa`, do `KR`.

Stąd `diverging_axis` = `[przediglicowy] [iglica @ beta] [łuk] [prosta] `. Iglice
łukowe (styczne, `beta ≈ 0`, ostrze wtopione w łuk) to inny wariant — na razie
modelujemy iglicę prostą.

**Model:** zespół iglic kotwiony w ostrzu (`PR` + `pre_blade_length` wzdłuż toru
głównego), zorientowany wzdłuż toru zasadniczego, nadaje `beta`; długość iglicy
jest po stronie modelu.

## Bézier — definicja i próbkowanie

Kubiczna:

```
B(t)  = (1-t)^3 p0 + 3(1-t)^2 t p1 + 3(1-t) t^2 p2 + t^3 p3,   t in [0,1]
B'(t) = 3(1-t)^2 (p1-p0) + 6(1-t)t (p2-p1) + 3t^2 (p3-p2)
```

- **pozycja** = `B(t)`, **kierunek** = `normalize(B'(t))`, `heading = atan2(B'y, B'x)`.
- styczna na końcach: w `t=0` wzdłuż `p1-p0`, w `t=1` wzdłuż `p3-p2` — dlatego
  uchwyty `p1`, `p2` leżą **na stycznych toru** w węzłach,
- kolejne segmenty są **G1** (styczne się zgadzają) — geometria edytora jest
  styczna z konstrukcji, więc łączenia są gładkie.

**Uwaga o parametryzacji:** `t` **nie jest** długością łuku. Do równego rozstawu
podrozjazdnic/podkładów trzeba reparametryzować po długości: całkować `|B'(t)|`
(albo próbkować adaptacyjnie) i szukać `t` dla zadanej odległości. Nie rozstawiaj
elementów po stałym kroku `t`.

## Skąd biorą się Béziery (aproksymacja)

**Reguła: jeden element osi = jeden Bézier.** Bez dzielenia. Węzły wyznaczają
`p0`/`p3` i styczne w węzłach, uchwyty leżą na tych stycznych:

- **prosta** → `p1`, `p2` na linii między `p0` a `p3` (uchwyty w 1/3 długości).
- **łuk** kąta `theta`, promień `R` → uchwyty wzdłuż stycznych o długości

  ```
  k = (4/3) * R * tan(theta/4)
  ```

  Dla kątów rozjazdowych (`theta < 8°`) błąd jest rzędu ułamka milimetra.
- **klotoida** (krzywa przejściowa, jeśli użyta) → też **jeden Bézier**: `p0`,
  `p3` i styczne z końców klotoidy, uchwyty na stycznych w 1/3 cięciwy. Kubiczna
  nie odda liniowo zmiennej krzywizny co do joty, ale przy długościach
  przejściówek w rozjeździe odchyłka jest mała i to akceptujemy — jeden segment,
  bez podziału.

Styczne w węzłach są wspólne dla sąsiednich elementów, więc Béziery łączą się G1.

## Szyny — krzywe przesunięte

Szyny to osie przesunięte o `±0,7175 m` wzdłuż normalnej lewej. **Nie** trzymaj
ich jako osobnych Bézierów — przesunięcie Béziera nie jest Bézierem. Runtime
próbkuje pozę osi i offsetuje per-próbka:

```
lewa  = B(t) + (-hy, hx) * 0.7175
prawa = B(t) - (-hy, hx) * 0.7175
```

Na promieniach rozjazdowych rozjeżdżanie się offsetu jest pomijalne.

## Podział elementów

**Ciągłe — sadzone z pozy wzdłuż osi (zero zależności od typu):**

- **szyny** — po obu osiach, offset jak wyżej; potrzebny profil przekroju,
- **podrozjazdnice** — po długości (reparametryzacja!), prostopadle do
  dwusiecznej obu osi, wydłużane w strefie iglic i krzyżownicy tak, by objąć oba
  tory. Parametryczne (skalowana długość) albo zestaw dyskretny — do ustalenia.

**Dyskretne odlewy — dobierane parametrem, nie nazwą szablonu:**

- **krzyżownica** — fizyczny odlew o kącie `alfa`; dobierana po **`crossing_mark`
  (skos 1:n)**, wstawiana w `KR`, ustawiona pod `crossing_angle`,
- **iglice (zwrotnica)** — dobierane po **`rail_profile`** (i ew. `R`), wstawiane
  w `PR`,
- **kierownice** — dowiązane do `KR`,
- **napęd + latarnia** — w `PR`, odsunięte w bok o ustalony offset.

Runtime: geometrię szyn/podrozjazdnic robi z Bézierów, a krzyżownicę i iglice
**wybiera po (`crossing_mark`, `rail_profile`)** i wstawia w `PR`/`KR`.

## Iglice — stany

Rozjazd się przekłada. Iglice potrzebują dwóch stanów (na wprost / na bok) albo
animacji przełożenia — do ustalenia z modelarzem. Pozycja bazowa: `PR`.

## Do ustalenia z modelarzem

1. origin w pionie: główka szyny czy spód podkładu,
2. profil: 49E1 / 60E1 / oba,
3. podrozjazdnice: parametryczne czy zestaw dyskretny,
4. iglice: ruchome (dwa stany / animacja) czy statyczne,
5. które elementy bierze modelarz, a które generuje runtime.

## Do zrobienia po stronie edytora

`JunctionGeom` ma dziś `PR (px,py)`, `KR (fx,fy)`, kierunek wyjścia `(fhx,fhy)`,
długość i osie w postaci polilinii. Do kontraktu trzeba dołożyć:

- **`crossing_mark` i `rail_profile`** w danych eksportu,
- **`blade_angle` (beta) i `pre_blade_length`** — dziś `lay_turnout` kładzie łuk
  styczny od `PR` (beta = 0, brak przediglicowego). Trzeba: odcinek przediglicowy
  (prosty), iglicę prostą pod `beta`, łuk o `alfa − beta`, prostą do `KR`,
- te dwa parametry **w szablonach** (`turnout_preset`) per typ,
- osie jako **Béziery** (dziś polilinie) — konwersja łuk/klotoida → kubiczne wg
  powyższego,
- jawne oznaczenie „to rozjazd" (nie zwykły łuk), z `PR`/`KR`/`alfa`.
