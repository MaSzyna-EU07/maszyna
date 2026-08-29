# Rozjazdy — jak to powinno być zrobione

Dokument projektowy, nie opis stanu. `TURNOUT_SPEC.md` mówi, co edytor oddaje
runtime'owi; ten mówi, **jak edytor powinien trzymać rozjazd u siebie**, bo obecny
model się nie broni i każda kolejna funkcja jest w nim przypadkiem szczególnym.

## 1. Co jest dziś

Fakty, nie opinie:

- `NiweletaSpec` = lista **prostych zakotwiczonych w XY** + `fits` w lukach między
  kolejnymi prostymi. Krzywa nie jest bytem — jest wynikiem wpasowania w lukę.
- `Junction` (rozjazd) to **relacja** na niwelecie zasadniczej: stacja + liczby
  katalogowe. Kładzie własną ścieżkę odgałęzienia (`lay_turnout`), ale ta ścieżka
  **nie należy do żadnej niwelety** — jest rysowana osobno z `JunctionGeom`.
- Odnoga to osobna niweleta, **przypinana sztywno** do krzyżownicy przy każdym
  rozwiązaniu (`pin_branch_to_frog`).
- Równolegle istnieje `BasketDraft` (395 linii) — drugi, własny mechanizm
  budowania krzywych, nie przechodzący przez model luk.
- `Junction` niesie **jednocześnie** liczby katalogowe (skos, R, długość, iglica)
  i swobodną krzywą wewnętrzną (`GapFit curve` z dowolnym koszem).
- Rozjazdy są serializowane **pozycyjnie i ręcznie**. `pre_blade`, `blade_angle`
  i `blade_length` nie są zapisywane w ogóle — doszły do modelu i wypadły z pliku.

## 2. Cztery przyczyny źródłowe

**2.1. Geometria rozjazdu leży poza grafem toru.**
Tor zasadniczy nie jest dzielony (słusznie), ale odgałęzienie rozjazdu nie należy
do niczego, a odnoga jest osobnym bytem. Ciągłość między krzyżownicą a odnogą nie
wynika z konstrukcji, tylko z **doklejania jej z powrotem przy każdym solve**.
Stąd błąd „odnoga uciekła 1,4 km", stąd konieczność oddawania `pinned` do
dokumentu, stąd szkic startujący ze starego miejsca.

**2.2. Krzywa jest obywatelem drugiej kategorii.**
Model brzmi „proste i luki między nimi". Wszystko, co nie jest łukiem między
dwiema prostymi, wymaga wyjątku: ukryta prosta-prowadnica, tryb 5, `BasketDraft`.
Dlatego „dodaj łuk wprost z krzyżownicy" było trudne — w danych **nie ma jak
powiedzieć „element: łuk, zaczyna się tutaj"**.

**2.3. Dwa źródła prawdy o współrzędnych.**
Dokument trzyma to, co narysowane; solver trzyma przypięte i przycięte
(`rendered_straights`). GUI czyta raz jedno, raz drugie. Stąd łuk startujący z
azymutem 0 i cała rodzina błędów „to nie jest tam, gdzie jest narysowane".

**2.4. Typ katalogowy zmieszany z egzemplarzem.**
„Rz 1:9 R190" ma geometrię **określoną przez typ**. Pozwalanie, by egzemplarz
miał dowolny łuk koszowy w środku, jest bez sensu — a jednocześnie to, co
naprawdę jest swobodne (dokąd biegnie tor za krzyżownicą), było najtrudniejsze do
edycji. Skutek uboczny: każda liczba katalogowa to kolejne pole doklejane do
formatu pliku.

## 3. Model docelowy

### 3.1. KR jest prostą  *(zrobione)*

Najprostsze możliwe ujęcie, i to ono wygrało: **koniec rozjazdu to zwyczajna
prosta**, tylko inaczej związana niż narysowana ręcznie.

```
StraightSpec {
    ...
    int frog_of;   // indeks rozjazdu, którego KR opuszcza ta prosta; -1 = zwykła
}
```

Początek i kierunek pisze solver z krzyżownicy, długość zostaje jej własna.
I to wszystko — **łuk za rozjazdem przestaje istnieć jako pojęcie**. Jest fitem w
luce między dwiema prostymi, jak każdy inny: to samo combo, ten sam promień, ten
sam kosz, ten sam komunikat „does not fit".

Styczność w krzyżownicy nie jest pilnowana, tylko **wynika**: prosta z KR leży na
stycznej, a fit między prostymi jest styczny z definicji.

Skasowało to: przypinanie całej odnogi, tryb 5, ukrytą kotwicę, `LeadArc`,
`StartPose` i cały `pose_fitter`. Odnoga przestała być bytem specjalnym — jest
niweletą, której pierwsza prosta ma jedną więź więcej.

Efekt uboczny, ważniejszy niż czystość: **to, co narysowane, zostaje tam, gdzie
narysowane**. Wcześniej cała odnoga jechała sztywno za krzyżownicą.

### 3.2. Element  *(do zrobienia, ale już nie dla rozjazdów)*

„Proste i luki" nadal wymaga wyjątku wszędzie tam, gdzie krzywa ma istnieć sama
z siebie — stąd `BasketDraft`. Docelowo prosta, łuk i klotoida są równorzędnymi
elementami ciągu, a to, co dziś robi szkic kosza, jest zwykłym dokładaniem
elementów. Rozjazdów to już nie dotyczy: 3.1 załatwiło je bez tego.

### 3.3. Rozjazd jako urządzenie typu  *(do zrobienia)*

Rozdzielić dwie rzeczy, dziś zlepione w `Junction`:

```
TurnoutType {                    // katalog, dane z Id-1, niezmienne
    name, crossing_n, radius, length,
    pre_blade, blade_angle, blade_length, rail_profile
}

TurnoutPlacement {               // to, co edytuje użytkownik
    type : TurnoutTypeRef        // po nazwie, nie przez kopiowanie liczb
    on : AlignmentRef, station, hand : L|P, facing
}
```

**Strefa rozjazdu jest nienaruszalna, bo jest egzemplarzem typu.** Nie da się jej
edytować, bo nie ma czego — liczby należą do typu. Rozjazd nietypowy to własny
typ, nie dowolność w egzemplarzu.

Serializacja przestaje puchnąć: egzemplarz zapisuje `type` po nazwie plus
umiejscowienie. Nowa liczba katalogowa (jak dzisiejsze `blade_angle`, które wciąż
nie trafia do pliku) idzie do katalogu i **nie dotyka formatu**.

## 4. Co to naprawia — konkretnie

| dzisiejszy problem | dlaczego znika |
|---|---|
| odnoga ucieka / dryfuje od krzyżownicy | start odnogi jest referencją do portu |
| szkic z azymutem 0 | nie ma drugiego źródła współrzędnych |
| „robi się sztywna prosta" za rozjazdem | pierwszy element odnogi może być łukiem |
| łuk wymaga dwóch prostych | element powstaje z pozy i punktu |
| kosz w środku „Rz 1:9" | geometria wewnętrzna należy do typu |
| liczby iglicy nie zapisują się | egzemplarz odwołuje się do typu po nazwie |
| `BasketDraft` obok modelu | kosz to ciąg elementów-łuków, nic osobnego |

## 5. Etapy

1. ~~Odnoga przestaje być przypinana~~ — **zrobione** przez 3.1: `frog_of`.
   Zniknęły `pin_branch_to_frog`, tryb 5, ukryta kotwica, `LeadArc` i
   `pose_fitter`.
2. **Typ oddzielony od umiejscowienia** (3.3). Katalog Id-1 jako dane; egzemplarz
   trzyma nazwę typu. Naprawia przy okazji serializację liczb iglicy.
3. **Elementy pierwszej kategorii** (3.2) i wycofanie `BasketDraft` — 395 linii
   równoległego świata.

## 6. Co zostaje bez zmian

Warto powiedzieć wprost, bo to nie jest przepisywanie wszystkiego:

- `domain::lay_turnout` i cała geometria rozjazdu — dobre, zostaje.
- `TurnoutPart` i punkty konstrukcyjne (`JunctionMark`) — zostają.
- `fit_arc_from_pose`, `fit_arc_from_pose_through` — to **są** właściwe prymitywy
  nowego modelu, przechodzą nietknięte.
- `GapFitter` i fitowanie między prostymi — zostaje, jako jeden ze sposobów
  wyznaczania elementu swobodnego.
- Rysowanie szyn, kafle, georeferencja — bez związku.

Do wyrzucenia jest przede wszystkim **klej**: przypinanie, ukryte prowadnice,
tryb 5 jako wyjątek i `BasketDraft`.
