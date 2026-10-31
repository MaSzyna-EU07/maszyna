# Nierówności toru

Opcjonalne atrybuty wpisu toru, przed `endtrack`:

```
node -1 0 tor1 track normal 100.0 1.435 0.15 25.0 20 0 flat vis
rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1
0.0 0.2 0.0 0.0
0.0 0.0 0.0
0.0 0.0 0.0
0.0 0.2 100.0 0.0
0.0
uneven 18
unevencross 25
jointdip 6
endtrack
```

| atrybut           | dotyczy               | opis                                                                                  |
|-------------------|-----------------------|---------------------------------------------------------------------------------------|
| `uneven <mm>`     | `normal`, `switch`    | największa odchyłka toku szynowego na cięciwie 10 m; 0-100, domyślnie 0               |
| `unevenup <mm>`   | `normal`, `switch`    | składowa w pionie, osobno od `uneven`                                                 |
| `unevenside <mm>` | `normal`, `switch`    | składowa w poziomie, osobno od `uneven`                                               |
| `unevencross <mm>`| `normal`              | składowa przechyłki, osobno od `uneven`; rozjazd jej nie ma                           |
| `jointdip <mm>`   | `normal`              | głębokość zapadnięcia na styku szyn; 0-100, domyślnie 0,4 składowej w pionie          |

Składowa niepodana przyjmuje wartość `uneven`. Na przykład sam bok to `unevenside 30`, a same zapadnięcia na stykach to `jointdip 6`.

Wszystkie są też w edytorze, w panelu toru, w sekcji „Ride and sound”. Na drogach, rzekach i obrotnicach są ignorowane.

## Jaką wartość wybrać

`uneven` odpowiada kolumnie „nierówności pionowe” z instrukcji PLK Id-14, załącznik 1 (klasa C):

| prędkość [km/h] | 20 | 40 | 60 | 80 | 100 | 120 | 160 |
|-----------------|----|----|----|----|-----|-----|-----|
| `uneven` [mm]   | 50 | 35 | 25 | 18 | 14  | 10  | 6   |

Tor w dobrym stanie to według instrukcji połowa tej wartości (klasa A), w dostatecznym 3/4 (klasa B).

Tor z `uneven 18` mierzy się tak (`tests/unit/test_track_uneven.cpp`), w nawiasie limit Id-14 dla 80 km/h:

| parametr                  | wynik   | Id-14   |
|---------------------------|---------|---------|
| nierówności pionowe       | 18 mm   | 18 mm   |
| nierówności poziome       | 19 mm   | 17 mm   |
| wichrowatość na bazie 5 m | 15 mm   | 16 mm   |
| szerokość toru            | ±10 mm  | +10/-8 mm |
| przechyłka                | 20 mm   | 20 mm   |
| wskaźnik J                | 6,1     | 5,3     |

## Jak to działa

Nierówność to pole przesunięć zależne od położenia na mapie, złożone z fal o długości 5-42 m. W pionie krótkich fal jest mniej, żeby częściej dochodziły blisko wartości szczytowej, a do tego są fale 47-83 m: na cięciwie 10 m prawie się nie liczą, pudło unosi się na nich i opada, a pantograf za nimi nadąża. Nie ma w nim losowania: to samo miejsce wygina się tak samo przy każdym uruchomieniu. Przesunięty tor dostaje fale z nowego miejsca.

Każda składowa łączy się osobno:

- **Tor z torem:** na styku obowiązuje mniejsza z dwóch wartości, a każdy tor dochodzi do niej płynnie na 20 m. Wolny koniec toru leży w projekcie.
- **Rozjazd:** wszystkie jego drogi mają jedną wartość. Rozjazdy złączone bezpośrednio przyjmują najmniejszą ze swoich. Dołączony tor dochodzi do wartości rozjazdu, także większej od własnej. Tor bez danej składowej trzyma w niej cały rozjazd w projekcie.
- **Styki:** zapadnięcia są co `fSoundDistance`, na długości 6 m, i biegną dalej przez kolejne tory o tym samym odstępie. Stukot osi wypada w miejscu zapadnięcia. Przy odstępie 6 m lub mniejszym oraz na rozjazdach zapadnięć nie ma.
- **Przechyłka:** dolicza się do projektowej. Jest pochyleniem w planie mapy, więc tory wpisane ku sobie spotykają się na tej samej przechyłce. Na rozjazdach tylko ta wynikająca z fal, bez osobnego siadania toków.

Końce złączonych dróg spotykają się w środku między sobą, z jednym kierunkiem poprzecznym, jeśli są bliżej niż 20 cm, co zamyka pęknięcia z niezgodnych współrzędnych w danych, także na torach bez nierówności. Przechyłkę na takim styku rozjazd bierze od zwykłego toru, a dwa tory albo dwa rozjazdy spotykają się w połowie. To dotyczy też fizyki.

Za nierównością idą szyny, wierzch podsypki, `sleepermodel`, osie, przechył pudła i bujanie kabiny. Skarpa podsypki przejmuje różnicę, a jej spód zostaje w projekcie, na terenie.

Nie idą za nią modele z includów (np. napędy i podkłady rozjazdów), teren ani sieć trakcyjna, więc przy dużych wartościach szyna rozjeżdża się z takim modelem o kilka centymetrów.

Na torze z nierównością kabinę buja sam tor. Odpada kołysanie wyliczane z przyspieszenia poprzecznego, czyli z promienia łuku, oraz losowe szarpnięcia przy większej prędkości. Na torach bez nierówności działają jak dotąd.

Pantograf podnosi się z szybkością ciśnienia, ok. 0,14 m/s, a szczelina do drutu większa niż 1 cm odcina napięcie i wybija wyłącznik szybki. Nierówność dokłada się do tego przy dużej prędkości. Na torach Bałtyku symulacja daje szczelinę 0,6 cm, gdy składowa w pionie razy prędkość w km/h wynosi 925, czyli np. 8 mm przy 120 km/h, 13 mm przy 70 km/h i 23 mm przy 40 km/h. Wartości z tabeli Id-14 są od tego większe. Sam drut też to potrafi: na Bałtyku przy `tor0290` wznosi się o ok. 1%, więc pantograf nadąża tam do ok. 46 km/h, a powyżej wyłącznik wybija także na prostym torze.

Fizyka bierze promień łuku i pochylenie z projektu, więc nierówności nie zmieniają tych wartości, a do wykolejeń dokładają tylko przechyłkę, ok. 0,06 m/s² przy `uneven 18` wobec progu ok. 3,2 m/s².

## Wydajność

Tor z nierównością ma geometrię co 2 m zamiast co 10 m na prostych. Rozjazdy zachowują swój podział.

Zmierzone (kamera w ruchu lub kabina, 1920×1080):

| scena                                 | bez flag           | z flagami 18/35    |
|---------------------------------------|--------------------|--------------------|
| Bałtyk, 56 km torów: wierzchołki      | 0,56 mln (ok. 27 MB) | 2,17 mln (ok. 104 MB) |
| Bałtyk: najdłuższa klatka w przelocie | 31,5 ms            | 64 ms              |
| `td.scn`, jazda 120 km/h: klatka      | 2,73 ms            | 2,93 ms            |

Koszt rośnie z liczbą oflagowanych torów, a najdłuższe klatki pojawiają się przy budowaniu geometrii kolejnych sekcji. Sama jazda po takim torze praktycznie nie kosztuje: ok. 0,3 µs na oś w kroku fizyki.
