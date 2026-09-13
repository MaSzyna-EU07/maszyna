# Binarny format scenerii („eu7") — specyfikacja zapisu na dysku

Źródło autorytatywne: [`scene/scenerybinary.h`](scenerybinary.h) i
[`scene/scenerybinary.cpp`](scenerybinary.cpp). Ten dokument opisuje format tak, jak kod go
aktualnie czyta i zapisuje (wersja na dysku: **10**). Jeśli kiedykolwiek się rozjadą,
rozstrzyga kod — wtedy zaktualizuj ten plik.

## 1. Cel

Format binarny zastępuje stare formaty tekstowe (`.scn` / `.scm` / `.inc` / `.ctr`) oraz dawny
blob terenu `.sbt`. To **przezroczysty, modularny cache**: każdy tekstowy plik scenerii
kompiluje się do dokładnie jednego binarnego „twina", konsumowanego na warstwie plikowej
`cParser`, więc deserializer scenerii nigdy nie widzi formatu. Twin przechowuje własny, w pełni
rozwinięty strumień tokenów pliku źródłowego (komentarze usunięte, liczby otypowane, stringi
zinternowane) i zachowuje dyrektywy `include` jako referencje — twin jednego pliku wskazuje na
twiny plików, które tamten dołącza.

Cele projektowe: małe pliki, **odczyt strumieniowy z bufora** (cały twin wczytany raz, potem
parsowany wskaźnikiem, a tokeny-stringi serwowane jako widoki — bez wywołań strumienia
per-bajt, bez materializacji pośredniej), równoległy/offline'owy baking oraz **progresywne
ładowanie** (najpierw infrastruktura, żeby gra mogła ruszyć, potem strumieniowanie węzłów
wizualnych w kolejności odległości od kamery).

## 2. Nazewnictwo plików (mapowanie twina)

Jeden plik źródłowy → jeden twin; rozszerzenie binarne wynika z rodzaju źródła
(`scenerybinary_extension_for()`):

| źródło  | twin    | `scenery_file_kind` |
|---------|---------|---------------------|
| `.scn`  | `.scnb` | `scn` (0)           |
| `.inc`  | `.incb` | `inc` (1)           |
| `.scm`  | `.scmb` | `scm` (2)           |
| `.ctr`  | `.ctrb` | `scn` (0)¹          |

¹ `.ctr` dostaje własne rozszerzenie `.ctrb`, ale jest tokenizowany jako rodzaj `scn` (enum
`kind` nie ma osobnej wartości dla `ctr`).

Twin jest **odtwarzany tylko, jeśli jest co najmniej tak nowy jak źródło tekstowe** (porównanie
czasu modyfikacji); edycja pliku scenerii wymusza rekompilację zamiast odtworzenia
nieaktualnego twina.

## 3. Ogólny układ pliku

```
+-----------------------------------------------------------+
| Nagłówek                                                  |
|   magic    : uint32 LE  = MAKE_ID4('e','u','7',10)        |  4 bajty
|   kind     : uint8      = scenery_file_kind               |  1 bajt
|   flags    : uint32 LE  (bit0 = infra-relevant)           |  4 bajty
+-----------------------------------------------------------+
| Tablica stringów                                          |
|   count    : varint                                       |
|   powtórz count razy:                                     |
|     length : varint                                       |
|     bytes  : <length> surowych bajtów (bez NUL)           |
+-----------------------------------------------------------+
| Strumień wpisów                                           |
|   wpisy jeden po drugim aż do końca pliku                 |
+-----------------------------------------------------------+
```

* **magic** — `"eu7"` plus jednobajtowa wersja (obecnie `10`), little-endian przez `MAKE_ID4`.
  Reader odrzuca każdy bufor, którego magic/wersja nie zgadza się dokładnie (starsze twiny są
  rekompilowane, nie odczytywane błędnie). Minimalny poprawny plik to 9 bajtów (nagłówek).
* **kind** — z jakiego formatu tekstowego ten twin został skompilowany (tabela wyżej).
* **flags bit0 (`infra-relevant`)** — ustawiony, jeśli plik ma co najmniej jeden węzeł
  infrastruktury lub jakikolwiek `include`. Czysto wizualny liść (np. `.incb` z florą: same
  kształty + dyrektywy transformacji) ma go wyzerowany, więc pass infrastrukturalny może w ogóle
  nie otwierać pliku, zamiast przechodzić go po to, żeby odrzucić jego treść wizualną.
* **tablica stringów** — każdy odrębny tekst tokenu jest internowany raz i referowany dalej
  przez swój zerobazowy indeks. Stringi mają prefiks długości (bez terminatora NUL), więc reader
  może serwować `std::string_view` wprost w bufor.

## 4. Kodowania prymitywów

* **varint** — LEB128 bez znaku: 7 bitów danych na bajt, najwyższy bit = „kolejny bajt
  następuje". Jeden bajt obejmuje wartości < 128 — to przypadek typowy dla indeksów tablicy
  stringów.
* **zig-zag** — liczby ze znakiem są mapowane na bez znaku przed varintem
  (`(n << 1) ^ (n >> 63)`), więc małe wartości ujemne pozostają krótkie.
* **f32 / f64** — IEEE-754, little-endian (4 / 8 bajtów), zgodnie z `sn_utils`.

Wszystkie skalary wielobajtowe są little-endian.

## 5. Strumień wpisów

Każdy wpis zaczyna się od **head varint**. Jego dolne 3 bity to tag (`TAG_MASK = 0x7`); pozostałe
górne bity (`head >> 3`) niosą ładunek wpisów-tokenów (indeks w tablicy stringów). Przypadek
typowy — token — kosztuje więc tylko ten jeden varint.

| wartość tagu | nazwa     | ładunek po head                                                   |
|--------------|-----------|-------------------------------------------------------------------|
| 0            | `TOKEN`   | brak — `head >> 3` to indeks w tablicy stringów. Małymi literami per-konsument przy odtwarzaniu. |
| 1            | `INCLUDE` | `varint(fileexpr_count)` + tyle `varint(string_idx)`, potem `varint(param_count)` + tyle `varint(string_idx)`. |
| 2            | `INT`     | jeden zig-zag varint (wartość całkowita ze znakiem).              |
| 3            | `F32`     | 4 bajty, float little-endian.                                     |
| 4            | `F64`     | 8 bajtów, double little-endian.                                   |
| 5            | `QTOKEN`  | brak — `head >> 3` to indeks w tablicy stringów. **Token cytowany: case zachowany dosłownie** przy odtwarzaniu (nigdy nie zmniejszany). |
| 6            | `NODE`    | marker węzła — patrz §6. Po nim następuje ciało węzła.            |

Logiczne typy wpisów widziane przez konsumenta (`scenery_entry_type`): `token`, `qtoken`,
`number` (dowolny z INT/F32/F64), `include`. Podział INT/F32/F64 to szczegół kodowania ukryty
wewnątrz readera/writera.

### Kodowanie liczb (`write_number`)

Token liczbowy zapisywany jest w najbardziej zwartej formie, która wystarczająco dobrze
round-trip'uje:

1. Wartość **całkowita** w zakresie ±2⁵³ (`±9007199254740992`) → `INT` (zig-zag varint). Pokrywa
   wiele małych liczb w jednym–dwóch bajtach.
2. W przeciwnym razie **`F32`**, jeśli float jest skończony i reprezentuje wartość albo dokładnie,
   albo z błędem względnym ≤ `1e-6` (`|float(v) − v| ≤ 1e-6·|v|`).
3. W przeciwnym razie pełny **`F64`**.

### Wpisy include

Include zapisuje **dosłowne wyrażenie nazwy pliku** (pojedynczy token albo zbiór losowy, np.
`[ a.inc b.inc ]` — z tokenami nawiasów) plus listę parametrów, wszystko jako indeksy
zinternowanych stringów. Przy odtwarzaniu wyrażenie jest re-ewaluowane (z ponownym losowaniem
zbioru losowego), a plik-dziecko wchodzony przezroczyście; własne tokeny dziecka pochodzą z jego
twina (lub tekstu). To właśnie zachowuje `cParser::Name()`, grupowanie węzłów per-`.inc` oraz
substytucję parametrów `(pN)` dokładnie jak ścieżka tekstowa.

## 6. Markery węzłów (progresywne ładowanie)

Każdy **węzeł najwyższego poziomu** (blok `node … end<typ>`; węzły nigdy nie są zagnieżdżone)
jest opakowany w marker `NODE`, żeby reader mógł serwować lub pomijać cały węzeł per-pass.
Układ markera:

```
varint(TAG_NODE)          # head, tag = 6
varint(class)             # NODECLASS_*  (poniżej)
varint(byte_span)         # długość w bajtach buforowanego ciała następującego po markerze
[ f32 X, f32 Y, f32 Z, f32 Range ]   # TYLKO gdy class == NODECLASS_VISUAL_POS (4 floaty)
<body>                    # `byte_span` bajtów: własne wpisy węzła (tokeny/liczby/...)
```

Klasy węzłów (`NODECLASS_*`):

| wartość | nazwa          | znaczenie                                                               |
|---------|----------------|------------------------------------------------------------------------|
| 0       | `INFRA`        | węzeł infrastruktury (tor, trakcja, memcell, event, dźwięk, …).        |
| 1       | `VISUAL`       | węzeł wizualny bez pozycji w markerze (kształty / starsze twiny).      |
| 2       | `VISUAL_POS`   | wizualny węzeł modelu, którego marker niesie pozycję lokalną **oraz zasięg widoczności** — 4 f32: `X, Y, Z, Range` (`Range` = `range_max`, `-1` = nieograniczony). |

> Uwaga: komentarz w kodzie przy `NODECLASS_VISUAL_POS` wciąż mówi „3 f32"; writer realnie
> emituje **4** floaty (X, Y, Z, Range). Na dysku jest układ 4-floatowy powyżej.

`Range`/pozycja pozwalają ładowaniu wizualnemu sterowanemu odległością od kamery („ring")
**przetestować dystans i pominąć węzeł w O(1)** — bez dekodowania żadnego z jego tokenów — oraz
budować eagerly modele dalekie, ale o dużym zasięgu (range poza promieniem strumieniowania),
zamiast porzucać je na dystansie. Kształty (triangles/lines) nie mają pozycji w markerze i
spadają do testu sekcyjnego w deserializerze.

## 7. Passy ładowania (`scenery_load_pass`)

Readerowi mówi się, którą klasę węzłów serwować; `next()` pomija markery węzłów i ciała węzłów
spoza passu (przesuwając kursor o `byte_span` w O(1)), aż dojdzie do wpisu do zaserwowania.
**Wpisy nie-węzłowe (dyrektywy, include'y) są serwowane zawsze**, w każdym passie, by utrzymać
spójny stan transformacji/grup.

| pass             | serwuje                                                            |
|------------------|-------------------------------------------------------------------|
| `all` (0)        | wszystko (ładowanie jednoprzebiegowe, == zachowanie sprzed v6).   |
| `infrastructure` | dyrektywy/include + węzły `INFRA`; węzły wizualne pominięte.       |
| `visual` (2)     | dyrektywy/include + węzły `VISUAL`/`VISUAL_POS`; infra pominięta.  |

Ten sam twin jest otwierany raz, potem przechodzony ponownie: najpierw infrastruktura eagerly
(gra może ruszyć), potem wizuale strumieniowane po sekcjach kamery.

## 8. Powierzchnia API readera (semantyka)

`scenery_binary_reader` (bufor musi przeżyć reader):

* `open(string_view)` — waliduje nagłówek, indeksuje tablicę stringów, ustawia na pierwszym
  wpisie.
* `set_pass(pass)` / `next(out)` — strumieniuje kolejny serwowany wpis (§7).
* `node_position(x, y, z, range)` — pozycja lokalna + `range_max` aktualnie serwowanego węzła,
  jeśli marker je niósł (`VISUAL_POS`); w przeciwnym razie false.
* `skip_to_node_end()` — przeskakuje kursor na koniec aktualnie serwowanego węzła (O(1)), by
  porzucić węzeł po podejrzeniu tylko jego pierwszych wpisów (ring skip kamery).
* `node_offset()` / `seek_node(offset)` — bajtowy offset markera bieżącego węzła oraz
  repozycjonowanie tam, żeby kolejne `next()` zaserwowało ten węzeł (odbudowa jednego węzła na
  żądanie bez re-skanowania twina).
* `infra_relevant()` — flaga z nagłówka (§3); pozwala passowi infrastrukturalnemu w ogóle nie
  otwierać czysto wizualnych liści.
* `progress()` — ułamek skonsumowanej sekcji wpisów (0..100), dla paska ładowania.

Odczyt jest **kontrolowany pod kątem granic**: przy przekroczeniu kursor parkuje na końcu i
zwracane jest zero, więc obcięty twin dekoduje się do pustki zamiast czytać poza bufor.

## 9. Wersjonowanie

`SCENERYBINARY_MAGIC` osadza wersję w 4. bajcie magic (obecnie `10`). Podbicie unieważnia
wszystkie starsze twiny (nie przechodzą `open()` i są rekompilowane) — tak bezpiecznie wdraża
się zmiany formatu. Skrócona historia (szczegóły w nagłówku):

* **v4** — reader strumieniowy z bufora, spakowane tagi wpisów, liczby całkowite jako zig-zag
  varint.
* **v5** — tokeny w oryginalnym case + flaga cytowania, więc baking jest niezależny od gramatyki
  (umożliwia headless/standalone baker).
* **v6** — węzły najwyższego poziomu opakowane w marker (klasa + byte span) → progresywne
  ładowanie.
* **v7** — marker modelu wizualnego niesie też pozycję lokalną modelu.
* **v8+** — poprawka terminatora eventlauncher; marker niesie też zasięg widoczności modelu
  (`range_max`), dając obecny 4-floatowy ładunek `VISUAL_POS`. Aktualna wersja na dysku: **10**.

## 10. Baking

Serializacja i zapis twina to praca samodzielna, zdejmowana ze ścieżki budowy sceny:

* **Asynchronicznie** (`scenerybinary_write_async` / `scenerybinary_wait_all`) — podczas
  normalnego ładowania tekstowego gotowe writery trafiają do małej puli wątków, więc I/O nakłada
  się na resztę budowy sceny; wyniki logowane są na wątku głównym po zakończeniu ładowania.
* **Headless** (`scenerybinary_bake_headless`, tryb linii poleceń `-bake`) — tokenizuje
  scenariusz i, rekurencyjnie, każdy dołączany plik, kompilując każdy twin **bez sceny, renderera
  i okna**. Graf plików odkrywany jest przez tokenizację każdego pliku w izolacji (include'y
  zapisane, ale nieotwierane), a każdy twin kompilowany na wątku roboczym; każdy plik bakowany
  jest dokładnie raz (deduplikacja), więc zapisy twinów nigdy się nie ścigają. Świeże twiny są
  nietknięte; rekompilowane są tylko brakujące/nieaktualne.

## 11. Kto używa maszynerii odtwarzania

Tylko deserializer scenerii (`simulation/simulationstateserializer.cpp`) steruje API
replay/pass/streaming binarnego twina. Każdy inny konsument `cParser` (modele, eventy, pojazdy,
audio, config, …) korzysta ze zwykłego tokenizera tekstu i nie wie nic o formacie binarnym.
