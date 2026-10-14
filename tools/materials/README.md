# Norma materiałów

Wartości, przy których tekstury wyglądają poprawnie w shaderach opengl33 (`shaders/light_common.glsl`, GGX z roughness = 1 − G).
`validate_materials.py` sprawdza .mat i tekstury według tej normy.

Norma i opis niżej dotyczą shaderów z tej gałęzi (`fix/cab-lighting`: GTAO, parallax mapping, połysk z N·L, reflektory, `param_glossiness` jako wykładnik Phonga, ciemny metal). Na `master` shadery liczą jeszcze po staremu, różnice są opisane przy każdym punkcie. Wartości tekstur z tabel są dobre dla obu wersji, bo stare shadery różnią się tylko połyskiem przy niskim słońcu, reflektorami, `param_glossiness` i ciemnym metalem.

## Jak shader liczy piksel

- albedo = diffuse (sRGB) × kolor diffuse submodelu
- słońce rozproszone = 1.5 · (N·L)^1.25 · albedo · (1 − B)
- połysk słońca = GGX(roughness) · N·L · R · `param_specular`, dla metalu dodatkowo × albedo; bez Fresnela
- otoczenie = cubemapa(roughness) · EnvBRDF(F0, roughness) · alfa normalmapy · `param_reflection`, F0 = 0.04 dla niemetalu, albedo dla metalu; metal odbija co najmniej w połowie, niezależnie od alfy
- reflektory: rozproszone 1.35 · (N·L + 0.1) · albedo, połysk jak od słońca (GGX · N·L · R · `param_specular`)

Na `master` połysk nie miał N·L (przy niskim słońcu nie gasł), a reflektory mnożyły jasność przez (1 + `param_specular`).

## Kanały

| tekstura | kanał | znaczenie |
|---|---|---|
| diffuse | RGB | kolor (sRGB); dla metalu jego kolor odbicia |
| specgloss | R | siła połysku słońca, liniowo, mnożona przez `param_specular` |
| specgloss | G | gładkość, roughness = 1 − G (α = roughness²); `param_glossiness` ≥ 1 się skraca |
| specgloss | B | metal: 0 albo 1; metal nie ma diffuse, cały kolor bierze z odbicia; „metal” ciemniejszy niż liniowo 0.2 shader liczy częściowo, a poniżej 0.02 całkiem, jak lakier z R 0.06 |
| normalmap | RG | normalna (B pomijany, Z liczony) |
| normalmap | A | odbicie otoczenia; 255 tam, gdzie powierzchnia odbija, mniej tylko we wnękach |

## Wartości docelowe (`param_specular: 1`)

| materiał | diffuse sRGB | R | G | B |
|---|---|---|---|---|
| lakier nowy | kolor 40–230 | 15 (0.06) | 150–190 | 0 |
| lakier zużyty, brudny | kolor | 10–15 | 75–130 | 0 |
| plastik matowy (pulpit) | kolor | 13 | 75–115 | 0 |
| plastik błyszczący (gałki) | kolor | 15 | 150–190 | 0 |
| guma, wykładzina | 30–80 | 8–10 | 13–50 | 0 |
| tkanina, tapicerka | kolor | 5–10 | 0–40 | 0 |
| drewno lakierowane / surowe | kolor | 13 / 8 | 130–165 / 25–65 | 0 |
| szkło | – | 15 | 240–255 | 0 |
| stal goła | 170–200 szara | 255 | 130–180 | 255 |
| chrom, nikiel | 190–215 | 255 | 205–240 | 255 |
| aluminium | 225–245 | 255 | 100–180 | 255 |
| mosiądz / miedź | ~(245,228,175) / ~(250,209,194) | 255 | 130–205 | 255 |
| rdza, żeliwo, malowany metal | kolor | 10 | 25–75 | **0** |
| beton, tłuczeń, ziemia | kolor | 8–10 | 0–50 | 0 |

Zasady:

- `param_specular` = 1; siłę połysku niesie R.
- Niemetal: R·`param_specular` 0.02–0.12. R = 255 daje połysk ~17× za mocny; obniżanie G zamiast R tylko rozmywa odblask, a nie go ścisza.
- Metal: B = 255, jasny diffuse (liniowo ≥ 0.2, sRGB ≥ ~125) i alfa normalmapy wysoko. Ciemnego „metalu” shader nie zostawia czarnego, tylko liczy go jak lakier (diffuse wraca, połysk 0.06), a odbicie metalu ma co najmniej 0.5. Na masterze ciemny diffuse albo niska alfa dają czarny metal.
- Bez szumu w R, G, B i alfie normalmapy: w słońcu iskrzy.

## Shadery bez specgloss

| shader | roughness | połysk słońca |
|---|---|---|
| brak `shader:` / `default`, `reflmap` | 0.04 (lustro, tylko otoczenie × `param_reflection`) | brak |
| `normalmap`, `parallax`, `detail_*`, `shadowlessnormalmap`, `sunlessnormalmap`, `default_detail` | (2 / (`param_glossiness` + 2))^¼ jak wykładnik Phonga: 10 (domyślnie) → 0.64, 75 → 0.40, 1000 → 0.21 | `param_specular` · (r+g+b)/2 diffuse |
| `rain_windscreen` | zawsze 0.04 | `param_specular` · (r+g+b)/2 diffuse |

Na `master` `normalmap`, `parallax` i `detail_*` miały roughness 1 − 1/`param_glossiness` (większa wartość = bardziej matowo), a `shadowlessnormalmap`, `sunlessnormalmap` i `default_detail` zawsze 0.04. Walidator rozpoznaje obie wersje z plików shaderów.

## Walidator

Wymaga Pythona 3 z `numpy` i `Pillow` (`pip install numpy pillow`).

```
python3 tools/materials/validate_materials.py <katalog z assetami> [podkatalogi...] [opcje]
python3 tools/materials/validate_materials.py ~/pctga dynamic --fallback <katalog gry> --shaders <repo>/shaders --csv wyniki.csv
```

| opcja | znaczenie |
|---|---|
| podkatalogi | co sprawdzić względem katalogu z assetami, domyślnie `dynamic`; może być węższe, np. `dynamic/pkp/sa103` |
| `--fallback DIR` | drugi katalog gry, w którym szukać tekstur, których nie ma w pierwszym (można podać kilka razy) |
| `--shaders DIR` | shadery, według których liczyć; domyślnie `shaders/` z tego repo. Gdy gra ma inne shadery niż repo, podaj jej katalog `shaders`, żeby ocena zgadzała się z tym, co widać w symulatorze |
| `--csv FILE` | wyniki do pliku CSV: material, shader, poziom, opis |
| `--info` | pokazuje też informacje, nie tylko błędy i uwagi |
| `-j N` | liczba procesów, domyślnie wszystkie rdzenie; cały pctga trwa ok. 80 s |

Wynik: dla każdego materiału z problemem ścieżka `.mat`, shader i lista komunikatów, na końcu podsumowanie.

- `BLAD` – trzeba poprawić (np. połysk niemetalu kilkanaście razy za mocny, brak tekstury, nieistniejący shader).
- `UWAGA` – prawie na pewno źle wygląda (lustro na niemetalu, ciemny metal, szum w specgloss).
- `info` – tylko z `--info`, rzeczy do wiadomości.

Walidator czyta pliki shaderów i z nich rozpoznaje, co każdy shader robi ze specgloss, `param_glossiness`, `param_specular` i `param_reflection`, więc działa i dla starych, i dla nowych shaderów.

### Jak poprawiać

1. Uruchom walidator na swoim katalogu pojazdu, np. `dynamic/pkp/sa103`.
2. Najpierw `BLAD`: zwykle R w specgloss (niemetal ok. 15/255) albo `param_specular` (zostaw 1, siłę połysku daje R).
3. Potem `UWAGA`: G (gładkość) według tabeli wartości docelowych, B tylko 0 albo 255, metal z jasnym diffuse.
4. Poprawiaj źródłowe TGA i uruchom walidator jeszcze raz. Z TGA będą robione tekstury BC7, które zastąpią obecne DDS, więc poprawki w DDS przepadną.
