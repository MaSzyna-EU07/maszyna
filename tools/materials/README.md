# Norma materiałów

Wartości, przy których tekstury wyglądają poprawnie w obecnych shaderach (`shaders/light_common.glsl`, GGX z roughness = 1 − G).
`validate_materials.py` sprawdza .mat i tekstury według tej normy.

## Jak shader liczy piksel

- albedo = diffuse (sRGB) × kolor diffuse submodelu
- słońce rozproszone = 1.5 · (N·L)^1.25 · albedo · (1 − B)
- połysk słońca = GGX(roughness) · R · `param_specular`, dla metalu dodatkowo × albedo; bez Fresnela i bez N·L
- otoczenie = cubemapa(roughness) · EnvBRDF(F0, roughness) · alfa normalmapy · `param_reflection`, F0 = 0.04 dla niemetalu, albedo dla metalu
- reflektory = (N·L + 0.1) · (1 + `param_specular`) · albedo

## Kanały

| tekstura | kanał | znaczenie |
|---|---|---|
| diffuse | RGB | kolor (sRGB); dla metalu jego kolor odbicia |
| specgloss | R | siła połysku słońca, liniowo, mnożona przez `param_specular` |
| specgloss | G | gładkość, roughness = 1 − G (α = roughness²); `param_glossiness` ≥ 1 się skraca |
| specgloss | B | metal: 0 albo 1; metal nie ma diffuse, cały kolor bierze z odbicia |
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

- `param_specular` = 1. Większy rozjaśnia też obiekt w reflektorach (×(1 + `param_specular`)).
- Niemetal: R·`param_specular` 0.02–0.12. R = 255 daje połysk ~17× za mocny; obniżanie G zamiast R tylko rozmywa odblask, a nie go ścisza.
- Metal: B = 255, jasny diffuse (liniowo ≥ 0.2, sRGB ≥ ~125) i alfa normalmapy wysoko. Ciemny diffuse albo niska alfa = czarny metal.
- Bez szumu w R, G, B i alfie normalmapy: w słońcu iskrzy.

## Shadery bez specgloss

| shader | roughness | połysk słońca |
|---|---|---|
| brak `shader:` / `default`, `reflmap` | 0.04 (lustro, tylko otoczenie × `param_reflection`) | brak |
| `normalmap`, `parallax`, `detail_*` | 1 − 1/`param_glossiness` (większa wartość = bardziej matowo) | `param_specular` · (r+g+b)/2 diffuse |
| `shadowlessnormalmap`, `sunlessnormalmap`, `default_detail`, `rain_windscreen` | zawsze 0.04 | `param_specular` · (r+g+b)/2 diffuse |

## Walidator

```
python3 tools/materials/validate_materials.py ~/Projects/pctga dynamic --fallback <katalog gry> --csv wyniki.csv
```

`BLAD` – trzeba poprawić, `UWAGA` – prawie na pewno źle wygląda, `--info` pokazuje też informacje.
