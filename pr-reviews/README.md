# Recenzje otwartych PR-ów — MaSzyna-EU07/maszyna (stan na 7.11.2026)

23 PR-y (bez draftów #216, #221, #203, #173, #137). Każda recenzja w osobnym pliku `PR-<numer>.md`, gotowa do wklejenia na GitHuba.
Punktem odniesienia były zasady zespołu z `CLAUDE.md` (GCC, Sonar, kompatybilność danych, renderer GL4.6).

| PR | Tytuł (skrót) | Autor | Werdykt |
|---|---|---|---|
| #265 | AI pod górę: trzymanie i ruszanie na hamulcu | pwarszawski | Request changes |
| #264 | AI: mniejsze, wcześniejsze stopnie hamowania | pwarszawski | Approve z uwagami |
| #263 | ESt3: nacisk sprężyn, stopień min., odluźniacz, AL2 | pwarszawski | Approve; golden/CI do sprawdzenia |
| #262 | AI: odluźniacz tylko przy ruszaniu | pwarszawski | Approve |
| #261 | 401Da: lampka pompy oleju, dźwignia prędkościomierza | jerrrrycho | Request changes (wydzielić AVX2) |
| #260 | SU45/SU46: lampka styczników liniowych | jerrrrycho | Approve |
| #259 | SM03: gaszenie silnika wyłącznikiem | jerrrrycho | Approve z drobnymi |
| #243 | Edytor NMT: przypisz z kamery, `//$g` w include | miloszunio | Request changes |
| #241 | Edytor: lista wypełnionych obszarów | miloszunio | Request changes |
| #240 | GL46: głębia kursora edytora | miloszunio | Approve po teście |
| #239 | Refaktor długich funkcji Mover | jerrrrycho | Request changes (rebase) |
| #234 | Przełączniki świateł 3-pozycyjne | dos1 | Approve z uwagami |
| #231 | Przełączanie AI z menu | dos1 | Request changes (drobne) |
| #230 | Rozmiar Słońca i Księżyca | dos1 | Approve (+ odpowiednik GL46) |
| #228 | XYZ→RGB w sRGB | dos1 | Approve z drobnymi |
| #227 | Wartości turbidity | dos1 | Approve po opisie |
| #224 | float3d → glm | docentYT | Request changes (build VR) |
| #187 | AI: spadki, dojazdy, 8–15 m przed semaforem | pwarszawski | Approve kierunku; Sonar/GCC |
| #172 | UART Hardware Protocol v2 | Hirek193 | Request changes |
| #167 | whois +48 | pwarszawski | Approve z drobnymi |
| #165 | HUD | DJ-YASHITAKA | Request changes → draft |
| #159 | Spawn składów z Lua | jakubg1 | Request changes |
| #150 | Podział OnCommands z Train | jerrrrycho | Approve kierunku; rebase |

## Najważniejsze znaleziska
- **#165** — przypadkowe zmiany w `Globals.h`: `targetfps` 60 → 0 i usunięty klucz UART `dynamicbrake` (psuje odczyt `eu07.ini`).
- **#224** — usuwa `ReplaceMatrix()`, którego używa `vr/openvr_imp.cpp` → build z OpenVR się nie skompiluje (CI tego nie łapie).
- **#265** — flaga `bUphillStart` może zostać `true` po przejęciu sterowania przez gracza; pełna moc przeciw zahamowanemu składowi przez 10–18 s to ryzyko dla sprzęgów.
- **#150** — po 10.10 doszedł `world/RoadPoint.cpp` ze starym `#include "vehicle/Train.h"` → po merge build się wysypie bez rebase.
- **#263** — zmienia fizykę każdego ESt3, a nie rusza testów golden; GitHub pokazuje `unstable`.
- **#159** — `insert_trainset_full()` nie zatrzymuje się na `endtrainset`; `lua_tostring` może zwrócić `NULL`.

## Proponowana kolejność merge (konflikty)
1. Małe, niezależne: #262, #260, #259, #167, #228, #227, #230, #234, #231 (po poprawce).
2. Stos AI/hamulce, po kolei z rebase: #187 → #263 → #264 → #265 (najlepiej w jednym oknie).
3. Poprawka AVX2 z #261 jako osobny PR, potem #261.
4. Refaktory na końcu, z ponownym puszczeniem skryptu / rebase: #150 (Train.cpp), #239 (Mover.cpp), #224 (float3 → glm).
5. Gałęzie poboczne: #240, #241 → `feat/gl46-migration`; #243 → `feat/switch-blades`.
6. Do rozmowy z autorami / draft: #165, #172, #159.
