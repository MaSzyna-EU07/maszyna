# Pogoda w fizyce pojazdów (`physics.weather`)

Pogoda symulacji (`environment/weather.h`, `environment/snowcover.h`) wpływa na fizykę ruchu pojazdów, a shadery pokazują ten
sam stan, z którego liczy fizyka. Wejście wszędzie jest jedno: natężenie opadu `saturate(Overcast - 1)` i rodzaj opadu
`world_weather::falling()` (to samo, z czego rysowany jest opad i woda na szybach), wiatr przy ziemi z porywami
`world_environment::wind()` (ten sam, z którym dryfuje deszcz na szybie), temperatura `Global.AirTemperature`, wilgotność
i mgła pogody.

## Co jest liczone

| Zjawisko | Gdzie | Co robi |
|---|---|---|
| Stan główki szyny | `environment/railsurface.{h,cpp}`, `world_weather::rails()` | woda (deszcz, mokry śnieg, rosa nocą), brud suchych dni (zmywany ulewą, mżawka robi z niego maź), lód (marznący deszcz, zamarzająca woda, szadź), liście jesienią. Na starcie scenerii stan „dochodzi” do pogody startu |
| Przyczepność | `weather_physics::adhesion_factor()`, `TDynamicObject::rails_on_track()` | tarcie toru × udział suchej szyny; oś prowadząca (gorsza z dwóch) decyduje; tunel ma szyny suche; śnieg na główce z `TTrack::snow_at()` |
| Czyszczenie przez koła | `TTrack::wheels_passed()`, `TTrack::since_wheels()` | koła zdejmują część liści, lodu i brudu: pierwszy pociąg ma najgorzej, wagony za lokomotywą jadą po oczyszczonej szynie |
| Piasek | `TMoverParameters::Adhesive()`, `weather_physics::sanded()` | na suchej szynie jak dotąd (×1.25), na śliskiej przywraca co najmniej `physics.weather.sand` suchej |
| AI i drogi | `Global.FrictionWeatherFactor` | liczone z tego samego stanu szyny (bez czyszczenia kołami), 0.5..1 |
| Hamulce | `weather_physics::update_brakes()`, `TBrake::weather_factor()` | deszcz moczy okładziny, mróz zamraża wodę, śnieg wokół podwozia je oblodza; ciepło hamowania najpierw suszy, potem rozmraża. Tarcie zależy od materiału (żeliwo P10, kompozyt, tarcza). Dotyczy też przewidywania hamowania AI (`GetFC`) |
| Wiatr i powietrze | `weather_physics::aerodynamic_force()`, `TMoverParameters::WeatherDragForce()` | opór względem powietrza, nie toru: wiatr czołowy hamuje, w plecy pcha (też stojący wagon), boczny zwiększa opór (`windyaw`); gęstość powietrza z temperatury. Bez wiatru w 15 °C siła jest dokładnie ta sama co dotąd |
| Wiatr na drutach | `weather_physics::wire_sway()`, `TTraction::sway()` / `sway_at()` | przęsło odchyla się z wiatrem (obciążenie wiatrem drutu, z lodem grubszego, naciąg `wiretension`): kształt `4u(1-u)`, przy słupach zero. Pantograf szuka drutu tam, gdzie wiatr go zaniósł (`vehicle_table::update_traction`, `basic_cell::update_traction`): drut poza nakładką = brak styku, a gdy wraca pod podniesiony ślizg, pantograf się łamie (istniejąca logika `dtrain_pantograph`). Przęsło 65 m: ok. 0.19 m przy 20 m/s, 0.42 m przy 30 m/s |
| Osad na drutach | `snow_cover::ice()`, `snow_cover::wire_ice(kind)` | cztery rodzaje, każdy osobno: **gołoledź** (oblodzenie, marznący deszcz, 900 kg/m3, szklista), **szadź twarda** (przechłodzona mgła na wietrze od 2–4 m/s, -1..-12 °C, ok. 450 kg/m3), **szadź miękka i szron** (sadź; mgła w ciszy albo w silnym mrozie, nasycone powietrze, ok. 120 kg/m3, wiatr od 8 m/s ją strąca), **mokry śnieg** (rękaw na drucie, zsuwa się w odwilż i w deszczu). „Szadź” i „sadź” to ta sama nazwa zjawiska (sadź: termin meteorologiczny); na drutach liczy się podział na twardą i miękką |
| Łuki elektryczne | `TDynamicObject::wire_ice_break()`, `weather_physics::arc_strength()` | przerwa styku pod prądem zapala łuk o sile zależnej od prądu pantografu (od 20 A, pełny od ok. 600 A); bez prądu łuku nie ma. Światło (niebiesko-białe, migoczące, oświetla otoczenie, `gfx::local_lights`), błysk przy ślizgu (`scene_walk::Render_arcs`, z testem głębokości: zasłania go dach, wiadukt) i trzask (`pantographarc:` w .mmd, a bez niego dźwięk robiony przez symulator, `sounds/pantograph_arc` w danych go zastępuje) |
| Lód na drutach | `TDynamicObject::wire_ice_break()`, `weather_physics::break_rate()` | przerwy styku pantografu z drutem: częstsze z grubością lodu i prędkością, rzadsze dla pantografu jadącego za innym (pierwszy zdziera lód); długie przerwy wyłączają WS, krótkie przetrzymuje istniejące 0.2 s |

## Shadery

`gl::weather_ubs::surface_state` (oba `weather_common.glsl`): x woda, y lód, z liście, w lód na drutach w mm.

- gl46, `light_common.glsl: apply_wet()`: nawierzchnie pod gołym niebem są ciemniejsze i błyszczące, gdy mokre, i szkliste, gdy
  oblodzone. Pod dachem, mostem i w tunelu suche (`sky_occlusion_cover()`, ta sama funkcja, co opad), pod śniegiem nie.
- `traction.frag` (gl46 i vk14): kolor drutu od rodzaju osadu (`model_ubs.param[2]`, mm każdego rodzaju): gołoledź szklista i szara,
  szadź twarda biała, szadź miękka bielsza, mokry śnieg szarobiały. Z bliska bryły drutów (i ich cienie) grubieją o grubość
  lodu (`param[1].w`, wzdłuż normalnych w `wire_sway.glsl`).
- `traction.vert` (gl46 i vk14): druty i ich bryły z bliska odchylają się o to samo `TTraction::sway()`, z którym liczy fizyka
  (`model_ubs.param[1]`: oś w poprzek przęsła i wychylenie środka; `v_coord.x` wierzchołka: miejsce w przęśle). Linie przęsła są
  dzielone na odcinki nie dłuższe niż 1/8 przęsła (`TTraction::sway_vertices()`). Wzór jest w jednym miejscu, `wire_sway.glsl`.
  Cienie drutów kołyszą się razem z nimi (`physics.weather.wireshadows`): własne programy cieni, `wireshadow.vert` dla kaskad
  słońca i `wireshadowcube.vert` dla map lamp (`walk_shader::line_shadow`). Stary renderer (gl33) nie rysuje odchylenia.
- vk14 nie rysuje (tak jak śniegu) mokrej nawierzchni, ma tylko pole w bloku i lód na drutach.

## Ustawienia (eu07.ini)

Wszystkie wpisy, okno i eksport ustawień biorą się z jednej tabeli `weather_physics::bool_parameters` / `float_parameters`
(`McZapkie/weatherphysics.h`). Okno: menu *Mode windows → Weather physics* („Fizyka pogody”), w nim też stan szyn
do ustawienia ręcznie, przyspieszenie czasu i podgląd prowadzonego pojazdu.

| Klucz | Domyślnie | Znaczenie |
|---|---|---|
| `physics.weather` | yes | całość; `no` = stare 0.85 deszcz / 0.75 śnieg |
| `physics.weather.adhesion` / `.brakes` / `.wind` / `.airdensity` / `.pantographs` | yes | poszczególne części |
| `physics.weather.wirewind` | yes | wiatr odchyla druty (rysowanie i fizyka razem) i zrywa pantografy |
| `physics.weather.wireshadows` | yes | cienie drutów odchylane razem z drutami; `no` = cienie jak dawniej, nieruchome |
| `physics.weather.arcs` | yes | łuki przy przerwach styku (światło, błysk, dźwięk) |
| `physics.weather.arcbrightness` | 1 | jasność łuków |
| `physics.weather.glaze` / `.rimehard` / `.rimesoft` / `.wetsnow` | yes | które rodzaje osadu pogoda kładzie na drutach; wyłączony rodzaj znika |
| `physics.weather.breakglaze` / `.breakrimehard` / `.breakrimesoft` / `.breakwetsnow` | 1 / 0.6 / 0.25 / 0.5 | jak bardzo każdy rodzaj przerywa styk (udział gołoledzi) |
| `physics.weather.wiretension` / `.wirediameter` | 10000 N / 0.012 m | naciąg i średnica przewodu jezdnego |
| `physics.weather.wet` | 0.75 | przyczepność mokrej szyny (udział suchej) |
| `physics.weather.emulsion` | 0.6 | dodatkowo: pierwsza woda na brudzie (razem ok. 0.45) |
| `physics.weather.ice` | 0.3 | lód na główce |
| `physics.weather.snow` | 0.5 | śnieg na główce (od 2 cm) |
| `physics.weather.leavesdry` / `.leaveswet` | 0.5 / 0.25 | zgniecione liście, suche / mokre |
| `physics.weather.minimum` | 0.12 | najmniej łącznie |
| `physics.weather.leaves` | 1 | ile liści spada jesienią (0..2) |
| `physics.weather.passrecovery` | 600 | s, po jakim czasie szyna oczyszczona kołami wraca do stanu z pogody |
| `physics.weather.sand` | 0.8 | piasek przywraca co najmniej tyle suchej szyny |
| `physics.weather.brakewet{castiron,composite,disc}` | 0.7 / 0.85 / 0.92 | tarcie mokrych hamulców |
| `physics.weather.brakeice{castiron,composite,disc}` | 0.6 / 0.45 / 0.5 | tarcie oblodzonych hamulców |
| `physics.weather.windyaw` | 1.5 | wzrost oporu z kątem wiatru |
| `physics.weather.wireicebreaks` | 1 | skala częstości przerw styku |
| `physics.weather.breaktime` | 0.08 | s, przerwa przy 1 mm lodu (rośnie z pierwiastkiem grubości) |
| `physics.weather.trailingshare` | 0.3 | część przerw dla pantografu za innym |

## Czego nie ma (świadomie)

- Mróz a opory łożysk: nie realizujemy.
- Wiatr w tunelu: druty kołyszą się tak samo jak na zewnątrz (przęsło nie wie, że jest w tunelu; rysowanie i fizyka są
  przynajmniej zgodne).
- Bocznikowanie obwodów torowych (rezystancja koło–szyna, liście, rdza): to sterowanie ruchem, osobny temat.
- Iskry (cząstki) przy łuku: jest światło, błysk i dźwięk, bez rozprysku iskier (system cząstek ma tylko dym).

Testy: `tests/unit/test_weatherphysics.cpp`.
