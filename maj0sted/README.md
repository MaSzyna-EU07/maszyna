# maj0sted

Statyczna biblioteka **C++20** do obsługi **edytora torowiska**, zbudowana
zgodnie z **Domain-Driven Design (DDD)**.

Biblioteka dostarcza czysty model domenowy — bez zależności od UI, bazy danych
czy frameworka. Warstwy aplikacji i infrastruktury podpina się do zdefiniowanych
tu typów.

## Model domenowy

```
maj0sted::domain
├── value_objects/         Value Objects (niezmienne, porównywane przez wartość)
│   ├── Crs                układ odniesienia (kod EPSG), domyślnie EPSG:2180
│   └── CartesianPosition  pozycja kartezjańska (x, y w jednostkach CRS)
└── MapProject             projekt mapy — definiuje CRS dla wszystkich pozycji
```

Projekt mapy (`MapProject`) definiuje kartezjański układ odniesienia, w którym
wyrażane są wszystkie pozycje. Domyślnie jest to **EPSG:2180** (PUWG 1992 /
Poland CS92), którego jednostką jest metr.

## Szybki start

```cpp
#include "maj0sted/maj0sted.hpp"
using namespace maj0sted::domain;

MapProject project;                 // domyślnie EPSG:2180
int epsg = project.crs().epsg();    // 2180

CartesianPosition p{500000.0, 300000.0};  // x, y w metrach
```

Pełniejszy przykład: [`examples/basic_editor.cpp`](examples/basic_editor.cpp).

## Budowanie

Wymagania: kompilator C++20 (GCC 11+/Clang 14+/MSVC 19.3+) oraz CMake ≥ 3.20.

```bash
cmake -S . -B build            # dodaj -G Ninja, jeśli masz ninja
cmake --build build
ctest --test-dir build --output-on-failure
./build/examples/maj0sted_example
```

### Opcje CMake

| Opcja                        | Domyślnie | Znaczenie                     |
|------------------------------|-----------|-------------------------------|
| `MAJ0STED_BUILD_TESTS`       | `ON`      | buduj testy jednostkowe       |
| `MAJ0STED_BUILD_EXAMPLES`    | `ON`      | buduj przykłady               |
| `MAJ0STED_WARNINGS_AS_ERRORS`| `OFF`     | traktuj ostrzeżenia jak błędy |

### Użycie w innym projekcie

Po `cmake --install build` (albo przez `add_subdirectory`):

```cmake
find_package(maj0sted REQUIRED)
target_link_libraries(twoj_target PRIVATE maj0sted::maj0sted)
```

## Struktura projektu

```
maj0sted/
├── CMakeLists.txt
├── include/maj0sted/     nagłówki publiczne (API)
├── src/domain/           implementacje (.cpp)
├── tests/                testy jednostkowe (CTest, bez zależności)
├── examples/             przykłady użycia
└── cmake/                pliki pakietu find_package
```

## Testy

Testy używają własnego, malutkiego frameworka (`tests/check.hpp`) — nie wymagają
żadnych zewnętrznych bibliotek. Każdy plik testowy to osobny plik wykonywalny
zwracający kod wyjścia dla CTest.

## Licencja

Do uzupełnienia (np. MIT) — dodaj plik `LICENSE`.
