#pragma once

#include <cstdint>
#include <string>

#include "maj0sted/domain/fitting/fit_result.hpp"
#include "maj0sted/domain/geometry/horizontal_alignment.hpp"
#include "maj0sted/domain/geometry/vertical_alignment.hpp"
#include "maj0sted/domain/value_objects/cartesian_position.hpp"
#include "maj0sted/domain/value_objects/length.hpp"
#include "maj0sted/domain/value_objects/niweleta_id.hpp"
#include "maj0sted/domain/value_objects/straight_id.hpp"

namespace maj0sted::domain {

/// Niweleta — jednolity odcinek toru złożony z wielu krzywych w planie i w
/// profilu.
///
/// Aggregate Root: to granica spójności całego odcinka. Geometria w planie i w
/// profilu opisują ten sam fizyczny tor, więc ich sumaryczne długości muszą być
/// zgodne — pilnuje tego lengths_consistent(). Krzywe dokłada się wyłącznie
/// przez metody agregatu, który utrzymuje ich kolejność.
class Niweleta {
public:
    explicit Niweleta(NiweletaId id, std::string name = {});

    [[nodiscard]] NiweletaId id() const noexcept { return id_; }
    [[nodiscard]] const std::string& name() const noexcept { return name_; }

    /// Dodaje prostą na końcu planu, nadając jej automatycznie StraightId,
    /// który zwraca (do późniejszych referencji: równoległość, fitowanie).
    StraightId add_straight(CartesianPosition start, CartesianPosition end,
                            JointContinuity joint = JointContinuity::Tangent);

    /// Dokłada krzywą na końcu geometrii w planie, łącząc ją z poprzednim
    /// elementem zadanym stykiem (domyślnie stycznym). Jeśli elementem jest
    /// prosta bez tożsamości, niweleta nadaje jej StraightId.
    void add_plan_element(PlanElement element,
                          JointContinuity joint = JointContinuity::Tangent);

    /// Wpina wynik fitowania w plan: podmienia dwie sąsiednie proste (entry,
    /// exit z @p result) na entry(przyciętą) → curve... → exit(przyciętą),
    /// stykami stycznymi (Tangent). Proste zachowują swoje StraightId.
    /// @throws std::invalid_argument jeśli proste nie zostaną znalezione lub nie
    ///         są sąsiednie (entry bezpośrednio przed exit).
    void apply_fit(const FitResult& result);

    /// Dokłada krzywą na końcu geometrii w profilu.
    void add_profile_element(ProfileElement element);

    [[nodiscard]] const HorizontalAlignment& plan() const noexcept { return plan_; }
    [[nodiscard]] const VerticalAlignment& profile() const noexcept { return profile_; }

    [[nodiscard]] Length plan_length() const { return plan_.total_length(); }
    [[nodiscard]] Length profile_length() const { return profile_.total_length(); }

    /// Czy plan i profil opisują tor tej samej długości (z zadaną tolerancją)?
    [[nodiscard]] bool lengths_consistent(double tolerance_m = 1e-6) const;

private:
    StraightId next_straight_id() noexcept;

    NiweletaId id_;
    std::string name_;
    HorizontalAlignment plan_;
    VerticalAlignment profile_;
    std::uint64_t straight_seq_{0};
};

}  // namespace maj0sted::domain
