#include "maj0sted/editor/model.hpp"

namespace maj0sted::editor {
namespace {

template <typename Container, typename Id>
auto* lookup(Container& items, Id id) {
    if (id == Id::none) {
        return decltype(&items[0]){nullptr};
    }
    for (auto& item : items) {
        if (item.id == id) {
            return &item;
        }
    }
    return decltype(&items[0]){nullptr};
}

}  // namespace

ElementId mint_element(Document& document) { return static_cast<ElementId>(document.next_id++); }
TrackId mint_track(Document& document) { return static_cast<TrackId>(document.next_id++); }
TurnoutId mint_turnout(Document& document) { return static_cast<TurnoutId>(document.next_id++); }

const Track* find_track(const Document& document, TrackId id) {
    return lookup(document.tracks, id);
}
Track* find_track(Document& document, TrackId id) { return lookup(document.tracks, id); }

const TurnoutPlacement* find_turnout(const Document& document, TurnoutId id) {
    return lookup(document.turnouts, id);
}
TurnoutPlacement* find_turnout(Document& document, TurnoutId id) {
    return lookup(document.turnouts, id);
}

const TurnoutType* find_type(const Document& document, const std::string& name) {
    for (const auto& type : document.turnout_types) {
        if (type.name == name) {
            return &type;
        }
    }
    return nullptr;
}

const Element* find_element(const Document& document, ElementId id) {
    if (id == ElementId::none) {
        return nullptr;
    }
    for (const auto& track : document.tracks) {
        for (const auto& element : track.elements) {
            if (element.id == id) {
                return &element;
            }
        }
    }
    return nullptr;
}

const Track* find_element_track(const Document& document, ElementId id) {
    if (id == ElementId::none) {
        return nullptr;
    }
    for (const auto& track : document.tracks) {
        for (const auto& element : track.elements) {
            if (element.id == id) {
                return &track;
            }
        }
    }
    return nullptr;
}

}  // namespace maj0sted::editor
