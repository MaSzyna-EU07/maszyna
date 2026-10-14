/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "utilities/Classes.h"
#include "audio/sound.h"

// animacja dwustanowa, włącza jeden z dwóch submodeli (jednego z nich może nie być)
class TButton {

public:
// methods
    TButton() = default;
    void Clear(int const i = -1);
    inline
    void FeedbackBitSet( int const i ) {
        iFeedbackBit = 1 << i; };
    void Turn( bool const State );
    inline
    bool GetValue() const {
        return m_state; }
    inline
    bool Active() {
        return pModelOn != nullptr || pModelOff != nullptr; }
    void Update( bool const Power = true );
    bool Init( std::string const &asName, TModel3d const *pModel, bool bNewOn = false );
    void Load( cParser &Parser, TDynamicObject const *Owner );
    void AssignBool(bool const *bValue);
    // sounds of a lamp defined outside the .mmd file (vehicle graph), in the format of soundinc: and sounddec:,
    // played when the lamp lights and goes out. assign them after Init(), to place them at the submodel
    void AssignSounds( std::string const &Increase, std::string const &Decrease, TDynamicObject const *Owner );
    // returns offset of submodel associated with the button from the model centre
    glm::vec3 model_offset() const;
	void gain(float new_volume);
	inline uint8_t b() { return m_state ? 1 : 0; };

private:
// methods
    // imports member data pair from the config file
    bool
        Load_mapping( cParser &Input );
    // plays the sound associated with current state
    void
        play();

// members
    TSubModel
        *pModelOn { nullptr },
        *pModelOff { nullptr }; // submodel dla stanu załączonego i wyłączonego
    bool m_state { false };
    bool const *bData { nullptr };
    int iFeedbackBit { 0 }; // Ra: bit informacji zwrotnej, do wyprowadzenia na pulpit
    sound_source m_soundfxincrease { sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE }; // sound associated with increasing control's value
    sound_source m_soundfxdecrease { sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE }; // sound associated with decreasing control's value
};

//---------------------------------------------------------------------------
