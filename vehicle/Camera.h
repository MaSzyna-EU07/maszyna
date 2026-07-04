/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "input/command.h"
#include "vehicle/DynObj.h"

//---------------------------------------------------------------------------

class TCamera {

  public: // McZapkie: potrzebuje do kiwania na boki
	void Init(glm::dvec3 const &Location, glm::dvec3 const &Angle, TDynamicObject *Owner);
    void Reset();
    void OnCursorMove(double x, double y);
    bool OnCommand( command_data const &Command );
    void Update();
    bool SetMatrix(glm::dmat4 &Matrix);
    void RaLook();

    glm::vec3 Angle; // pitch, yaw, roll
	glm::dvec3 Pos; // współrzędne obserwatora
	glm::dvec3 LookAt; // Idealnie gdyby był to znormalizowany kierunek, ale obecnie są to współrzędne punktu, na który ma patrzeć
	glm::vec3 vUp;
	glm::dvec3 Velocity;

    TDynamicObject *m_owner { nullptr }; // TODO: change to const when shake calculations are part of vehicles update
	glm::vec3 m_owneroffset{};

private:
    glm::dvec3 m_moverate;
    glm::vec3 m_rotationoffsets; // requested changes to pitch, yaw and roll

};

struct viewport_proj_config {
    glm::vec3 pa; // screen lower left corner
    glm::vec3 pb; // screen lower right corner
    glm::vec3 pc; // screen upper left corner
    glm::vec3 pe; // eye position
};
