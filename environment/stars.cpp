#include "stdafx.h"
#include "environment/stars.h"
#include "utilities/Globals.h"
#include "model/MdlMngr.h"

//////////////////////////////////////////////////////////////////////////////////////////
// cStars -- simple starfield model, simulating appearance of starry sky

void
cStars::init() {

    m_stars = TModelsManager::GetModel( "skydome_stars.t3d", false );
}
