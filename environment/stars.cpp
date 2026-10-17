#include "stdafx.h"
#include "environment/stars.h"
#include "utilities/Globals.h"
#include "model/MdlMngr.h"

//////////////////////////////////////////////////////////////////////////////////////////
// cStars -- simple starfield model, simulating appearance of starry sky

void
cStars::init() {

    // the stars are drawn from the catalogue built into the renderer (environment/starcatalog.h), the model of the starry
    // dome is no longer loaded
    m_stars = nullptr;
}
