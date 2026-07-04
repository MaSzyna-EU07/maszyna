/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak and others

*/

#include "stdafx.h"
#include "model/AnimModel.h"

#include "rendering/renderer.h"
#include "model/MdlMngr.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "world/Event.h"
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "utilities/Logs.h"
#include "rendering/renderer.h"

std::list<std::weak_ptr<TAnimContainer>> TAnimModel::acAnimList;

TAnimContainer::TAnimContainer()
{
	vRotateAngles = glm::vec3(0.0f, 0.0f, 0.0f); // aktualne kąty obrotu
	vDesiredAngles = glm::vec3(0.0f, 0.0f, 0.0f); // docelowe kąty obrotu
    fRotateSpeed = 0.0;
	vTranslation = glm::dvec3(0.0, 0.0, 0.0); // aktualne przesunięcie
    vTranslateTo = glm::dvec3(0.0, 0.0, 0.0); // docelowe przesunięcie
    fTranslateSpeed = 0.0;
    fAngleSpeed = 0.0;
    pSubModel = nullptr;
	iAnim = 0; // położenie początkowe
	evDone = nullptr; // powiadamianie o zakończeniu animacji
}

bool TAnimContainer::Init(TSubModel *pNewSubModel)
{
    fRotateSpeed = 0.0f;
    pSubModel = pNewSubModel;
    return pSubModel != nullptr;
}

void TAnimContainer::SetRotateAnim(glm::vec3 vNewRotateAngles, double fNewRotateSpeed)
{
    vDesiredAngles = vNewRotateAngles;
    fRotateSpeed = fNewRotateSpeed;
    iAnim |= 1;
    /* //Ra 2014-07: jeśli model nie jest renderowany, to obliczyć czas animacji i dodać event
     wewnętrzny
     //można by też ustawić czas początku animacji zamiast pobierać czas ramki i liczyć różnicę
    */
    if (evDone)
    { // dołączyć model do listy aniomowania, żeby animacje były przeliczane również bez
        // wyświetlania
        if (iAnim >= 0)
        { // jeśli nie jest jeszcze na liście animacyjnej
			TAnimModel::acAnimList.push_back(shared_from_this());
			iAnim |= 0x80000000; // dodany do listy
        }
    }
}

void TAnimContainer::SetTranslateAnim(glm::dvec3 vNewTranslate, double fNewSpeed)
{
    vTranslateTo = vNewTranslate;
    fTranslateSpeed = fNewSpeed;
    iAnim |= 2;
    /* //Ra 2014-07: jeśli model nie jest renderowany, to obliczyć czas animacji i dodać event
     wewnętrzny
     //można by też ustawić czas początku animacji zamiast pobierać czas ramki i liczyć różnicę
    */
    if (evDone)
    { // dołączyć model do listy aniomowania, żeby animacje były przeliczane również bez
        // wyświetlania
        if (iAnim >= 0)
        { // jeśli nie jest jeszcze na liście animacyjnej
			TAnimModel::acAnimList.push_back(shared_from_this());
            iAnim |= 0x80000000; // dodany do listy
        }
    }
}

// przeliczanie animacji wykonać tylko raz na model
void TAnimContainer::UpdateModel() {

    if (pSubModel) // pozbyć się tego - sprawdzać wcześniej
    {
        if (fTranslateSpeed != 0.0)
        {
            auto dif = vTranslateTo - vTranslation; // wektor w kierunku docelowym
			double l2 = glm::length2(dif); // długość wektora potrzebnego przemieszczenia
            if (l2 >= sq(0.01))
            { // jeśli do przemieszczenia jest ponad 1cm
				auto s = glm::normalize(dif); // jednostkowy wektor kierunku // Długość wektora nie jest równa 0, sprawdzane wcześniej więc wektor normalny będzie zawsze prawidłowy.
                s = s *
                    (fTranslateSpeed *
                     Timer::GetDeltaTime()); // przemieszczenie w podanym czasie z daną prędkością
                if (glm::length2(s) < l2) //żeby nie jechało na drugą stronę
                    vTranslation += s;
                else
                    vTranslation = vTranslateTo; // koniec animacji, "koniec animowania" uruchomi
                // się w następnej klatce
            }
            else
            { // koniec animowania
                vTranslation = vTranslateTo;
                fTranslateSpeed = 0.0; // wyłączenie przeliczania wektora
				if (glm::length2(vTranslation) <= sq(0.01)) // jeśli jest w punkcie początkowym
                    iAnim &= ~2; // wyłączyć zmianę pozycji submodelu
                if( evDone ) {
                    // wykonanie eventu informującego o zakończeniu
                    simulation::Events.AddToQuery( evDone, nullptr );
                }
            }
        }
        if (fRotateSpeed != 0.0)
        {
            bool anim = false;
            auto dif = vDesiredAngles - vRotateAngles;
            double s;
            s = std::abs( fRotateSpeed ) * sign(dif.x) * Timer::GetDeltaTime();
            if (fabs(s) >= fabs(dif.x))
                vRotateAngles.x = vDesiredAngles.x;
            else
            {
                vRotateAngles.x += s;
                anim = true;
            }
            s = std::abs( fRotateSpeed ) * sign(dif.y) * Timer::GetDeltaTime();
            if (fabs(s) >= fabs(dif.y))
                vRotateAngles.y = vDesiredAngles.y;
            else
            {
                vRotateAngles.y += s;
                anim = true;
            }
            s = std::abs( fRotateSpeed ) * sign(dif.z) * Timer::GetDeltaTime();
            if (fabs(s) >= fabs(dif.z))
                vRotateAngles.z = vDesiredAngles.z;
            else
            {
                vRotateAngles.z += s;
                anim = true;
            }
            // HACK: negative speed allows to work around legacy behaviour, where desired angle > 360 meant permanent rotation
            if( fRotateSpeed > 0.0 ) {
                while( vRotateAngles.x >= 360 )
                    vRotateAngles.x -= 360;
                while( vRotateAngles.x <= -360 )
                    vRotateAngles.x += 360;
                while( vRotateAngles.y >= 360 )
                    vRotateAngles.y -= 360;
                while( vRotateAngles.y <= -360 )
                    vRotateAngles.y += 360;
                while( vRotateAngles.z >= 360 )
                    vRotateAngles.z -= 360;
                while( vRotateAngles.z <= -360 )
                    vRotateAngles.z += 360;
            }

            if( vRotateAngles.x == 0.0
             && vRotateAngles.y == 0.0
             && vRotateAngles.z == 0.0 ) {
                iAnim &= ~1; // kąty są zerowe
            }
            if (!anim)
            { // nie potrzeba przeliczać już
                fRotateSpeed = 0.0;
                if( evDone ) {
                    // wykonanie eventu informującego o zakończeniu
                    simulation::Events.AddToQuery( evDone, nullptr );
                }
            }
        }
        if( fAngleSpeed != 0.f ) {
            // NOTE: this is angle- not quaternion-based rotation TBD, TODO: switch to quaternion rotations?
            fAngleCurrent += fAngleSpeed * Timer::GetDeltaTime(); // aktualny parametr interpolacji
        }
    }
};

void TAnimContainer::PrepareModel()
{ // tutaj zostawić tylko ustawienie submodelu, przeliczanie ma być w UpdateModel()
    if (pSubModel) // pozbyć się tego - sprawdzać wcześniej
    {
        // nanoszenie animacji na wzorzec
        if (iAnim & 1) // zmieniona pozycja względem początkowej
            pSubModel->SetRotateXYZ(vRotateAngles); // ustawia typ animacji
        if (iAnim & 2) // zmieniona pozycja względem początkowej
            pSubModel->SetTranslate(vTranslation);
        if (iAnim & 4) // zmieniona pozycja względem początkowej
        {
            if (fAngleSpeed > 0.0f)
            {
                if (fAngleCurrent >= 1.0f)
                { // interpolacja zakończona, ustawienie na pozycję końcową
                    qCurrent = qDesired;
                    fAngleSpeed = 0.0; // wyłączenie przeliczania wektora
                    if( evDone ) {
                        // wykonanie eventu informującego o zakończeniu
                        simulation::Events.AddToQuery( evDone, nullptr );
                    }
                }
                else
                { // obliczanie pozycji pośredniej
                    // normalizacja jest wymagana do interpolacji w następnej animacji
                    qCurrent = Normalize(
                        Slerp(qStart, qDesired, fAngleCurrent)); // interpolacja sferyczna kąta
                    // qCurrent=Slerp(qStart,qDesired,fAngleCurrent); //interpolacja sferyczna kąta
                    if (qCurrent.w ==
                        1.0) // rozpoznać brak obrotu i wyłączyć w iAnim w takim przypadku
                        iAnim &= ~4; // kąty są zerowe
                }
            }
            mAnim->Quaternion(&qCurrent); // wypełnienie macierzy (wymaga normalizacji?)
            pSubModel->mAnimMatrix = mAnim; // użyczenie do submodelu (na czas renderowania!)
        }
    }
    // if (!strcmp(pSubModel->pName,"?Z?“?^?[")) //jak główna kość
    // WriteLog(AnsiString(pMovementData->iFrame)+": "+AnsiString(iAnim)+"
    // "+AnsiString(vTranslation.x)+" "+AnsiString(vTranslation.y)+" "+AnsiString(vTranslation.z));
}

bool TAnimContainer::InMovement()
{ // czy trwa animacja - informacja dla obrotnicy
    return fRotateSpeed != 0.0 || fTranslateSpeed != 0.0;
}

void TAnimContainer::EventAssign(basic_event *ev)
{ // przypisanie eventu wykonywanego po zakończeniu animacji
    evDone = ev;
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

TAnimModel::TAnimModel( scene::node_data const &Nodedata ) : basic_node( Nodedata ) {

    m_lightcolors.fill( glm::vec3{ -1.f } );
    m_lightopacities.fill( 1.f );
}

bool TAnimModel::Init(std::string const &asName, std::string const &asReplacableTexture)
{
    if( asReplacableTexture.substr( 0, 1 ) == "*" ) {
        // od gwiazdki zaczynają się teksty na wyświetlaczach
        asText = asReplacableTexture.substr( 1, asReplacableTexture.length() - 1 ); // zapamiętanie tekstu
    }
    else if( asReplacableTexture != "none" ) {
        m_materialdata.assign( asReplacableTexture );
    }

// TODO: redo the random timer initialization
//    fBlinkTimer = Random() * ( fOnTime + fOffTime );

    pModel = TModelsManager::GetModel( asName );
    return pModel != nullptr;
}

bool
TAnimModel::is_keyword( std::string const &Token )
{

    return Token == "endmodel"
        || Token == "lights"
        || Token == "lightcolors"
        || Token == "angles"
        || Token == "scale"
        || Token == "notransition";
}

bool TAnimModel::Load(cParser *parser, bool ter)
{ // rozpoznanie wpisu modelu i ustawienie świateł
	auto name = parser->getToken<std::string>();
	auto texture = parser->getToken<std::string>(false);
    replace_slashes( name );
    replace_slashes( texture );
    if (!Init( name, texture ))
    {
        if (name != "notload")
        { // gdy brak modelu
            if (ter) // jeśli teren
            {
				if( name.ends_with(".t3d") ) {
					name[ name.length() - 3 ] = 'e';
				}
#ifdef EU07_USE_OLD_TERRAINCODE
                Global.asTerrainModel = name;
                WriteLog("Terrain model \"" + name + "\" will be created.");
#endif
            }
            else
                ErrorLog("Missed file: " + name);
        }
    }
    else
    { // wiązanie świateł, o ile model wczytany
        LightsOn[0] = pModel->GetFromName("Light_On00");
        LightsOn[1] = pModel->GetFromName("Light_On01");
        LightsOn[2] = pModel->GetFromName("Light_On02");
        LightsOn[3] = pModel->GetFromName("Light_On03");
        LightsOn[4] = pModel->GetFromName("Light_On04");
        LightsOn[5] = pModel->GetFromName("Light_On05");
        LightsOn[6] = pModel->GetFromName("Light_On06");
        LightsOn[7] = pModel->GetFromName("Light_On07");
        LightsOff[0] = pModel->GetFromName("Light_Off00");
        LightsOff[1] = pModel->GetFromName("Light_Off01");
        LightsOff[2] = pModel->GetFromName("Light_Off02");
        LightsOff[3] = pModel->GetFromName("Light_Off03");
        LightsOff[4] = pModel->GetFromName("Light_Off04");
        LightsOff[5] = pModel->GetFromName("Light_Off05");
        LightsOff[6] = pModel->GetFromName("Light_Off06");
        LightsOff[7] = pModel->GetFromName("Light_Off07");
		sm_winter_variant = pModel->GetFromName("winter_variant");
		sm_spring_variant = pModel->GetFromName("spring_variant");
		sm_summer_variant = pModel->GetFromName("summer_variant");
		sm_autumn_variant = pModel->GetFromName("autumn_variant");
    }
    for (int i = 0; i < iMaxNumLights; ++i)
        if (LightsOn[i] || LightsOff[i]) // Ra: zlikwidowałem wymóg istnienia obu
            iNumLights = i + 1;

    std::string token;
    do {
        token = parser->getToken<std::string>();

        if( token == "lights" ) {
            auto i{ 0 };
            while( false == (token = parser->getToken<std::string>()).empty()
                && false == is_keyword(token) ) {

                if( i < iNumLights ) {
                    // stan światła jest liczbą z ułamkiem
                    LightSet( i, std::stof( token ) );
                }
                ++i;
            }
        }

        if( token == "lightcolors" ) {
            auto i{ 0 };
            while( false == (token = parser->getToken<std::string>()).empty()
                && false == is_keyword(token) ) {

                if( i < iNumLights
                 && token != "-1" ) { // -1 leaves the default color intact
                    auto const lightcolor { std::stoi( token, 0, 16 ) };
                    m_lightcolors[i] = {
                        ( lightcolor >> 16 & 0xff ) / 255.f,
                        ( lightcolor >> 8  & 0xff ) / 255.f,
                        ( lightcolor       & 0xff ) / 255.f };
                }
                ++i;
            }
        }

        if( token == "angles" ) {
            parser->getTokens( 3 );
            *parser
                >> vAngle[ 0 ]
                >> vAngle[ 1 ]
                >> vAngle[ 2 ];
        }

        if( token == "scale" ) {
            // Per-node scale: `scale <x> <y> <z>` (always three tokens, mirroring
            // the `angles` syntax). For uniform scaling, write the same value
            // three times (e.g. `scale 2 2 2`). Combines multiplicatively with
            // any active scale block from the scenariostateserializer (which is
            // applied at deserialize_model time before Load() is called, so
            // m_scale already reflects the outer block when we arrive here).
            parser->getTokens( 3 );
            glm::vec3 factor;
            *parser >> factor.x >> factor.y >> factor.z;
            if( factor.x > 0.0f && factor.y > 0.0f && factor.z > 0.0f ) {
                m_scale.x *= factor.x;
                m_scale.y *= factor.y;
                m_scale.z *= factor.z;
            }
        }

        if( token == "notransition" ) {
            m_transition = false;
        }

    } while( false == token.empty()
          && token != "endmodel" );

    update_instanceable_flag();
    return true;
}

namespace {
// returns true if this animation type mutates per-instance state on the shared
// TSubModel tree, which would make batched rendering unsafe. Most animations
// loaded from .t3d files are global functions of time (clocks, wind, sky) and
// only transform the local modelview matrix — those are safe to share. Camera-
// relative billboards also operate purely on the local matrix using whatever
// modelview the caller pushed, which is exactly per-instance behaviour.
// The runtime SetRotate/SetTranslate animations (at_Rotate / at_RotateXYZ /
// at_Translate) are tied to per-instance iAnimOwner and are unsafe to share.
// at_Undefined is the type assigned to .t3d submodels declared with `anim: true`
// (a generic "this submodel is animatable" hint) and to anything else that
// doesn't match a recognised animation keyword — these are driven by event-
// triggered SetRotate/SetTranslate at runtime, which would silently break if
// the model were batched.
bool anim_type_unsafe_for_instancing(const TAnimType a ) {
    switch( a ) {
    case TAnimType::at_Rotate:
    case TAnimType::at_RotateXYZ:
    case TAnimType::at_Translate:
    case TAnimType::at_DigiClk:    // mutates child submodels via SetRotate
    case TAnimType::at_Undefined:  // `anim: true` / unknown — driven by events
        return true;
    default:
        return false;
    }
}

// recursively walks a submodel tree and returns true if any submodel declares
// an animation type that's unsafe to batch, OR carries the runtime "needs
// animation matrix" flag (iFlags bit 0x4000), which is set whenever the
// submodel was tagged as animatable in the .t3d file or had WillBeAnimated()
// called on it during model load. Either signal means the submodel may receive
// per-instance event-driven animation commands at runtime, which the GPU-
// instanced path (one shared submodel tree across all instances) cannot serve.
bool submodel_tree_blocks_instancing( TSubModel const *Sub ) {
    if( Sub == nullptr ) { return false; }
    if( anim_type_unsafe_for_instancing( Sub->b_Anim ) ) { return true; }
    if( ( Sub->iFlags & 0x4000 ) != 0 ) { return true; }
    if( submodel_tree_blocks_instancing( Sub->Child ) ) { return true; }
    if( submodel_tree_blocks_instancing( Sub->Next ) ) { return true; }
    return false;
}
} // anonymous namespace

int TAnimModel::s_instanceable_total = 0;
int TAnimModel::s_classified_total = 0;
int TAnimModel::s_rejected_no_pmodel = 0;
int TAnimModel::s_rejected_lights = 0;
int TAnimModel::s_rejected_animlist = 0;
int TAnimModel::s_rejected_animated_submodel = 0;

void TAnimModel::update_instanceable_flag() {
    // The instanceable path skips RaAnimate() entirely and only calls RaPrepare()
    // once per bucket. The conditions below ensure that's safe:
    //   - no lights:        per-instance light state machines must not exist
    //   - no anim list:     per-instance submodel animations must not exist
    //   - no animated submodels in the shared TModel3d
    // Replacable skins, seasonal variants, and submodel replacable-skin material
    // refs are intentionally allowed — they're either passed per-instance via
    // Material() or share global state (season) across all instances.
    m_instanceable = false;
    ++s_classified_total;
    if( pModel == nullptr ) { ++s_rejected_no_pmodel; return; }
    if( iNumLights != 0 ) { ++s_rejected_lights; return; }
    if( !m_animlist.empty() ) { ++s_rejected_animlist; return; }
    if( submodel_tree_blocks_instancing( pModel->Root ) ) { ++s_rejected_animated_submodel; return; }
    m_instanceable = true;
    ++s_instanceable_total;
}

std::shared_ptr<TAnimContainer> TAnimModel::AddContainer(std::string const &Name)
{ // dodanie sterowania submodelem dla egzemplarza
    if (!pModel)
		return nullptr;
    TSubModel *tsb = pModel->GetFromName(Name);
    if (tsb)
    {
		auto tmp = std::make_shared<TAnimContainer>();
        tmp->Init(tsb);
		m_animlist.push_back(tmp);
		return tmp;
    }
	return nullptr;
}

std::shared_ptr<TAnimContainer> TAnimModel::GetContainer(std::string const &Name)
{ // szukanie/dodanie sterowania submodelem dla egzemplarza
	if (true == Name.empty())
		return !m_animlist.empty() ? m_animlist.front() : nullptr; // pobranie pierwszego (dla obrotnicy)

	for (auto entry : m_animlist) {
		if (entry->NameGet() == Name)
			return entry;
	}

	return AddContainer(Name);
}

// przeliczenie animacji - jednorazowo na klatkę
void TAnimModel::RaAnimate( unsigned int const Framestamp ) {
    
    if( Framestamp == m_framestamp ) { return; }

    auto const timedelta { Timer::GetDeltaTime() };

    // interpretacja ułamka zależnie od typu
    // case ls_Off: ustalenie czasu migotania, t<1s (f>1Hz), np. 0.1 => t=0.1 (f=10Hz)
    // case ls_On: ustalenie wypełnienia ułamkiem, np. 1.25 => zapalony przez 1/4 okresu
    // case ls_Blink: ustalenie częstotliwości migotania, f<1Hz (t>1s), np. 2.2 => f=0.2Hz (t=5s)
    for (int idx = 0; idx < iNumLights; ++idx) {
		float modeintegral;
        const float modefractional = std::modf(std::abs(lsLights[idx]), &modeintegral);
    	const auto mode = static_cast<TLightState>(modeintegral);
        if (mode == ls_Dark || mode == ls_Home)
            continue; // light threshold modes don't use timers

        float &opacity { m_lightopacities[ idx ] };
        float &timer { m_lighttimers[ idx ] };
    	// Phase logic
    	float ontime =
    		modefractional < 0.01f ? fOnTime :
    		mode == ls_Off ? modefractional * 0.5f :
    		mode == ls_On ? modefractional * (fOnTime + fOffTime) :
    		mode == ls_Blink ? 0.5f / modefractional :
    		fOnTime; // fallback
    	float offtime =
    		modefractional < 0.01f ? fOffTime :
    		mode == ls_Off ? modefractional * 0.5f :
    		mode == ls_On ? (1 - modefractional) * (fOnTime + fOffTime) :
    		mode == ls_Blink ? 0.5f / modefractional :
    		fOffTime; // fallback
    	// Determine whether the light is currently on and update the timers
    	bool on = false;
    	if ((mode == ls_Off || mode == ls_On) && modefractional < 0.01f)
    		on = mode == ls_On;
    	else {
    		timer = fmod(timer + timedelta, ontime + offtime); // 0..(ontime+offtime)
			const float time = fmod(timer + ( lsLights[ idx ] > 0.f ? 0.f : 0.5f ), ontime + offtime); // time with correction for phase shift for negative light values
    		on = time < ontime;
    	}
    	// Update the light brightness
    	const float transitionontime = std::min(0.25f, std::min(ontime, offtime) * 0.95f);
    	const float transitionofftime = std::min(0.45f, std::min(ontime, offtime) * 0.95f);
    	if (on)
    		opacity += m_transition ? timedelta / transitionontime : 1.f; // increase to max value
    	else
    		opacity -= m_transition ? timedelta / transitionofftime : 1.f; // reduce to zero
    	// Clamp the opacity
		opacity = std::clamp(opacity, 0.f, 1.f);
    }

    // Ra 2F1I: to by można pomijać dla modeli bez animacji, których jest większość
	for (const auto entry : m_animlist) {
		if (!entry->evDone) // jeśli jest bez eventu
			entry->UpdateModel(); // przeliczenie animacji każdego submodelu
	}

    m_framestamp = Framestamp;
}

// aktualizujemy submodele w zaleznosci od aktualnej porty roku
void TAnimModel::on_season_update() {
	if (this->sm_winter_variant != nullptr) // pokazujemy wariant zimowy
		this->sm_winter_variant->SetVisibilityLevel(Global.Season == "winter:" ? 1 : 0, true, false);
	if (this->sm_spring_variant != nullptr) // pokazujemy wariant wiosenny
		this->sm_spring_variant->SetVisibilityLevel(Global.Season == "spring:" ? 1 : 0, true, false);
	if (this->sm_summer_variant != nullptr) // pokazujemy wariant letni
		this->sm_summer_variant->SetVisibilityLevel(Global.Season == "summer:" ? 1 : 0, true, false);
	if (this->sm_autumn_variant != nullptr) // pokazujemy wariant jesienny
		this->sm_autumn_variant->SetVisibilityLevel(Global.Season == "autumn:" ? 1 : 0, true, false);
}

void TAnimModel::RaPrepare()
{ // ustawia światła i animacje we wzorcu modelu przed renderowaniem egzemplarza
    bool state; // stan światła
	if (Global.UpdateMaterials)
	    on_season_update();
    for (int i = 0; i < iNumLights; ++i)
    {
        auto const lightmode { static_cast<int>( std::abs( lsLights[ i ] ) ) };
        switch( lightmode ) {
            case ls_On:
            case ls_Off:
            case ls_Blink: {
                if (LightsOn[i]) {
                    LightsOn[i]->iVisible = m_lightopacities[i] > 0.f;
                    LightsOn[i]->SetVisibilityLevel( m_lightopacities[i], true, false );
                }
                if (LightsOff[i]) {
                    LightsOff[i]->iVisible = m_lightopacities[i] < 1.f;
                    LightsOff[i]->SetVisibilityLevel( 1.f, true, false );
                }
                break;
            }
            case ls_Dark: {
                // zapalone, gdy ciemno
                state =
			        Global.fLuminance - std::max(0.f, Global.Overcast - 1.f) <= (lsLights[i] == static_cast<float>(ls_Dark) ? DefaultDarkThresholdLevel : lsLights[i] - static_cast<float>(ls_Dark));
                break;
            }
            case ls_Home: {
                // like ls_dark but off late at night
                auto const simulationhour { simulation::Time.data().wHour };
                state =
			        Global.fLuminance - std::max(0.f, Global.Overcast - 1.f) <= (lsLights[i] == static_cast<float>(ls_Home) ? DefaultDarkThresholdLevel : lsLights[i] - static_cast<float>(ls_Home));
                // force the lights off between 1-5am
                state = state && (simulationhour < 1 || simulationhour >= 5);
                break;
            }
            default: {
                break;
            }
        }
        if( lightmode >= ls_Dark ) {
            // crude as hell but for test will do :x
            if (LightsOn[i]) {
                LightsOn[i]->iVisible = state;
                // TODO: set visibility for the entire submodel's children as well
                LightsOn[i]->fVisible = m_lightopacities[i];
            }
            if (LightsOff[i])
                LightsOff[i]->iVisible = !state;
        }
        // potentially modify freespot colors
        if( LightsOn[i] ) {
            LightsOn[i]->SetDiffuseOverride( m_lightcolors[i], true);
        }
    }
    TSubModel::iInstance = reinterpret_cast<std::uintptr_t>( this ); //żeby nie robić cudzych animacji
    TSubModel::pasText = &asText; // przekazanie tekstu do wyświetlacza (!!!! do przemyślenia)

	for (const auto entry : m_animlist) {
		entry->PrepareModel();
	}
}

int TAnimModel::Flags()
{ // informacja dla TGround, czy ma być w Render, RenderAlpha, czy RenderMixed
    int i = pModel ? pModel->Flags() : 0; // pobranie flag całego modelu
    if( m_materialdata.replacable_skins[ 1 ] > 0 ) // jeśli ma wymienną teksturę 0
        i |= (i & 0x01010001) * (m_materialdata.textures_alpha & 1 ? 0x20 : 0x10);
    return i;
}

//---------------------------------------------------------------------------

int TAnimModel::TerrainCount()
{ // zliczanie kwadratów kilometrowych (główna linia po Next) do tworznia tablicy
    return pModel ? pModel->TerrainCount() : 0;
}

TSubModel * TAnimModel::TerrainSquare(const int n)
{ // pobieranie wskaźników do pierwszego submodelu
    return pModel ? pModel->TerrainSquare(n) : 0;
}

//---------------------------------------------------------------------------
void TAnimModel::LightSet(int const n, float const v)
{ // ustawienie światła (n) na wartość (v)
    if( n >= iMaxNumLights ) {
        return; // przekroczony zakres
    }
    lsLights[ n ] = v;
}

std::optional<std::tuple<float, float, std::optional<glm::vec3>> > TAnimModel::LightGet(const int n)
{
	if (n >= iMaxNumLights)
		return std::nullopt;
	if (!LightsOn[n] && !LightsOff[n])
		return std::nullopt;

	std::optional<glm::vec3> color;

	if (m_lightcolors[n].r >= 0.0f)
		color.emplace(m_lightcolors[n]);

	if (!color && LightsOn[n])
		color = LightsOn[n]->GetDiffuse();

	return std::make_tuple(lsLights[n], m_lightopacities[n], color);
}

void TAnimModel::SkinSet( int const Index, material_handle const Material ) {

    m_materialdata.replacable_skins[ std::clamp( Index, 1, 4 ) ] = Material;
}

void TAnimModel::AnimUpdate(double dt)
{ // wykonanie zakolejkowanych animacji, nawet gdy modele nie są aktualnie wyświetlane
	acAnimList.remove_if([](std::weak_ptr<TAnimContainer> ptr)
	{
		    const std::shared_ptr<TAnimContainer> container = ptr.lock();
		if (!container)
			return true;

		container->UpdateModel(); // na razie bez usuwania z listy, bo głównie obrotnica na nią wchodzi
		return false;
	});
}

// radius() subclass details, calculates node's bounding radius.
// For non-uniform scale we use the largest axis factor so the bounding sphere
// fully contains the scaled model — undersizing would cause incorrect culling.
float
TAnimModel::radius_() {

    if( pModel == nullptr ) { return 0.f; }
    float const max_scale = std::max( { m_scale.x, m_scale.y, m_scale.z } );
    return pModel->bounding_radius() * max_scale;
}

// serialize() subclass details, sends content of the subclass to provided stream
void
TAnimModel::serialize_( std::ostream &Output ) const {

    // TODO: implement
}
// deserialize() subclass details, restores content of the subclass from provided stream
void
TAnimModel::deserialize_( std::istream &Input ) {

    // TODO: implement
}

// export() subclass details, sends basic content of the class in legacy (text) format to provided stream.
// Smart export: omit fields that match defaults so reloaded scenarios stay clean.
//   - if X and Z rotation are zero, fold Y rotation into the 4th token slot
//     (the legacy `node ... model X Y Z <rotation.y> ...` format) and skip the
//     `angles` block entirely
//   - if any axis of rotation needs all three components, emit `angles X Y Z`
//   - emit `scale X Y Z` only when m_scale isn't (1,1,1)
void
TAnimModel::export_as_text_( std::ostream &Output ) const {

    // header
    Output << "model ";

    // location and rotation. The 4th token after location is a legacy
    // shorthand for the Y rotation. We use it (and skip the angles block)
    // whenever the rotation is purely around Y, which is the common case.
    bool const xz_rotation_zero = vAngle.x == 0.0f && vAngle.z == 0.0f;
    Output << std::fixed << std::setprecision( 3 )
        << location().x << ' '
        << location().y << ' '
        << location().z << ' '
        << ( xz_rotation_zero ? vAngle.y : 0.0f ) << ' ';

    // 3d shape
    auto modelfile { (
        pModel ?
            pModel->NameGet() + ".t3d" : // rainsted requires model file names to include an extension
            "none" ) };
    if( modelfile.find( paths::models ) == 0 ) {
        // don't include 'models/' in the path
        modelfile.erase( 0, std::string{ paths::models }.size() );
    }
    Output << modelfile << ' ';
    // texture
    auto texturefile { (
        m_materialdata.replacable_skins[ 1 ] != null_handle ?
            GfxRenderer->Material( m_materialdata.replacable_skins[ 1 ] )->GetName() :
            "none" ) };
    if( texturefile.find( paths::textures ) == 0 ) {
        // don't include 'textures/' in the path
        texturefile.erase( 0, std::string{ paths::textures }.size() );
    }
    if( contains( texturefile, ' ' ) ) {
        Output << "\"" << texturefile << "\"" << ' ';
    }
    else {
        Output << texturefile << ' ';
    }
    // light submodels activation configuration
    if( iNumLights > 0 ) {
        Output << "lights ";
        for( int lightidx = 0; lightidx < iNumLights; ++lightidx ) {
            Output << lsLights[ lightidx ] << ' ';
        }
    }
    // potential light transition switch
    if( false == m_transition ) {
        Output << "notransition" << ' ';
    }
    // angles directive only when X or Z are rotated — otherwise the Y angle
    // already lives in the 4th token slot above.
    if( false == xz_rotation_zero ) {
        Output << "angles "
            << vAngle.x << ' '
            << vAngle.y << ' '
            << vAngle.z << ' ';
    }
    // scale directive only when actually scaled — keeps default-scale models
    // from being polluted with redundant `scale 1 1 1` entries on every save.
    if( m_scale.x != 1.0f || m_scale.y != 1.0f || m_scale.z != 1.0f ) {
        Output << "scale "
            << m_scale.x << ' '
            << m_scale.y << ' '
            << m_scale.z << ' ';
    }
    // footer
    Output
        << "endmodel"
        << "\n";
}

//---------------------------------------------------------------------------
