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
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/

// the animated parts of a vehicle: pantographs, doors, platforms, mirrors, wipers, axles, levers, hoses, lights

#include "stdafx.h"
#include "vehicle/DynObj.h"
#include "physics/mover.h"
#include "vehicle/lightsprogrammer.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "rendering/lightarray.h"
#include "vehicle/Camera.h"
#include "vehicle/Train.h"
#include "vehicle/odometer.h"
#include "vehicle/pulleemode.h"
#include "vehicle/Driver.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <random>
#include <iterator>
#include <numbers>
#include <optional>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "utilities/workerpool.h"
#include "utilities/Logs.h"
#include "utilities/glmHelpers.h"
#include "Console.h"
#include "world/Traction.h"
#include "audio/sound.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "vehicle/doorwings.h"
#include "rendering/renderer.h"
#include "editor/editorTerrainStreamer.hpp"
#include "application/uitranscripts.h"
#include "input/messaging.h"
#include "application/application.h"
#include "vehicle/Driver.h"
#include "vehicle/dynamic/internal.h"


//---------------------------------------------------------------------------
void TAnimPant::common_pantograph_settings()
{ // wspólne ustawienia, liczone po przypisaniu wymiarów ramion
	vPos = glm::dvec3(0, 0, 0);
	fAngleL0 = glm::radians(2.8547285515689267247882521833308);
	fAngleL = fAngleL0; // początkowy kąt dolnego ramienia
	fAngleU0 = acos((fLenL1 * cos(fAngleL) + fHoriz) / fLenU1); // górne ramię
	fAngleU = fAngleU0; // początkowy kąt
	PantWys = fLenL1 * sin(fAngleL) + fLenU1 * sin(fAngleU) + fHeight; // wysokość początkowa
	PantTraction = PantWys;
	hvPowerWire = nullptr;
	fWidthExtra = 0.381f; //(2.032m-1.027)/2
	// poza obszarem roboczym jest aproksymacja łamaną o 5 odcinkach
	fHeightExtra[0] = 0.0f;  //+0.0762
	fHeightExtra[1] = -0.01f; //+0.1524
	fHeightExtra[2] = -0.03f; //+0.2286
	fHeightExtra[3] = -0.07f; //+0.3048
	fHeightExtra[4] = -0.15f; //+0.3810
}

void TAnimPant::AKP_4E()
{ // ustawienie wymiarów dla pantografu AKP-4E
    fLenL1 = 1.22; // 1.176289 w modelach
    fLenU1 = 1.755; // 1.724482197 w modelach
    fHoriz = 0.535; // 0.54555075 przesunięcie ślizgu w długości pojazdu względem
    // osi obrotu dolnego
    // ramienia
    fHeight = 0.07; // wysokość ślizgu ponad oś obrotu
    fWidth = 0.635; // połowa szerokości ślizgu, 0.635 dla AKP-1 i AKP-4E
	common_pantograph_settings();
};
void TAnimPant::WBL85()
{ // ustawienie wymiarów dla pantografu WBL88
    // mnozniki animacji ramion dla pantografu WBL88
	rd1rf = 1.f;
	rd2rf = 1.2;
	rg1rf = 0.875;
	rg2rf = 1.0;
	slizgrf = 1.f;

	fLenL1 = 1.98374;
	fLenU1 = 2.14199;
	fHoriz = 0.142; // 0.54555075 przesunięcie ślizgu w długości pojazdu względem
	                // osi obrotu dolnego ramienia
	fHeight = 0.09353; // wysokość ślizgu ponad oś obrotu
	fWidth = 0.4969; // połowa szerokości ślizgu
	common_pantograph_settings();
};
void TAnimPant::EC160_200()
{ // ustawienie wymiarów dla pantografow EC160 lub EC200
	// mnozniki animacji ramion dla pantografow EC160 lub EC200
	rd1rf = 1.f;
	rd2rf = 0.85;
	rg1rf = 1.f;
	rg2rf = 1.f; // 0.833
	slizgrf = 1.f;

	fLenL1 = 1.98374;
	fLenU1 = 2.14199;
	fHoriz = 0.142; // 0.54555075 przesunięcie ślizgu w długości pojazdu względem
	                // osi obrotu dolnego ramienia
	fHeight = 0.09353; // wysokość ślizgu ponad oś obrotu
	fWidth = 0.4969; // połowa szerokości ślizgu
	common_pantograph_settings();
};
void TAnimPant::DSAx()
{ // ustawienie wymiarów dla pantografow z rodziny DSA
	// mnozniki animacji ramion dla pantografow z rodziny DSA
	rd1rf = 1.f;
	rd2rf = 1.025;
	rg1rf = 0.875;
	rg2rf = 1.f;
	slizgrf = 1.f;

	fLenL1 = 1.98374;
	fLenU1 = 2.14199;
	fHoriz = 0.142; // 0.54555075 przesunięcie ślizgu w długości pojazdu względem
	                // osi obrotu dolnego ramienia
	fHeight = 0.09353; // wysokość ślizgu ponad oś obrotu
	fWidth = 0.4969; // połowa szerokości ślizgu
	common_pantograph_settings();
};
//---------------------------------------------------------------------------
int TAnim::TypeSet(int i, TMoverParameters currentMover, int fl)
{ // ustawienie typu animacji i zależnej od niego ilości animowanych submodeli
    fMaxDist = -1.0; // normalnie nie pokazywać
    switch (i)
    { // maska 0x000F: ile używa wskaźników na submodele (0 gdy jeden,
    // wtedy bez tablicy)
    // maska 0x00F0:
    // 0-osie,1-drzwi,2-obracane,3-zderzaki,4-wózki,5-pantografy,6-tłoki
    // maska 0xFF00: ile używa liczb float dla współczynników i stanu
    case 0:
        iFlags = 0x000;
        break; // 0-oś
    case 1:
        iFlags = 0x010;
        break; // 1-drzwi
    case 2:
        iFlags = 0x020;
        fParam = fl ? new float[fl] : nullptr;
        iFlags += fl << 8;
        break; // 2-wahacz, dźwignia itp.
    case 3:
        iFlags = 0x030;
        break; // 3-zderzak
    case 4:
        iFlags = 0x040;
        break; // 4-wózek
    case 5: // 5-pantograf - 5 submodeli
        iFlags = 0x055;
        fParamPants = new TAnimPant();
        switch (currentMover.EnginePowerSource.CollectorParameters.PantographType) {
		    case TPantType::AKP_4E:
			    fParamPants->AKP_4E();
                break;
		    case TPantType::DSAx:
                fParamPants->DSAx();
			    break;
            case TPantType::EC160_200:
                fParamPants->EC160_200();
			    break;
            case TPantType::WBL85:
                fParamPants->WBL85();
			    break;
        }

        break;
    case 6:
        iFlags = 0x068;
        break; // 6-tłok i rozrząd - 8 submodeli
    case 7:
        iFlags = 0x070;
        break; // doorstep
    case 8:
        iFlags = 0x080;
        break; // mirror
    case 9:
		iFlags = 0x023; // 2: Rotating/Motion-based, 8: Uses 3 submodels
		break;
    default:
        iFlags = 0;
    }
    yUpdate = nullptr;
    return iFlags & 15; // ile wskaźników rezerwować dla danego typu animacji
};

TAnim::~TAnim()
{ // usuwanie animacji
    switch (iFlags & 0xF0)
    { // usuwanie struktur, zależnie ile zostało stworzonych
    case 0x20: // 2-wahacz, dźwignia itp.
        delete fParam;
        break;
    case 0x50: // 5-pantograf
        delete fParamPants;
        break;
    default:
        break;
    }
};

int TDynamicObject::GetPneumatic(bool front, bool red)
{
    int x, y, z; // 1=prosty, 2=skośny
    if (red)
    {
        if (front)
        {
            x = btCPneumatic1.GetStatus();
            y = btCPneumatic1r.GetStatus();
        }
        else
        {
            x = btCPneumatic2.GetStatus();
            y = btCPneumatic2r.GetStatus();
        }
    }
    else if (front)
    {
        x = btPneumatic1.GetStatus();
        y = btPneumatic1r.GetStatus();
    }
    else
    {
        x = btPneumatic2.GetStatus();
        y = btPneumatic2r.GetStatus();
    }
    z = 0; // brak węży?
    if (x > 0 && y > 0)
        z = 3; // dwa
    if (x > 0 && y == 0)
        z = 1; // lewy, brak prawego
    if (x == 0 && y > 0)
        z = 2; // brak lewego, prawy

    return z;
}

void TDynamicObject::SetPneumatic(bool front, bool red)
{
	int x = 0,
		ten = 0,
		tamten = 0;
    ten = GetPneumatic(front, red); // 1=lewy skos,2=prawy skos,3=dwa proste
    if (front)
        if (PrevConnected()) // pojazd od strony sprzęgu 0
            tamten = PrevConnected()->GetPneumatic(PrevConnectedNo() == end::front ? true : false, red);
    if (!front)
        if (NextConnected()) // pojazd od strony sprzęgu 1
            tamten = NextConnected()->GetPneumatic(NextConnectedNo() == end::front ? true : false, red);
    if (ten == tamten) // jeśli układ jest symetryczny
        switch (ten)
        {
        case 1:
            x = 2;
            break; // mamy lewy skos, dać lewe skosy
        case 2:
            x = 3;
            break; // mamy prawy skos, dać prawe skosy
        case 3: // wszystkie cztery na prosto
            if (MoverParameters->Couplers[front ? end::front : end::rear].Render)
                x = 1;
            else
                x = 4;
            break;
        }
    else
    {
        if (ten == 2)
            x = 4;
        if (ten == 1)
            x = 1;
        if (ten == 3)
            if (tamten == 1)
                x = 4;
            else
                x = 1;
    }
    if (front)
    {
        if (red)
            cp1 = x;
        else
            sp1 = x;
    } // który pokazywać z przodu
    else
    {
        if (red)
            cp2 = x;
        else
            sp2 = x;
    } // który pokazywać z tyłu
}

void TDynamicObject::UpdateAxle(TAnim *pAnim)
{ // animacja osi
    size_t wheel_id = pAnim->dWheelAngle;
    pAnim->smAnimated->SetRotate(float3(1, 0, 0), dWheelAngle[wheel_id]);
    pAnim->smAnimated->future_transform = glm::rotate((float)glm::radians(m_future_wheels_angle[wheel_id]), glm::vec3(1.0f, 0.0f, 0.0f));
};

// angle of a slotted lever for given angle of the wheel, which drives the coupling rod with the pin
// all points of the coupling rod move along circles of crank radius, the lever follows the pin sliding in its slot
double TDynamicObject::slottedlever_angle( slottedlever_data const &Lever, double const Wheelangle ) {
    // crank turned around X axis, the same way as the wheel
    auto const wheelangle { glm::radians( Wheelangle ) };
    auto const crank { glm::dvec2(
        Lever.crank.x * std::cos( wheelangle ) - Lever.crank.y * std::sin( wheelangle ),
        Lever.crank.x * std::sin( wheelangle ) + Lever.crank.y * std::cos( wheelangle ) ) };
    auto const pin { Lever.pin - Lever.crank + crank };
    // angle relative to the pose of the lever in the model, which matches wheel angle 0
    return glm::degrees( std::atan2( pin.y, pin.x ) - std::atan2( Lever.pin.y, Lever.pin.x ) );
}

void TDynamicObject::update_slottedlevers() {

    for( auto const &lever : m_slottedlevers ) {
        auto const wheelid { (
            ( lever.axle < iAnimType[ ANIM_WHEELS ] ) && ( lever.axle < static_cast<int>( pAnimations.size() ) ) ?
                pAnimations[ lever.axle ].dWheelAngle :
                1 ) }; // driving wheels and coupling rods by default
        auto const angle { slottedlever_angle( lever, dWheelAngle[ wheelid ] ) };
        auto const futureangle { slottedlever_angle( lever, dWheelAngle[ wheelid ] + m_future_wheels_angle[ wheelid ] ) };
        lever.submodel->SetRotate( float3( 1, 0, 0 ), angle );
        lever.submodel->future_transform = glm::rotate( (float)glm::radians( std::remainder( futureangle - angle, 360.0 ) ), glm::vec3( 1.0f, 0.0f, 0.0f ) );
    }
}
namespace
{


// the position of the door the animated wing belongs to: its shift or angle from closed
float door_position( TMoverParameters const &Mover, TAnim const *Animation ) {

    return Mover.Doors.instances[ (
        door_wings::on_right_side( Animation->iNumber ) ?
            side::right :
            side::left ) ].position;
}

// the wing in the main model, and its namesake in the low poly interior; either can be missing
std::array<TSubModel *, 2> wing_parts( TAnim const *Animation ) {

    return { Animation->smAnimated, Animation->smInterior };
}

// folding door: the smaller wing turns, the larger wing (its child) the other way, the step (child of that) under it
void fold_wing( TSubModel *Wing, door_wings::fold_angles const &Angles ) {

    if( Wing == nullptr ) { return; }
    // skrzydło mniejsze
    Wing->SetRotate( float3( 0, 0, 1 ), Angles.smaller );
    // skrzydło większe
    auto *larger { Wing->ChildGet() };
    if( larger == nullptr ) { return; }
    larger->SetRotate( float3( 0, 0, 1 ), Angles.larger );
    // podnóżek?
    auto *step { larger->ChildGet() };
    if( step == nullptr ) { return; }
    step->SetRotate( float3( 0, 1, 0 ), Angles.step );
}

} // namespace

// animacja drzwi - przesuw
void TDynamicObject::UpdateDoorTranslate(TAnim *pAnim) {

    if( pAnim->smAnimated == nullptr ) { return; }

    auto const shift { door_wings::slide( door_position( *MoverParameters, pAnim ) ) };
    for( auto *wing : wing_parts( pAnim ) ) {
        if( wing != nullptr ) {
            wing->SetTranslate( shift );
        }
    }
};

// animacja drzwi - obrót
void TDynamicObject::UpdateDoorRotate(TAnim *pAnim) {

    if( pAnim->smAnimated == nullptr ) { return; }

    auto const angle { door_position( *MoverParameters, pAnim ) };
    for( auto *wing : wing_parts( pAnim ) ) {
        if( wing != nullptr ) {
            wing->SetRotate( float3( 1, 0, 0 ), angle );
        }
    }
};

// animacja drzwi - obrót
void TDynamicObject::UpdateDoorFold(TAnim *pAnim) {

    if( pAnim->smAnimated == nullptr ) { return; }

    auto const angles { door_wings::fold( door_position( *MoverParameters, pAnim ) ) };
    for( auto *wing : wing_parts( pAnim ) ) {
        fold_wing( wing, angles );
    }
};

// animacja drzwi - odskokprzesuw
void TDynamicObject::UpdateDoorPlug(TAnim *pAnim) {

    if( pAnim->smAnimated == nullptr ) { return; }

    auto const shift { door_wings::plug( door_position( *MoverParameters, pAnim ), MoverParameters->Doors.range_out ) };
    for( auto *wing : wing_parts( pAnim ) ) {
        if( wing != nullptr ) {
            wing->SetTranslate( shift );
        }
    }
}

// a wing animated in its own way (animdoor: of the .mmd): as far along its own range as the door of its side is
void TDynamicObject::UpdateDoorWing( TAnim *pAnim, door_wings::wing_spec const &Spec ) {

    if( pAnim->smAnimated == nullptr ) { return; }

    auto const position { door_wings::wing_position( door_position( *MoverParameters, pAnim ), MoverParameters->Doors.range, Spec.range ) };
    for( auto *wing : wing_parts( pAnim ) ) {
        if( wing == nullptr ) { continue; }
        switch( Spec.type ) {
            case door_wings::shift_type: {
                wing->SetTranslate( door_wings::slide( position ) );
                break;
            }
            case door_wings::rotate_type: {
                wing->SetRotate( float3( 1, 0, 0 ), position );
                break;
            }
            case door_wings::fold_type: {
                fold_wing( wing, door_wings::fold( position ) );
                break;
            }
            case door_wings::plug_type: {
                wing->SetTranslate( door_wings::plug( position, Spec.range_out ) );
                break;
            }
            default: {
                break;
            }
        }
    }
}

void TDynamicObject::bind_door_animation( TAnim &Animation ) {

    if( auto const *spec { door_wings::find( m_doorwings, Animation.iNumber ) } ) {
        Animation.yUpdate = [ this, spec = *spec ]( TAnim *pAnim ) { UpdateDoorWing( pAnim, spec ); };
        return;
    }
    switch( MoverParameters->Doors.type ) { // od razu zapinamy potrzebny typ animacji
        case 1:
            Animation.yUpdate = std::bind( &TDynamicObject::UpdateDoorTranslate, this, std::placeholders::_1 );
            break;
        case 2:
            Animation.yUpdate = std::bind( &TDynamicObject::UpdateDoorRotate, this, std::placeholders::_1 );
            break;
        case 3:
            Animation.yUpdate = std::bind( &TDynamicObject::UpdateDoorFold, this, std::placeholders::_1 );
            break; // obrót 3 kolejnych submodeli
        case 4:
            Animation.yUpdate = std::bind( &TDynamicObject::UpdateDoorPlug, this, std::placeholders::_1 );
            break;
        default:
            break;
    }
}

void TDynamicObject::UpdatePant(TAnim *pAnim)
{ // animacja pantografu - 4 obracane ramiona, ślizg piąty
	float a, b, c;
	a = glm::degrees(pAnim->fParamPants->fAngleL - pAnim->fParamPants->fAngleL0);
	b = glm::degrees(pAnim->fParamPants->fAngleU - pAnim->fParamPants->fAngleU0);
	c = a + b;
	if (pAnim->smElement[0])
		pAnim->smElement[0]->SetRotate(float3(-1, 0, 0), a); // dolne ramie 1
	if (pAnim->smElement[1])
		pAnim->smElement[1]->SetRotate(float3(1, 0, 0), a * pAnim->fParamPants->rd2rf); // dolne ramie 2
	if (pAnim->smElement[2])
		pAnim->smElement[2]->SetRotate(float3(1, 0, 0), c); // górne ramie 1
	if (pAnim->smElement[3])
		pAnim->smElement[3]->SetRotate(float3(-1, 0, 0), c * pAnim->fParamPants->rg2rf); // gorne ramie 2
	if (pAnim->smElement[4])
		pAnim->smElement[4]->SetRotate(float3(-1, 0, 0), b * pAnim->fParamPants->slizgrf); // ślizg
}

// doorstep animation, shift
void TDynamicObject::UpdatePlatformTranslate( TAnim *pAnim ) {

    if( pAnim->smAnimated == nullptr ) { return; }

    auto const &door { MoverParameters->Doors.instances[ (
        ( pAnim->iNumber & 1 ) == 0 ?
            side::right :
            side::left ) ] };

    pAnim->smAnimated->SetTranslate(
        glm::vec3{
            std::lerp( 0.f, MoverParameters->Doors.step_range, door.step_position ),
            0.0,
            0.0 } );
}

// doorstep animation, rotate
void TDynamicObject::UpdatePlatformRotate( TAnim *pAnim ) {

    if( pAnim->smAnimated == nullptr ) { return; }

    auto const &door { MoverParameters->Doors.instances[ (
        ( pAnim->iNumber & 1 ) == 0 ?
            side::right :
            side::left ) ] };

    pAnim->smAnimated->SetRotate(
        float3( 0, 1, 0 ),
        std::lerp( 0.f, MoverParameters->Doors.step_range, door.step_position ) );
}

// mirror animation, rotate
void TDynamicObject::UpdateMirror( TAnim *pAnim ) {

    if( pAnim->smAnimated == nullptr ) { return; }

    // only animate the mirror if it's located on the same end of the vehicle as the active cab
    auto const isactive { (
        MoverParameters->CabOccupied > 0 ? ( pAnim->iNumber >> 4 == end::front ? 1.0 : 0.0 ) :
        MoverParameters->CabOccupied < 0 ? ( pAnim->iNumber >> 4 == end::rear  ? 1.0 : 0.0 ) :
        0.0 ) };

    if( pAnim->iNumber & 1 )
        pAnim->smAnimated->SetRotate(
            float3( 0, 1, 0 ),
            std::lerp( 0.0, MoverParameters->MirrorMaxShift, dMirrorMoveR * isactive ) );
    else
        pAnim->smAnimated->SetRotate(
            float3( 0, 1, 0 ),
            std::lerp( 0.0, MoverParameters->MirrorMaxShift, dMirrorMoveL * isactive ) );
}

// wipers
void TDynamicObject::UpdateWiper(TAnim* pAnim)
{
	if (!pAnim || !pAnim->smElement)
		return;

	int i = pAnim->iNumber; 
    // odwaramy animacje dla parzystych indexow
	const double rotateAngle = (i + 1) % 2 == 0 ? -MoverParameters->WiperAngle : MoverParameters->WiperAngle;

	if (pAnim->smElement[0]) // ramie 1
		pAnim->smElement[0]->SetRotate(float3(0, 1, 0), smoothInterpolate(0.0, rotateAngle, dWiperPos[i]));
	if (pAnim->smElement[1]) // ramie 2
		pAnim->smElement[1]->SetRotate(float3(0, 1, 0), smoothInterpolate(0.0, rotateAngle, dWiperPos[i]));
	if (pAnim->smElement[2]) // pioro
		pAnim->smElement[2]->SetRotate(float3(0, 1, 0), smoothInterpolate(0.0, -rotateAngle, dWiperPos[i]));
}

/*
void TDynamicObject::UpdateLeverDouble(TAnim *pAnim)
{ // animacja gałki zależna od double
    pAnim->smAnimated->SetRotate(float3(1, 0, 0), pAnim->fSpeed * *pAnim->fDoubleBase);
};
void TDynamicObject::UpdateLeverFloat(TAnim *pAnim)
{ // animacja gałki zależna od float
    pAnim->smAnimated->SetRotate(float3(1, 0, 0), pAnim->fSpeed * *pAnim->fFloatBase);
};
void TDynamicObject::UpdateLeverInt(TAnim *pAnim)
{ // animacja gałki zależna od int
    pAnim->smAnimated->SetRotate(float3(1, 0, 0), pAnim->fSpeed * *pAnim->iIntBase);
};
void TDynamicObject::UpdateLeverEnum(TAnim *pAnim)
{ // ustawienie kąta na
    // wartość wskazaną przez
    // int z tablicy fParam
    // pAnim->fParam[0]; - dodać lepkość
    pAnim->smAnimated->SetRotate(float3(1, 0, 0), pAnim->fParam[*pAnim->iIntBase]);
};
*/
// sets light levels for registered interior sections
void
TDynamicObject::toggle_lights() {

    if( true == SectionLightsActive ) {
        // switch all lights off...
        for( auto &section : Sections ) {
            auto const sectionname { section.compartment->pName };
            if( sectionname.rfind( "cab", 0 ) != 0 ) {
                section.light_level = 0.0;
            }
        }
        SectionLightsActive = false;
    }
    else {
        // set lights with probability depending on the compartment type. TODO: expose this in .mmd file
        for( auto &section : Sections ) {

            auto const sectionname { section.compartment->pName };
            if( sectionname.find("corridor") == 0
             || sectionname.find("korytarz") == 0 ) {
                // corridors are lit 100% of time
                section.light_level = 0.75f;
            }
            else if(
                sectionname.find("compartment") == 0
             || sectionname.find("przedzial") == 0 ) {
                // compartments are lit with 75% probability
                section.light_level = Random() < 0.75 ? 0.75f : 0.10f;
            }
        }
        SectionLightsActive = true;
    }
}

void
TDynamicObject::set_cab_lights( int const Cab, float const Level ) {

    for( auto &section : Sections ) {
        // cab compartments are placed at the beginning of the list, so we can bail out as soon as we find different compartment type
        auto const sectionname { section.compartment->pName };
        if( sectionname.size() < 4 ) { return; }
        if( sectionname.find( "cab" ) != 0 ) { return; }
        if( sectionname[ 3 ] != Cab + '0' ) { continue; } // match the cab with correct index

        section.light_level = Level;
    }
}

// aktualizujemy submodele w zaleznosci od aktualnej pory roku
void TDynamicObject::on_season_update()
{
    if (sm_winter_variant != nullptr) // pokazujemy wariant zimowy
        sm_winter_variant->SetVisibilityLevel(Global.Season == "winter:" ? 1 : 0, true, false);
    if (sm_spring_variant != nullptr) // pokazujemy wariant wiosenny
        sm_spring_variant->SetVisibilityLevel(Global.Season == "spring:" ? 1 : 0, true, false);
    if (sm_summer_variant != nullptr) // pokazujemy wariant letni
        sm_summer_variant->SetVisibilityLevel(Global.Season == "summer:" ? 1 : 0, true, false);
    if (sm_autumn_variant != nullptr) // pokazujemy wariant jesienny
        sm_autumn_variant->SetVisibilityLevel(Global.Season == "autumn:" ? 1 : 0, true, false);
}
