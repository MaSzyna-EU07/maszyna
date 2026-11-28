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

// reading the definition of the model and the sounds of a vehicle (.mmd)

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

namespace
{


// a luadestinationsign: or wasmdestinationsign: block, its script in the vehicle folder when it has no path of its own
void deserialize_script_sign( cParser &Parser, script_screens::screen_config &Sign, std::string const &Folder ) {

    Sign.deserialize( Parser );
    if( Sign.script.empty() ) { return; }
    if( auto path { script_screens::screen_script_path( Folder, Sign.script ) }; path.has_value() ) {
        Sign.script = std::move( *path );
    }
    else {
        ErrorLog( "Bad script sign: \"" + Sign.script + "\" leads outside the game's folder" );
        Sign.script.clear();
    }
}

} // namespace

// McZapkie-250202
// wczytywanie pliku z danymi multimedialnymi (dzwieki)
void TDynamicObject::LoadMMediaFile( std::string const &TypeName, std::string const &ReplacableSkin ) {
	rTypeName = TypeName;
	rReplacableSkin = ReplacableSkin;
    Global.asCurrentDynamicPath = asBaseDir;
    std::string asFileName = asBaseDir + TypeName + ".mmd";
    std::string asAnimName;
    bool Stop_InternalData = false;
    pants = nullptr; // wskaźnik pierwszego obiektu animującego dla pantografów
	wipers = nullptr; // wskaznik pierwszego obiektu animujacego dla wycieraczek
    {
        // preliminary check whether the file exists
        cParser parser( TypeName + ".mmd", cParser::buffer_FILE, asBaseDir );
        if( false == parser.ok() ) {
            ErrorLog( "Failed to load appearance data for vehicle " + MoverParameters->Name );
            return;
        }
    }
    // use #include wrapper to access the appearance data file
    // this allows us to provide the file content with user-defined parameters
    cParser parser(
        "include " + TypeName + ".mmd"
        + " " + asName // (p1)
        + " " + TypeName // (p2)
        + " " + ReplacableSkin // (p3)
        + " end",
        cParser::buffer_TEXT,
        asBaseDir, true, std::vector<std::string>(), true );
	std::string token;
    do {
		token = "";
		parser.getTokens(); parser >> token;

		if( token == "models:" ) {
			// modele i podmodele
            m_materialdata.multi_textures = 0; // czy jest wiele tekstur wymiennych?
			parser.getTokens();
			parser >> asModel;
            replace_slashes( asModel );
            if( asModel.back() == '#' ) // Ra 2015-01: nie podoba mi siê to
            { // model wymaga wielu tekstur wymiennych
                m_materialdata.multi_textures = 1;
                asModel.erase( asModel.length() - 1 );
            }
            // name can contain leading slash, erase it to avoid creation of double slashes when the name is combined with current directory
            erase_leading_slashes( asModel );
            /*
            // never really used, may as well get rid of it
            std::size_t i = asModel.find( ',' );
            if ( i != std::string::npos )
            { // Ra 2015-01: może szukać przecinka w nazwie modelu, a po przecinku była by liczba tekstur?
                if( i < asModel.length() ) {
                    m_materialdata.multi_textures = asModel[ i + 1 ] - '0';
                }
                m_materialdata.multi_textures = std::clamp( m_materialdata.multi_textures, 0, 1 ); // na razie ustawiamy na 1
            }
            */
            asModel = asBaseDir + asModel; // McZapkie 2002-07-20: dynamics maja swoje modele w dynamics/basedir
            Global.asCurrentTexturePath = asBaseDir; // biezaca sciezka do tekstur to dynamic/...
            mdModel = TModelsManager::GetModel(asModel, true);
            if (ReplacableSkin != "none") {
                m_materialdata.assign( ReplacableSkin );
            }
            Global.asCurrentTexturePath = paths::textures; // z powrotem defaultowa sciezka do tekstur
            do {
				token = "";
				parser.getTokens(); parser >> token;

				if( token == "animations:" ) {
					// Ra: ustawienie ilości poszczególnych animacji - musi być jako pierwsze, inaczej ilości będą domyślne
/*
                    if( nullptr == pAnimations )
*/
                    if( true == pAnimations.empty() )
                    { // jeśli nie ma jeszcze tabeli animacji, można odczytać nowe ilości
                        int co = 0;
                        iAnimations = 0;
                        int ile;
                        do
                        { // kolejne liczby to ilość animacj, -1 to znacznik końca
                            ile = -1;
							parser.getTokens( 1, false );
                            parser >> ile; // ilość danego typu animacji
                            if (ile >= 0)
                            {
                                iAnimType[co] = ile; // zapamiętanie
                                iAnimations += ile; // ogólna ilość animacji
                            }
                            else {
                                iAnimType[co] = 0;
                            }
                            ++co;
                        } while ( ile >= 0 && co < ANIM_TYPES ); //-1 to znacznik końca

                        pAnimations.resize( iAnimations );
                        int i, j, k = 0, sm = 0;
                        for (j = 0; j < ANIM_TYPES; ++j) // petla po wszystkich wpisach w animations
                            for (i = 0; i < iAnimType[j]; ++i) // petla iteruje sie tyle razy ile mamy wpisane w animations
                            {
                                if (j == ANIM_PANTS) // zliczamy poprzednie animacje
                                    if (!pants)
                                        if (iAnimType[ANIM_PANTS]) // o ile jakieś pantografy są (a domyślnie są)
                                            pants = &pAnimations[k]; // zapamiętanie na potrzeby wyszukania submodeli
								if (j == ANIM_WIPERS)
									if (!wipers)
										if (iAnimType[ANIM_WIPERS])
											wipers = &pAnimations[k];
                                pAnimations[k].iShift = sm; // przesunięcie do przydzielenia wskaźnika
								sm += pAnimations[k++].TypeSet(j, *MoverParameters); // ustawienie typu animacji i zliczanie tablicowanych submodeli
							}
                        if (sm) // o ile są bardziej złożone animacje
                        {
                            pAnimated = new TSubModel *[sm]; // tabela na animowane submodele
                            for (k = 0; k < iAnimations; ++k)
                                pAnimations[k].smElement = pAnimated + pAnimations[k].iShift; // przydzielenie wskaźnika do tabelki
                        }
                    }
                }

                else if(token == "lowpolyinterior:") {
					// ABu: wnetrze lowpoly
					parser.getTokens();
					parser >> asModel;

                    replace_slashes( asModel );
                    erase_leading_slashes( asModel );
                    asModel = asBaseDir + asModel; // McZapkie-200702 - dynamics maja swoje modele w dynamic/basedir
                    Global.asCurrentTexturePath = asBaseDir; // biezaca sciezka do tekstur to dynamic/...
                    mdLowPolyInt = TModelsManager::GetModel(asModel, true);
                }

                else if(token == "coupleradapter:") {
					// coupling adapter data
					parser.getTokens( 3 );
                    parser
                        >> m_coupleradapter.model
                        >> m_coupleradapter.position.x
                        >> m_coupleradapter.position.y;
                    replace_slashes( m_coupleradapter.model );
                    erase_leading_slashes( m_coupleradapter.model );
                }

                else if(token == "attachments:") {
					// additional 3d models attached to main body
                    // content provided as a series of values together enclosed in "{}"
                    // each value is a name of additional 3d model
                    // value can be optionally set of values enclosed in "[]" in which case one value will be picked randomly
                    // TBD: reconsider something more yaml-compliant and/or ability to define offset and rotation
                    while( (token = deserialize_random_set(parser)) != ""
                        && token != "}" ) {
                        if( token == "{" ) { continue; }
                        replace_slashes( token );
                        Global.asCurrentTexturePath = asBaseDir; // biezaca sciezka do tekstur to dynamic/...
                        auto *attachmentmodel { TModelsManager::GetModel( asBaseDir + token, true ) };
                        if( attachmentmodel != nullptr ) {
                            mdAttachments.emplace_back( attachmentmodel );
                        }
                    }
                }

                else if(token == "loads:") {
					// default load visualization models overrides
                    // content provided as "key: value" pairs together enclosed in "{}"
                    // value can be optionally set of values enclosed in "[]" in which case one value will be picked randomly
                    while( (token = parser.getToken<std::string>()) != ""
                        && token != "}" ) {
                        if( token.back() == ':' ) {
                            auto loadmodel { deserialize_random_set( parser ) };
                            replace_slashes( loadmodel );
                            LoadModelOverrides.emplace( token.erase( token.size() - 1 ), loadmodel );
                        }
                    }
                }

				else if( token == "brakemode:" ) {
                // Ra 15-01: gałka nastawy hamulca
					parser.getTokens();
					parser >> asAnimName;
                    smBrakeMode = GetSubmodelFromName( mdModel, asAnimName );
                    // jeszcze wczytać kąty obrotu dla poszczególnych ustawień
                }

				else if( token == "loadmode:" ) {
                // Ra 15-01: gałka nastawy hamulca
					parser.getTokens();
					parser >> asAnimName;
                    smLoadMode = GetSubmodelFromName( mdModel, asAnimName );
                    // jeszcze wczytać kąty obrotu dla poszczególnych ustawień
                }

                else if (token == "animwheelprefix:") {
					// prefiks kręcących się kół
                    int i, k, m;
					unsigned int j;
					parser.getTokens( 1, false ); parser >> token;
                    for (i = 0; i < iAnimType[ANIM_WHEELS]; ++i) // liczba osi
                    { // McZapkie-050402: wyszukiwanie kol o nazwie str*
                        asAnimName = token + std::to_string(i + 1);
                        pAnimations[i].smAnimated = GetSubmodelFromName( mdModel, asAnimName );
                        if (pAnimations[i].smAnimated)
                        { //++iAnimatedAxles;
                            pAnimations[i].smAnimated->WillBeAnimated(); // wyłączenie optymalizacji transformu
							pAnimations[i].yUpdate = std::bind( &TDynamicObject::UpdateAxle, this, std::placeholders::_1 );
							pAnimations[i].fMaxDist = Global.fDistanceFactor * MoverParameters->WheelDiameter * 200;
							pAnimations[i].fMaxDist *= pAnimations[i].fMaxDist;
                        }
                    }
                    // Ra: ustawianie indeksów osi
                    for (i = 0; i < iAnimType[ANIM_WHEELS]; ++i) // ilość osi (zabezpieczenie przed błędami w CHK)
                        pAnimations[i].dWheelAngle = 1; // domyślnie wskaźnik na napędzające
                    i = 0;
                    j = 1;
                    k = 0;
                    m = 0; // numer osi; kolejny znak; ile osi danego typu; która średnica
                    if (MoverParameters->WheelDiameterL != MoverParameters->WheelDiameter ||
                        MoverParameters->WheelDiameterT != MoverParameters->WheelDiameter)
                    { // obsługa różnych średnic, o ile występują
                        while (i < iAnimType[ANIM_WHEELS] &&
                               j <= MoverParameters->AxleArangement.length())
                        { // wersja ze wskaźnikami jest bardziej elastyczna na nietypowe układy
                            if (k >= 'A' && k <= 'J') // 10 chyba maksimum?
                            {
                                pAnimations[i++].dWheelAngle = 1; // obrót osi napędzających
                                --k; // następna będzie albo taka sama, albo bierzemy kolejny znak
                                m = 2; // następujące toczne będą miały inną średnicę
                            }
                            else if (k >= '1' && k <= '9')
                            {
                                pAnimations[i++].dWheelAngle = m; // obrót osi tocznych
                                --k; // następna będzie albo taka sama, albo bierzemy kolejny znak
                            }
                            else
                                k = MoverParameters->AxleArangement[j++]; // pobranie kolejnego znaku
                        }
                    }
                }
                // else if (str==AnsiString("animrodprefix:")) //prefiks wiazarow dwoch
                // {
                //  str= Parser->GetNextSymbol();
                //  for (int i=1; i<=2; i++)
                //  {//McZapkie-050402: wyszukiwanie max 2 wiazarow o nazwie str*
                //   asAnimName=str+i;
                //   smWiazary[i-1]=mdModel->GetFromName(asAnimName.c_str());
                //   smWiazary[i-1]->WillBeAnimated();
                //  }
                // }

				else if( token == "animpantprefix:" ) {
                 // Ra: pantografy po nowemu mają literki i numerki
                }
                // Pantografy - Winger 160204
				else if( token == "animpantrd1prefix:" ) {
                // prefiks ramion dolnych 1
					parser.getTokens(); parser >> token;
                    float4x4 m; // macierz do wyliczenia pozycji i wektora ruchu pantografu
                    TSubModel *sm;
                    if (pants)
                        for (int i = 0; i < iAnimType[ANIM_PANTS]; i++)
                        { // Winger 160204: wyszukiwanie max 2 patykow o nazwie str*
                            asAnimName = token + std::to_string(i + 1);
                            sm = GetSubmodelFromName( mdModel, asAnimName );
                            pants[i].smElement[0] = sm; // jak NULL, to nie będzie animowany
                            if (sm)
                            { // w EP09 wywalało się tu z powodu NULL
                                sm->WillBeAnimated();
                                sm->ParentMatrix(&m); // pobranie macierzy transformacji
                                // m(3)[1]=m[3][1]+0.054; //w górę o wysokość ślizgu (na razie tak)
                                pants[i].fParamPants->vPos.z = m[3][0]; // przesunięcie w bok (asymetria)
                                pants[i].fParamPants->vPos.y = m[3][1]; // przesunięcie w górę odczytane z modelu
                                if ((sm = pants[i].smElement[0]->ChildGet()) != nullptr)
                                { // jeśli ma potomny, można policzyć długość (odległość potomnego od osi obrotu)
                                    m = float4x4(*sm->GetMatrix()); // wystarczyłby wskaźnik, nie trzeba kopiować
                                    // może trzeba: pobrać macierz dolnego ramienia, wyzerować przesunięcie, przemnożyć przez macierz górnego
                                    pants[i].fParamPants->fHoriz = -fabs(m[3][1]);
                                    pants[i].fParamPants->fLenL1 = hypot(m[3][1], m[3][2]); // po osi OX nie potrzeba
                                    pants[i].fParamPants->fAngleL0 = atan2(fabs(m[3][2]), fabs(m[3][1]));
                                    // if (pants[i].fParamPants->fAngleL0<M_PI_2)
                                    // pants[i].fParamPants->fAngleL0+=M_PI; //gdyby w odwrotną stronę wyszło
                                    // if
                                    // ((pants[i].fParamPants->fAngleL0<0.03)||(pants[i].fParamPants->fAngleL0>0.09))
                                    // //normalnie ok. 0.05
                                    // pants[i].fParamPants->fAngleL0=pants[i].fParamPants->fAngleL;
                                    pants[i].fParamPants->fAngleL = pants[i].fParamPants->fAngleL0; // początkowy kąt dolnego ramienia
                                    if ((sm = sm->ChildGet()) != nullptr)
                                    { // jeśli dalej jest ślizg, można policzyć długość górnego ramienia
                                        m = float4x4(*sm->GetMatrix()); // wystarczyłby wskaźnik,
                                        // nie trzeba kopiować trzeba by uwzględnić macierz dolnego ramienia, żeby uzyskać kąt do poziomu...
                                        pants[i].fParamPants->fHoriz += fabs(m(3)[1]); // różnica długości rzutów ramion na
                                        // płaszczyznę podstawy (jedna dodatnia, druga ujemna)
                                        pants[i].fParamPants->fLenU1 = hypot( m[3][1], m[3][2] ); // po osi OX nie potrzeba
                                        // pants[i].fParamPants->pantu=acos((1.22*cos(pants[i].fParamPants->fAngleL)+0.535)/1.755); //górne ramię
                                        // pants[i].fParamPants->fAngleU0=acos((1.176289*cos(pants[i].fParamPants->fAngleL)+0.54555075)/1.724482197); //górne ramię
                                        pants[i].fParamPants->fAngleU0 = atan2( fabs(m[3][2]), fabs(m[3][1]) ); // początkowy kąt górnego ramienia, odczytany z modelu
                                        // if (pants[i].fParamPants->fAngleU0<M_PI_2)
                                        // pants[i].fParamPants->fAngleU0+=M_PI; //gdyby w odwrotną stronę wyszło
                                        // if (pants[i].fParamPants->fAngleU0<0)
                                        // pants[i].fParamPants->fAngleU0=-pants[i].fParamPants->fAngleU0;
                                        // if
                                        // ((pants[i].fParamPants->fAngleU0<0.00)||(pants[i].fParamPants->fAngleU0>0.09)) //normalnie ok. 0.07
                                        // pants[i].fParamPants->fAngleU0=acos((pants[i].fParamPants->fLenL1*cos(pants[i].fParamPants->fAngleL)+pants[i].fParamPants->fHoriz)/pants[i].fParamPants->fLenU1);
                                        pants[i].fParamPants->fAngleU = pants[i].fParamPants->fAngleU0; // początkowy kąt
                                        // Ra: ze względu na to, że niektóre modele pantografów są zrąbane, ich mierzenie ma obecnie ograniczony sens
                                        sm->ParentMatrix(&m); // pobranie macierzy transformacji pivota ślizgu względem wstawienia pojazdu
                                        float det = Det(m);
                                        if (std::fabs(det - 1.0) < 0.001) // dopuszczamy 1 promil błędu na skalowaniu ślizgu
                                        { // skalowanie jest w normie, można pobrać wymiary z modelu
                                            pants[i].fParamPants->fHeight = sm->MaxY(m); // przeliczenie maksimum wysokości wierzchołków względem macierzy
                                            pants[i].fParamPants->fHeight -= m[3][1]; // odjęcie wysokości pivota ślizgu
                                            pants[i].fParamPants->vPos.x = m[3][2]; // przy okazji odczytać z modelu pozycję w długości
                                            // ErrorLog("Model OK: "+asModel+",
                                            // height="+pants[i].fParamPants->fHeight);
                                            // ErrorLog("Model OK: "+asModel+",
                                            // pos.x="+pants[i].fParamPants->vPos.x);
                                        }
                                        else
                                        { // gdy ktoś przesadził ze skalowaniem
                                            pants[i].fParamPants->fHeight = 0.0; // niech będzie odczyt z pantfactors:
                                            ErrorLog(
                                                "Bad model: " + asModel + ", scale of " + sm->pName + " is " + std::to_string(100.0 * det) + "%",
                                                logtype::model );
                                        }
                                    }
                                }
                            }
                            else {
                                // brak ramienia
                                pants[ i ].fParamPants->fHeight = 0.0; // niech będzie odczyt z pantfactors:
                                ErrorLog( "Bad model: " + asFileName + " - missed submodel " + asAnimName, logtype::model );
                            }
                        }
                }

				else if( token == "animpantrd2prefix:" ) {
                // prefiks ramion dolnych 2
					parser.getTokens(); parser >> token;
                    float4x4 m; // macierz do wyliczenia pozycji i wektora ruchu pantografu
                    TSubModel *sm;
					if( pants ) {
						for( int i = 0; i < iAnimType[ ANIM_PANTS ]; i++ ) {
                            // Winger 160204: wyszukiwanie max 2 patykow o nazwie str*
							asAnimName = token + std::to_string( i + 1 );
                            sm = GetSubmodelFromName( mdModel, asAnimName );
							pants[ i ].smElement[ 1 ] = sm; // jak NULL, to nie będzie animowany
							if( sm ) { // w EP09 wywalało się tu z powodu NULL
                                sm->WillBeAnimated();
								if( pants[ i ].fParamPants->vPos.y == 0.0 ) {
                                    // jeśli pierwsze ramię nie ustawiło tej wartości, próbować drugim
                                    //!!!! docelowo zrobić niezależną animację ramion z każdej strony
                                    m = float4x4(
                                        *sm->GetMatrix()); // skopiowanie, bo będziemy mnożyć
									m( 3 )[ 1 ] =
										m[ 3 ][ 1 ] + 0.054; // w górę o wysokość ślizgu (na razie tak)
									while( sm->Parent ) {
										if( sm->Parent->GetMatrix() )
                                            m = *sm->Parent->GetMatrix() * m;
                                        sm = sm->Parent;
                                    }
									pants[ i ].fParamPants->vPos.z = m[3][0]; // przesunięcie w bok (asymetria)
									pants[ i ].fParamPants->vPos.y = m[3][1]; // przesunięcie w górę odczytane z modelu
                                }
                            }
                            else
								ErrorLog( "Bad model: " + asFileName + " - missed submodel " + asAnimName, logtype::model ); // brak ramienia
						}
                    }
                }

				else if( token == "animpantrg1prefix:" ) {
                 // prefiks ramion górnych 1
					parser.getTokens(); parser >> token;
                    if( pants ) {
                        for( int i = 0; i < iAnimType[ ANIM_PANTS ]; i++ ) {
                            // Winger 160204: wyszukiwanie max 2 patykow o nazwie str*
                            asAnimName = token + std::to_string( i + 1 );
                            pants[ i ].smElement[ 2 ] = GetSubmodelFromName( mdModel, asAnimName );
                            if( pants[ i ].smElement[ 2 ] ) {
                                pants[ i ].smElement[ 2 ]->WillBeAnimated();
                            }
                            else {
                                ErrorLog( "Bad model: " + asFileName + " - missed submodel " + asAnimName, logtype::model ); // brak ramienia
                            }
                        }
                    }
                }

				else if( token == "animpantrg2prefix:" ) {
                 // prefiks ramion górnych 2
                    parser.getTokens(); parser >> token;
                    if( pants ) {
                        for( int i = 0; i < iAnimType[ ANIM_PANTS ]; i++ ) {
                            // Winger 160204: wyszukiwanie max 2 patykow o nazwie str*
                            asAnimName = token + std::to_string( i + 1 );
                            pants[ i ].smElement[ 3 ] = GetSubmodelFromName( mdModel, asAnimName );
                            if( pants[ i ].smElement[ 3 ] ) {
                                pants[ i ].smElement[ 3 ]->WillBeAnimated();
                            }
                            else {
                                ErrorLog( "Bad model: " + asFileName + " - missed submodel " + asAnimName, logtype::model ); // brak ramienia
                            }
                        }
                    }
                }

				else if( token == "animpantslprefix:" ) {
                 // prefiks ślizgaczy
					parser.getTokens(); parser >> token;
                    if( pants ) {
                        for( int i = 0; i < iAnimType[ ANIM_PANTS ]; i++ ) {
                            // Winger 160204: wyszukiwanie max 2 patykow o nazwie str*
                            asAnimName = token + std::to_string( i + 1 );
                            pants[ i ].smElement[ 4 ] = GetSubmodelFromName( mdModel, asAnimName );
                            if( pants[ i ].smElement[ 4 ] ) {
                                pants[ i ].smElement[ 4 ]->WillBeAnimated();
                                pants[ i ].yUpdate = std::bind( &TDynamicObject::UpdatePant, this, std::placeholders::_1 );
                                pants[ i ].fMaxDist = 300 * 300; // nie podnosić w większej odległości
                                pants[ i ].iNumber = i;
                            }
                            else {
                                ErrorLog( "Bad model: " + asFileName + " - missed submodel " + asAnimName, logtype::model ); // brak ramienia
                            }
                        }
                    }
                }

				else if( token == "pantfactors:" ) {
                 // Winger 010304:
                    // parametry pantografow
					double pant1x, pant2x, pant1h, pant2h;
					parser.getTokens( 4, false );
					parser
						>> pant1x
						>> pant2x
						>> pant1h // wysokość pierwszego ślizgu
						>> pant2h;// wysokość drugiego ślizgu
					if( pant1h > 0.5 ) {
                        pant1h = pant2h; // tu może być zbyt duża wartość
					}
                    if (pant1x < 0 &&
                        pant2x > 0) // pierwsza powinna być dodatnia, a druga ujemna
                    {
                        pant1x = -pant1x;
                        pant2x = -pant2x;
                    }
                    if( pants ) {
                        for( int i = 0; i < iAnimType[ ANIM_PANTS ]; ++i ) {
                            // przepisanie współczynników do pantografów (na razie nie będzie lepiej)
                            auto &pantograph { *pants[i].fParamPants };

                            pantograph.fAngleL = pantograph.fAngleL0; // początkowy kąt dolnego ramienia
                            pantograph.fAngleU = pantograph.fAngleU0; // początkowy kąt
                            // pants[i].fParamPants->PantWys=1.22*sin(pants[i].fParamPants->fAngleL)+1.755*sin(pants[i].fParamPants->fAngleU);
                            // //wysokość początkowa
                            // pants[i].fParamPants->PantWys=1.176289*sin(pants[i].fParamPants->fAngleL)+1.724482197*sin(pants[i].fParamPants->fAngleU);
                            // //wysokość początkowa
                            if( pantograph.fHeight == 0.0 ) // gdy jest nieprawdopodobna wartość (np. nie znaleziony ślizg)
                            { // gdy pomiary modelu nie udały się, odczyt podanych parametrów z MMD
                                pantograph.vPos.x =
                                    i & 1 ?
                                        pant2x :
                                        pant1x;
                                pantograph.fHeight =
                                    i & 1 ?
                                        pant2h :
                                        pant1h; // wysokość ślizgu jest zapisana w MMD
                            }
                            pantograph.PantWys =
                                pantograph.fLenL1 * sin( pantograph.fAngleL ) +
                                pantograph.fLenU1 * sin( pantograph.fAngleU ) +
                                pantograph.fHeight; // wysokość początkowa
                            // pants[i].fParamPants->vPos.y=panty-panth-pants[i].fParamPants->PantWys; //np. 4.429-0.097=4.332=~4.335
                            if( pantograph.vPos.y == 0.0 ) {
                                // crude fallback, place the pantograph(s) atop of the vehicle-sized box
                                pantograph.vPos.y = MoverParameters->Dim.H - pantograph.fHeight - pantograph.PantWys;
                            }
                            // pants[i].fParamPants->vPos.z=0; //niezerowe dla pantografów asymetrycznych
                            pantograph.PantTraction = pantograph.PantWys;
                            // połowa szerokości ślizgu; jest w "Power: CSW="
                            pantograph.fWidth = 0.5 * MoverParameters->EnginePowerSource.CollectorParameters.CSW;

                            // create sound emitters for the pantograph
                            m_pantographsounds.emplace_back();
                        }
                    }
                }

                else if (token == "animpistonprefix:") {
					// prefiks tłoczysk - na razie uzywamy modeli pantografów
					parser.getTokens(1, false); parser >> token;
					for( int i = 1; i <= 2; ++i )
                    {
                        // asAnimName=str+i;
                        // smPatykird1[i-1]=mdModel->GetFromName(asAnimName.c_str());
                        // smPatykird1[i-1]->WillBeAnimated();
                    }
                }

				else if( token == "animconrodprefix:" ) {
                 // prefiks korbowodów - na razie używamy modeli pantografów
					parser.getTokens(); parser >> token;
					for( int i = 1; i <= 2; i++ )
                    {
                        // asAnimName=str+i;
                        // smPatykirg1[i-1]=mdModel->GetFromName(asAnimName.c_str());
                        // smPatykirg1[i-1]->WillBeAnimated();
                    }
                }

				else if( token == "pistonfactors:" ) {
                 // Ra: parametry
                    // silnika parowego
                    // (tłoka)
                    /* //Ra: tymczasowo wyłączone ze względu na porządkowanie animacji
                       pantografów
                             pant1x=Parser->GetNextSymbol().ToDouble(); //kąt przesunięcia
                       dla
                       pierwszego tłoka
                             pant2x=Parser->GetNextSymbol().ToDouble(); //kąt przesunięcia
                       dla
                       drugiego tłoka
                             panty=Parser->GetNextSymbol().ToDouble(); //długość korby (r)
                             panth=Parser->GetNextSymbol().ToDouble(); //długoś korbowodu
                       (k)
                    */
                    MoverParameters->EnginePowerSource.PowerType =
                        TPowerType::SteamPower; // Ra: po chamsku, ale z CHK nie działa
                }

				else if( token == "animreturnprefix:" ) {
                 // prefiks drążka mimośrodowego - na razie używamy modeli pantografów
					parser.getTokens(1, false); parser >> token;
					for( int i = 1; i <= 2; i++ )
                    {
                        // asAnimName=str+i;
                        // smPatykird2[i-1]=mdModel->GetFromName(asAnimName.c_str());
                        // smPatykird2[i-1]->WillBeAnimated();
                    }
                }

                else if (token == "animexplinkprefix:"){ // animreturnprefix:
                 // prefiks jarzma - na razie używamy modeli pantografów
					parser.getTokens(1, false); parser >> token;
					for( int i = 1; i <= 2; i++ )
                    {
                        // asAnimName=str+i;
                        // smPatykirg2[i-1]=mdModel->GetFromName(asAnimName.c_str());
                        // smPatykirg2[i-1]->WillBeAnimated();
                    }
                }

				else if( token == "animpendulumprefix:" ) {
                 // prefiks wahaczy
					parser.getTokens(); parser >> token;
                    asAnimName = "";
                    for( int i = 1; i <= 4; ++i ) {
                        // McZapkie-050402: wyszukiwanie max 4 wahaczy o nazwie str*
                        asAnimName = token + std::to_string( i );
                        smWahacze[ i - 1 ] = GetSubmodelFromName( mdModel, asAnimName );
                        if( smWahacze[ i - 1 ] ) {
                            smWahacze[ i - 1 ]->WillBeAnimated();
                        }
                    }
					parser.getTokens(); parser >> token;
					if( token == "pendulumamplitude:" ) {
						parser.getTokens( 1, false );
						parser >> fWahaczeAmp;
					}
                }

                else if( token == "animslottedlever:" ) {
                    // lever with a slot, turned by a pin on the coupling rod (e.g. speedometer drive)
                    // parameters: submodel name, number of axle (from animwheelprefix) driving the coupling rod, crank radius [m],
                    // pin position relative to the lever axis [m] and crank angle [deg] for wheel angle 0, in local (Y, Z) plane of the lever
                    // angle is measured from Y axis towards Z axis, i.e. in the direction of rotation around X axis
                    slottedlever_data lever;
                    int axle { 1 };
                    double crankradius { 0.0 };
                    double crankangle { 0.0 };
                    parser.getTokens( 6, false );
                    parser
                        >> asAnimName
                        >> axle
                        >> crankradius
                        >> lever.pin.x
                        >> lever.pin.y
                        >> crankangle;
                    lever.submodel = GetSubmodelFromName( mdModel, asAnimName );
                    if( lever.submodel != nullptr ) {
                        lever.submodel->WillBeAnimated();
                        lever.axle = std::max( 0, axle - 1 );
                        lever.crank = crankradius * glm::dvec2( std::cos( glm::radians( crankangle ) ), std::sin( glm::radians( crankangle ) ) );
                        m_slottedlevers.emplace_back( lever );
                    }
                    else {
                        ErrorLog( "Bad model: " + asFileName + " - missed submodel " + asAnimName, logtype::model );
                    }
                }

				else if( token == "animdoorprefix:" ) {
                    // nazwa animowanych drzwi
					int i, j;
					parser.getTokens(1, false); parser >> token;
                    for (i = 0, j = 0; i < ANIM_DOORS; ++i)
                        j += iAnimType[i]; // zliczanie wcześniejszych animacji
                    for (i = 0; i < iAnimType[ANIM_DOORS]; ++i) // liczba drzwi
                    { // NBMX wrzesien 2003: wyszukiwanie drzwi o nazwie str*
                      // ustalenie submodelu
                        asAnimName = token + std::to_string(i + 1);
                        pAnimations[ i + j ].smAnimated = GetSubmodelFromName( mdModel, asAnimName );
                        if (pAnimations[i + j].smAnimated)
                        { //++iAnimatedDoors;
                            pAnimations[i + j].smAnimated->WillBeAnimated(); // wyłączenie optymalizacji transformu
                            pAnimations[i + j].iNumber = i + 1; // parzyste działają inaczej niż nieparzyste
                            bind_door_animation( pAnimations[ i + j ] ); // od razu zapinamy potrzebny typ animacji
                            pAnimations[i + j].fMaxDist = 300 * 300; // drzwi to z daleka widać
/*
                            // NOTE: no longer used
                            pAnimations[i + j].fSpeed = Random(150); // oryginalny koncept z DoorSpeedFactor
                            pAnimations[i + j].fSpeed = (pAnimations[i + j].fSpeed + 100) / 100;
*/
                        }
                    }
                }

                else if( token == "animdoor:" ) {
                    // own kind of animation of one door wing, in place of the one of the .fiz (eu07/maszyna#70):
                    // animdoor: <wing number> { type: <Shift|Rotate|Fold|Plug> range: <shift or angle> rangeout: <plug shift out> }
                    // the keys left out keep the values of the .fiz. the wing still follows the door of its side
                    door_wings::wing_spec spec {
                        0,
                        MoverParameters->Doors.type,
                        MoverParameters->Doors.range,
                        MoverParameters->Doors.range_out };
                    auto error { door_wings::read_spec( parser, spec ) };
                    if( error.empty()
                     && ( spec.number < 1 || spec.number > iAnimType[ ANIM_DOORS ] ) ) {
                        error = "no door wing " + std::to_string( spec.number ) + " among the door animations";
                    }
                    if( false == error.empty() ) {
                        ErrorLog( "Bad model: " + asFileName + " - animdoor: " + error, logtype::model );
                    }
                    else {
                        door_wings::store( m_doorwings, spec );
                        // a wing animdoorprefix has found already gets the new kind of animation now; the ones it
                        // finds later get it then
                        auto const index { static_cast<std::size_t>( std::accumulate( std::begin( iAnimType ), std::begin( iAnimType ) + ANIM_DOORS, 0 ) + spec.number - 1 ) };
                        if( index < pAnimations.size() && pAnimations[ index ].yUpdate ) {
                            bind_door_animation( pAnimations[ index ] );
                        }
                    }
                }

                else if( token == "animstepprefix:" ) {
                    // animated doorstep submodel name prefix
					int i, j;
					parser.getTokens(1, false); parser >> token;
                    for (i = 0, j = 0; i < ANIM_DOORSTEPS; ++i)
                        j += iAnimType[i]; // zliczanie wcześniejszych animacji
                    for (i = 0; i < iAnimType[ANIM_DOORSTEPS]; ++i) // liczba drzwi
                    { // NBMX wrzesien 2003: wyszukiwanie drzwi o nazwie str*
                      // ustalenie submodelu
                        asAnimName = token + std::to_string(i + 1);
                        pAnimations[i + j].smAnimated = GetSubmodelFromName( mdModel, asAnimName );
                        if (pAnimations[i + j].smAnimated)
                        { //++iAnimatedDoors;
                            pAnimations[i + j].smAnimated->WillBeAnimated(); // wyłączenie optymalizacji transformu
                            switch( MoverParameters->Doors.step_type )
                            { // od razu zapinamy potrzebny typ animacji
                            case 1: // shift
								pAnimations[ i + j ].yUpdate = std::bind( &TDynamicObject::UpdatePlatformTranslate, this, std::placeholders::_1 );
                                break;
                            case 2: // rotate
								pAnimations[ i + j ].yUpdate = std::bind( &TDynamicObject::UpdatePlatformRotate, this, std::placeholders::_1 );
                                break;
							default:
								break;
							}
                            pAnimations[i + j].iNumber = i + 1; // parzyste działają inaczej niż nieparzyste
                            pAnimations[i + j].fMaxDist = 150 * 150; // drzwi to z daleka widać
/*
                            // NOTE: no longer used
                            pAnimations[i + j].fSpeed = Random(150); // oryginalny koncept z DoorSpeedFactor
                            pAnimations[i + j].fSpeed = (pAnimations[i + j].fSpeed + 100) / 100;
*/
                        }
                    }
                }

                else if( token == "animmirrorprefix:" ) {
                    // animated mirror submodel name prefix
                    int i, j;
                    parser.getTokens( 1, false ); parser >> token;
                    for( i = 0, j = 0; i < ANIM_MIRRORS; ++i )
                        j += iAnimType[ i ]; // zliczanie wcześniejszych animacji
                    for( i = 0; i < iAnimType[ ANIM_MIRRORS ]; ++i ) // liczba drzwi
                    { // NBMX wrzesien 2003: wyszukiwanie drzwi o nazwie str*
                      // ustalenie submodelu
                        asAnimName = token + std::to_string( i + 1 );
                        pAnimations[ i + j ].smAnimated = GetSubmodelFromName( mdModel, asAnimName );
                        if( pAnimations[ i + j ].smAnimated ) {
                            pAnimations[ i + j ].smAnimated->WillBeAnimated(); // wyłączenie optymalizacji transformu
                            // od razu zapinamy potrzebny typ animacji
                            auto const offset { pAnimations[ i + j ].smAnimated->offset() };
                            pAnimations[ i + j ].yUpdate = std::bind( &TDynamicObject::UpdateMirror, this, std::placeholders::_1 );
                            // we don't expect more than 2-4 mirrors, so it should be safe to store submodel location (front/rear) in the higher bits
                            // parzyste działają inaczej niż nieparzyste
                            pAnimations[ i + j ].iNumber =
                                ( ( pAnimations[ i + j ].smAnimated->offset().z > 0 ?
                                    end::front :
                                    end::rear ) << 4 )
                                + i;
                            pAnimations[ i + j ].fMaxDist = 150 * 150; // drzwi to z daleka widać
/*
                            // NOTE: no longer used
                            pAnimations[i + j].fSpeed = Random(150); // oryginalny koncept z DoorSpeedFactor
                            pAnimations[i + j].fSpeed = (pAnimations[i + j].fSpeed + 100) / 100;
*/
                        }
                    }
                }

                else if (token == "animwiperprefix:")
                {
					parser.getTokens(1, false);
					parser >> token;
					TSubModel *sm;
                    if (wipers)
                    {

						for (int i = 0; i < iAnimType[ANIM_WIPERS]; i++) // zebranie wszystkich submodeli wycieraczek
						{
							dWiperPos.emplace_back(0.0); // dodajemy na koniec zeby sie miejsce tam zrobilo i nie bylo invalid addressow potem

							asAnimName = token + std::to_string(i + 1);
                            // element wycieraczki nr 1
							sm = GetSubmodelFromName(mdModel, asAnimName + "_p1");
							wipers[i].smElement[0] = sm;
							if (sm)
							{
								wipers[i].smElement[0]->WillBeAnimated();
								// auto const offset{wipers[i].smElement[0]->offset()};
							}

                            // element wycieraczki nr 2
							sm = GetSubmodelFromName(mdModel, asAnimName + "_p2");
							wipers[i].smElement[1] = sm;
							if (sm)
							{
								wipers[i].smElement[1]->WillBeAnimated();
								// auto const offset{wipers[i].smElement[0]->offset()};
							}

                            // element wycieraczki nr 3
							sm = GetSubmodelFromName(mdModel, asAnimName + "_p3");
							wipers[i].smElement[2] = sm;
							if (sm)
							{
								wipers[i].smElement[2]->WillBeAnimated();
								// auto const offset{wipers[i].smElement[0]->offset()};
							}
							wipers[i].yUpdate = std::bind(&TDynamicObject::UpdateWiper, this, std::placeholders::_1);
							wipers[i].fMaxDist = 150 * 150;
							wipers[i].iNumber = i;
						}
                    }
                }

			} while( token != ""
	              && token != "endmodels" );

            if( false == MoverParameters->LoadAttributes.empty() ) {
                // Ra: tu wczytywanie modelu ładunku jest w porządku

                // bieżąca ścieżka do tekstur to dynamic/...
                Global.asCurrentTexturePath = asBaseDir;

                mdLoad = LoadMMediaFile_mdload( MoverParameters->LoadType.name );

                // z powrotem defaultowa sciezka do tekstur
                Global.asCurrentTexturePath = std::string( paths::textures );
            }

        } // models

		else if( token == "sounds:" ) {
			// dzwieki
			do {
				token = "";
				parser.getTokens(); parser >> token;
				if( token == "wheel_clatter:" ){
					// polozenia osi w/m srodka pojazdu
                    double dSDist;
					parser.getTokens( 1, false );
					parser >> dSDist;
                    while( (token = parser.getToken<std::string>()) != ""
                          && token != "end" ) {
                        // add another axle entry to the list
                        axle_sounds axle {
                            0,
                            std::atof( token.c_str() ) * -1.0, // for axle locations negative value means ahead of centre but vehicle faces +Z in 'its' space
                            { sound_placement::external, static_cast<float>( dSDist ) } };
                        axle.clatter.deserialize( parser, sound_type::single );
                        axle.clatter.owner( this );
                        axle.clatter.offset( { 0, 0, axle.offset } );
                        m_axlesounds.emplace_back( axle );
                    }
                    // arrange the axles in case they're listed out of order
                    std::sort(
                        std::begin( m_axlesounds ), std::end( m_axlesounds ),
                        []( axle_sounds const &Left, axle_sounds const &Right ) {
                            return Left.offset > Right.offset; } );
                }

				else if( token == "engine:"
					  && MoverParameters->Power > 0 ) {
					// plik z dzwiekiem silnika, mnozniki i ofsety amp. i czest.
                    m_powertrainsounds.engine.deserialize( parser, sound_type::single, sound_parameters::range | sound_parameters::amplitude | sound_parameters::frequency );
                    m_powertrainsounds.engine.owner( this );

                    auto const amplitudedivisor = static_cast<float>( MoverParameters->EngineType == TEngineType::DieselEngine   ? 1 :
					                                                 MoverParameters->EngineType == TEngineType::DieselElectric ? 1 :
					                                                                                                              MoverParameters->nmax * 60 + MoverParameters->Power * 3 );
                    m_powertrainsounds.engine.m_amplitudefactor /= amplitudedivisor;

				} else if (token == "fakeengine:") {
					m_powertrainsounds.fake_engine.deserialize(parser, sound_type::single);
					m_powertrainsounds.fake_engine.owner(this);
				}

                else if( token == "dieselinc:" ) {
                    // dzwiek przy wlazeniu na obroty woodwarda
                    m_powertrainsounds.engine_revving.deserialize( parser, sound_type::single, sound_parameters::range );
                    m_powertrainsounds.engine_revving.owner( this );
                }

                else if( token == "oilpump:" ) {
                    m_powertrainsounds.oil_pump.deserialize( parser, sound_type::single );
                    m_powertrainsounds.oil_pump.owner( this );
                }

                else if( token == "fuelpump:" ) {
                    m_powertrainsounds.fuel_pump.deserialize( parser, sound_type::single );
                    m_powertrainsounds.fuel_pump.owner( this );
                }

                else if( token == "waterpump:" ) {
                    m_powertrainsounds.water_pump.deserialize( parser, sound_type::single );
                    m_powertrainsounds.water_pump.owner( this );
                }

                else if( token == "waterheater:" ) {
                    m_powertrainsounds.water_heater.deserialize( parser, sound_type::single );
                    m_powertrainsounds.water_heater.owner( this );
                }

                else if( token == "tractionmotor:" && MoverParameters->Power > 0 ) {
                    // plik z dzwiekiem silnika, mnozniki i ofsety amp. i czest.
                    sound_source motortemplate { sound_placement::external };
                    motortemplate.deserialize( parser, sound_type::single, sound_parameters::range | sound_parameters::amplitude | sound_parameters::frequency );
                    auto const amplitudedivisor { static_cast<float>( MoverParameters->nmax * 60 + MoverParameters->Power * 3 ) };
                    motortemplate.m_amplitudefactor /= amplitudedivisor;
                    motortemplate.owner( this );

                    auto &motors { m_powertrainsounds.motors };

                    if( true == motors.empty() ) {
                        // fallback for cases without specified motor locations, convert sound template to a single sound source
                        motors.emplace_back( motortemplate );
                    }
                    else {
                        // apply configuration to all defined motors
                        for( auto &motor : motors ) {
                            // combine potential x- and y-axis offsets of the sound template with z-axis offsets of individual motors
                            auto motoroffset { motortemplate.offset() };
                            motoroffset.z = motor.offset().z;
                            motor = motortemplate;
                            motor.offset( motoroffset );
                            // apply randomized playback start offset for each instance, to reduce potential reverb with identical nearby sources
                            motor.start( LocalRandom( 0.0, 1.0 ) );
                        }
                    }
                }

                else if( token == "tractionacmotor:" && MoverParameters->Power > 0 ) {
                    // plik z dzwiekiem silnika, mnozniki i ofsety amp. i czest.
                    sound_source motortemplate { sound_placement::external };
                    motortemplate.deserialize( parser, sound_type::single, sound_parameters::range | sound_parameters::amplitude | sound_parameters::frequency );
                    motortemplate.owner( this );

                    auto &motors { m_powertrainsounds.acmotors };

                    if( true == motors.empty() ) {
                        // fallback for cases without specified motor locations, convert sound template to a single sound source
                        motors.emplace_back( motortemplate );
                    }
                    else {
                        // apply configuration to all defined motors
                        for( auto &motor : motors ) {
                            // combine potential x- and y-axis offsets of the sound template with z-axis offsets of individual motors
                            auto motoroffset { motortemplate.offset() };
                            motoroffset.z = motor.offset().z;
                            motor = motortemplate;
                            motor.offset( motoroffset );
                            // apply randomized playback start offset for each instance, to reduce potential reverb with identical nearby sources
                            motor.start( LocalRandom( 0.0, 1.0 ) );
                        }
                    }
                }

                else if( token == "motorblower:" ) {

                    sound_source blowertemplate { sound_placement::engine };
                    blowertemplate.deserialize( parser, sound_type::single, sound_parameters::range | sound_parameters::amplitude | sound_parameters::frequency );
                    blowertemplate.owner( this );

                    auto const amplitudedivisor { static_cast<float>(
                        MoverParameters->MotorBlowers[ end::front ].speed > 0 ? MoverParameters->MotorBlowers[ end::front ].speed * MoverParameters->nmax * 60 + MoverParameters->Power * 3 :
                        blowertemplate.has_bookends() ? 1 : // NOTE: for motorblowers with fixed speed if the sound has defined bookends we skip revolutions-based part of frequency/volume adjustments
                        MoverParameters->MotorBlowers[ end::front ].speed * -1 ) };
                    blowertemplate.m_amplitudefactor /= amplitudedivisor;
                    blowertemplate.m_frequencyfactor /= amplitudedivisor;

                    if( true == m_powertrainsounds.motors.empty() ) {
                        // fallback for cases without specified motor locations, convert sound template to a single sound source
                        m_powertrainsounds.motorblowers.emplace_back( blowertemplate );
                    }
                    else {
                        // apply configuration to all defined motor blowers
                        for( auto &blower : m_powertrainsounds.motorblowers ) {
                            // combine potential x- and y-axis offsets of the sound template with z-axis offsets of individual blowers
                            auto bloweroffset { blowertemplate.offset() };
                            bloweroffset.z = blower.offset().z;
                            blower = blowertemplate;
                            blower.offset( bloweroffset );
                            // apply randomized playback start offset for each instance, to reduce potential reverb with identical nearby sources
                            blower.start( LocalRandom( 0.0, 1.0 ) );
                        }
                    }
                }

				else if( token == "inverter:" ) {
					// plik z dzwiekiem wentylatora, mnozniki i ofsety amp. i czest.
                    m_powertrainsounds.inverter.deserialize( parser, sound_type::single, sound_parameters::range | sound_parameters::amplitude | sound_parameters::frequency );
                    m_powertrainsounds.inverter.owner( this );
				}

                else if( token == "ventilator:" ) {
					// plik z dzwiekiem wentylatora, mnozniki i ofsety amp. i czest.
                    m_powertrainsounds.rsWentylator.deserialize( parser, sound_type::single, sound_parameters::range | sound_parameters::amplitude | sound_parameters::frequency );
                    m_powertrainsounds.rsWentylator.owner( this );

                    m_powertrainsounds.rsWentylator.m_amplitudefactor /= MoverParameters->RVentnmax;
                    m_powertrainsounds.rsWentylator.m_frequencyfactor /= MoverParameters->RVentnmax;
				}

                else if( token == "radiatorfan1:" ) {
                    // primary circuit radiator fan
                    m_powertrainsounds.radiator_fan.deserialize( parser, sound_type::single );
                    m_powertrainsounds.radiator_fan.owner( this );
                }

                else if( token == "radiatorfan2" ) {
                    // auxiliary circuit radiator fan
                    m_powertrainsounds.radiator_fan_aux.deserialize( parser, sound_type::single );
                    m_powertrainsounds.radiator_fan_aux.owner( this );
                }

				else if( token == "transmission:" ) {
					// plik z dzwiekiem, mnozniki i ofsety amp. i czest.
                    // NOTE, fixed default parameters, legacy system leftover
                    auto &sound { m_powertrainsounds.transmission };
                    sound.m_amplitudefactor = 0.029;
                    sound.m_amplitudeoffset = 0.1;
                    sound.m_frequencyfactor = 0.005;
                    sound.m_frequencyoffset = 1.0;

                    sound.deserialize( parser, sound_type::single, sound_parameters::range );
                    sound.owner( this );
                }

                else if( token == "brakesound:" ) {
                    // hamowanie zwykle:
                    rsBrake.deserialize( parser, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency );
                    rsBrake.owner( this );
                    // NOTE: can't pre-calculate amplitude normalization based on max brake force, as this varies depending on vehicle speed
                    rsBrake.m_frequencyfactor /= 1 + MoverParameters->Vmax;
                }

				else if( token == "brake:"  ) {
					// plik z piskiem hamulca,  mnozniki i ofsety amplitudy.
                    rsPisk.deserialize( parser, sound_type::single, sound_parameters::range | sound_parameters::amplitude );
                    rsPisk.owner( this );

                    if( rsPisk.m_amplitudefactor > 10.f ) {
                        // HACK: convert old style activation point threshold to the new, regular amplitude adjustment system
                        rsPisk.m_amplitudefactor = 1.f;
                        rsPisk.m_amplitudeoffset = 0.f;
                    }
                    rsPisk.m_amplitudeoffset *= ( 105.f - Random( 10.f ) ) / 100.f;
                }

                else if( token == "brakecylinderinc:" ) {
                    // brake cylinder pressure increase sounds
                    m_brakecylinderpistonadvance.deserialize( parser, sound_type::single );
                    m_brakecylinderpistonadvance.owner( this );
                }

                else if( token == "brakecylinderdec:" ) {
                    // brake cylinder pressure decrease sounds
                    m_brakecylinderpistonrecede.deserialize( parser, sound_type::single );
                    m_brakecylinderpistonrecede.owner( this );
                }

				else if (token == "epbrakeinc:") {
				// brake cylinder pressure increase sounds
					m_epbrakepressureincrease.deserialize(parser, sound_type::single);
					m_epbrakepressureincrease.owner(this);
				}

				else if (token == "epbrakedec:") {
				// brake cylinder pressure decrease sounds
					m_epbrakepressuredecrease.deserialize(parser, sound_type::single);
					m_epbrakepressuredecrease.owner(this);
				}

                else if( token == "emergencybrake:" ) {
					// emergency brake sound
                    m_emergencybrake.deserialize( parser, sound_type::single );
                    m_emergencybrake.owner( this );
                }

				else if( token == "brakeacc:" ) {
					// plik z przyspieszaczem (upust po zlapaniu hamowania)
                    sBrakeAcc.deserialize( parser, sound_type::single );
                    sBrakeAcc.owner( this );

                    bBrakeAcc = true;
                }

				else if( token == "unbrake:" ) {
					// plik z piskiem hamulca, mnozniki i ofsety amplitudy.
					rsUnbrake.m_amplitudefactor = 20.0f;
                    rsUnbrake.deserialize( parser, sound_type::single, sound_parameters::range );
                    rsUnbrake.owner( this );
                }
                // spring brake sounds
                else if( token == "springbrake:" ) {
                    m_springbrakesounds.activate.deserialize( parser, sound_type::single );
                    m_springbrakesounds.activate.owner( this );
                }
                else if( token == "springbrakeoff:" ) {
                    m_springbrakesounds.release.deserialize( parser, sound_type::single );
                    m_springbrakesounds.release.owner( this );
                }

				else if( token == "derail:"  ) {
					// dzwiek przy wykolejeniu
                    rsDerailment.deserialize( parser, sound_type::single, sound_parameters::range );
                    rsDerailment.owner( this );
                }

				else if( token == "curve:" ) {
                    rscurve.deserialize( parser, sound_type::single, sound_parameters::range );
                    rscurve.owner( this );
                }

				else if( token == "horn1:" ) {
					// pliki z trabieniem
                    sHorn1.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sHorn1.owner( this );
                }

				else if( token == "horn2:" ) {
					// pliki z trabieniem wysokoton.
                    sHorn2.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sHorn2.owner( this );
                    // TBD, TODO: move horn selection to ai config file
                    if( iHornWarning ) {
                        iHornWarning = 2; // numer syreny do użycia po otrzymaniu sygnału do jazdy
                    }
				}

                else if( token == "horn3:" ) {
                    sHorn3.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sHorn3.owner( this );
                }

				else if( token == "pantographup:" ) {
					// pliki dzwiekow pantografow
                    sound_source pantographup { sound_placement::external };
                    pantographup.deserialize( parser, sound_type::single );
                    pantographup.owner( this );
                    for( auto &pantograph : m_pantographsounds ) {
                        pantograph.sPantUp = pantographup;
                    }
                }

				else if( token == "pantographdown:" ) {
					// pliki dzwiekow pantografow
                    sound_source pantographdown { sound_placement::external };
                    pantographdown.deserialize( parser, sound_type::single );
                    pantographdown.owner( this );
                    for( auto &pantograph : m_pantographsounds ) {
                        pantograph.sPantDown = pantographdown;
                    }
                }

				else if( token == "compressor:" ) {
					// pliki ze sprezarka
                    sCompressor.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sCompressor.owner( this );
                }

                else if( token == "compressoridle:" ) {
					// pliki ze sprezarka
                    sCompressorIdle.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sCompressorIdle.owner( this );
                }

				else if( token == "converter:" ) {
					// pliki z przetwornica
                    sConverter.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sConverter.owner( this );
                }

                // Dzwiek wentylatora rezystora hamowania
				else if (token == "brakingresistorventilator:")
				{
					sBRVent.deserialize(parser, sound_type::multipart, sound_parameters::range);
					sBRVent.owner(this);
				}

                else if( token == "heater:" ) {
                    // train heating device
                    sHeater.deserialize( parser, sound_type::single );
                    sHeater.owner( this );
                }

                else if( token == "battery:" ) {
                    // train heating device
                    m_batterysound.deserialize( parser, sound_type::single );
                    m_batterysound.owner( this );
                }

				else if( token == "turbo:" ) {
					// pliki z turbogeneratorem
                    m_powertrainsounds.engine_turbo.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    m_powertrainsounds.engine_turbo.owner( this );
                    m_powertrainsounds.engine_turbo.gain( 0 );
                }

				else if( token == "small-compressor:" ) {
					// pliki z przetwornica
                    sSmallCompressor.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sSmallCompressor.owner( this );
                }

                else if( token == "departuresignal:" ) {
					// pliki z sygnalem odjazdu
                    sound_source soundtemplate { sound_placement::general, 25.f };
                    soundtemplate.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    soundtemplate.owner( this );
                    for( auto &speaker : m_doorspeakers ) {
                        speaker.departure_signal = soundtemplate;
                        speaker.departure_signal.offset( speaker.offset );
                    }
                }

				else if( token == "dooropen:" ) {
                    sound_source soundtemplate { sound_placement::general };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &door : m_doorsounds ) {
                        // apply configuration to all defined doors, but preserve their individual offsets
                        auto const dooroffset { door.rsDoorOpen.offset() };
                        door.rsDoorOpen = soundtemplate;
                        door.rsDoorOpen.offset( dooroffset );
                    }
                }

				else if( token == "doorclose:" ) {
                    sound_source soundtemplate { sound_placement::general };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &door : m_doorsounds ) {
                        // apply configuration to all defined doors, but preserve their individual offsets
                        auto const dooroffset { door.rsDoorClose.offset() };
                        door.rsDoorClose = soundtemplate;
                        door.rsDoorClose.offset( dooroffset );
                    }
                }

                else if( token == "doorlock:" ) {
                    sound_source soundtemplate { sound_placement::general };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &door : m_doorsounds ) {
                        // apply configuration to all defined doors, but preserve their individual offsets
                        auto const dooroffset { door.lock.offset() };
                        door.lock = soundtemplate;
                        door.lock.offset( dooroffset );
                    }
                }

				else if( token == "doorunlock:" ) {
                    sound_source soundtemplate { sound_placement::general };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &door : m_doorsounds ) {
                        // apply configuration to all defined doors, but preserve their individual offsets
                        auto const dooroffset { door.unlock.offset() };
                        door.unlock = soundtemplate;
                        door.unlock.offset( dooroffset );
                    }
                }

                else if( token == "doorstepopen:" ) {
                    sound_source soundtemplate { sound_placement::general };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &door : m_doorsounds ) {
                        // apply configuration to all defined doors, but preserve their individual offsets
                        auto const dooroffset { door.step_open.offset() };
                        door.step_open = soundtemplate;
                        door.step_open.offset( dooroffset );
                    }
                }

                else if( token == "mirroropen:" || token == "mirrorclose:" ) {
                    sound_source soundtemplate { sound_placement::general };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &mirror : m_mirrorsounds ) {
                        // apply configuration to all defined mirrors, but preserve their individual offsets
                        auto &sound { token == "mirroropen:" ? mirror.unfold : mirror.fold };
                        auto const mirroroffset { sound.offset() };
                        sound = soundtemplate;
                        sound.offset( mirroroffset );
                    }
                }

                else if( token == "doorstepclose:" ) {
                    sound_source soundtemplate { sound_placement::general };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &door : m_doorsounds ) {
                        // apply configuration to all defined doors, but preserve their individual offsets
                        auto const dooroffset { door.step_close.offset() };
                        door.step_close = soundtemplate;
                        door.step_close.offset( dooroffset );
                    }
                }

                else if( token == "doorpermit:" ) {
                    sound_source soundtemplate { sound_placement::general };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &door : m_doorsounds ) {
                        // apply configuration to all defined doors, but preserve their individual offsets
                        auto const dooroffset { door.permit_granted.offset() };
                        door.permit_granted = soundtemplate;
                        door.permit_granted.offset( dooroffset );
                    }
                }

                else if( token == "unloading:" ) {
                    m_exchangesounds.unloading.range( MoverParameters->Dim.L * 0.5f * -1 );
                    m_exchangesounds.unloading.deserialize( parser, sound_type::single );
                    m_exchangesounds.unloading.owner( this );
                }

                else if( token == "loading:" ) {
                    m_exchangesounds.loading.range( MoverParameters->Dim.L * 0.5f * -1 );
                    m_exchangesounds.loading.deserialize( parser, sound_type::single );
                    m_exchangesounds.loading.owner( this );
                }

				else if( token == "sand:" ) {
                    sSand.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sSand.owner( this );
                }

				else if( token == "releaser:" ) {
					// pliki z odluzniaczem
                    sReleaser.deserialize( parser, sound_type::multipart, sound_parameters::range );
                    sReleaser.owner( this );
                }

                else if( token == "outernoise:" ) {
                    // szum podczas jazdy:
                    sound_source noisetemplate{ sound_placement::external, EU07_SOUND_RUNNINGNOISECUTOFFRANGE };
                    noisetemplate.deserialize( parser, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency, MoverParameters->Vmax );
                    noisetemplate.owner( this );

                    noisetemplate.m_amplitudefactor /= 1 + MoverParameters->Vmax;
                    noisetemplate.m_frequencyfactor /= 1 + MoverParameters->Vmax;
#ifdef EU07_SOUND_BOGIESOUNDS
                    if( true == m_bogiesounds.empty() ) {
                        // fallback for cases without specified noise locations, convert sound template to a single sound source
                        m_bogiesounds.emplace_back( noisetemplate );
                    }
                    else {
                        // apply configuration to all defined bogies
                        for( auto &bogie : m_bogiesounds ) {
                            // combine potential x- and y-axis offsets of the sound template with z-axis offsets of individual motors
                            auto bogieoffset{ noisetemplate.offset() };
                            bogieoffset.z = bogie.offset().z;
                            bogie = noisetemplate;
                            bogie.offset( bogieoffset );
                        }
                    }
                    // apply randomized playback start offset for each bogie, to reduce potential reverb with identical nearby sources
                    auto bogieidx( 0 );
                    for( auto &bogie : m_bogiesounds ) {
                        bogie.start( (
                            bogieidx % 2 ?
                                LocalRandom(  0.0, 30.0 ) :
                                LocalRandom( 50.0, 80.0 ) )
                            * 0.01 );
                        ++bogieidx;
                    }
#else
                    m_outernoise = noisetemplate;
                    // apply randomized playback start offset, to reduce potential reverb with identical nearby sources
                    m_outernoise.start( Random( 0.0, 80.0 ) * 0.01 );
#endif
                }

                else if( token == "wheelflat:" ) {
                    // szum podczas jazdy:
                    m_wheelflat.deserialize( parser, sound_type::single, sound_parameters::frequency );
                    m_wheelflat.owner( this );
                }

                else if(token == "announcements:") {
					// announcement sounds
                    // content provided as "key: value" pairs together enclosed in "{}"
                    // value can be optionally set of values enclosed in "[]" in which case one value will be picked randomly
                    std::unordered_map<std::string, announcement_t> const announcements = {
                        { "near_stop:", announcement_t::approaching },
                        { "stop:", announcement_t::current },
                        { "next_stop:", announcement_t::next },
                        { "destination:", announcement_t::destination },
                        { "chime:", announcement_t::chime } };
                    while( (token = parser.getToken<std::string>()) != ""
                        && token != "}" ) {
                        if( token.back() == ':' ) {

                            if( token == "soundproofing:" ) {
                                // custom soundproofing in format [ p1, p2, p3, p4, p5, p6 ]
                                parser.getTokens( 6, false, "\n\r\t ,;[]" );
                                std::array<float, 6> soundproofing;
                                parser
                                    >> soundproofing[ 0 ]
                                    >> soundproofing[ 1 ]
                                    >> soundproofing[ 2 ]
                                    >> soundproofing[ 3 ]
                                    >> soundproofing[ 4 ]
                                    >> soundproofing[ 5 ];
                                for( auto & soundproofingelement : soundproofing ) {
                                    if( soundproofingelement != -1.f ) {
                                        soundproofingelement = std::sqrt( std::clamp( soundproofingelement, 0.f, 1.f ) );
                                    }
                                }
                                m_pasystem.soundproofing = soundproofing;
                                continue;
                            }

                            auto const lookup { announcements.find( token ) };
                            auto const announcementtype { (
                                lookup != announcements.end() ?
                                    lookup->second :
                                    announcement_t::idle ) };
                            // NOTE: we retrieve key value for all keys, not just recognized ones
                            if( announcementtype == announcement_t::idle ) {
                                token = parser.getToken<std::string>();
                                continue;
                            }
/*
                            auto announcementsound { deserialize_random_set( parser ) };
                            replace_slashes( announcementsound );
*/
                            sound_source soundtemplate { sound_placement::engine }; // NOTE: sound range gets filled by pa system
                            soundtemplate.deserialize( parser, sound_type::single );
                            soundtemplate.owner( this );
                            m_pasystem.announcements[ static_cast<int>( announcementtype ) ] = soundtemplate;
                        }
                    }
                    // set provided custom soundproofing to assigned sounds (for sounds without their own custom soundproofing)
                    if( m_pasystem.soundproofing ) {
                        for( auto &announcement : m_pasystem.announcements ) {
                            if( !announcement.soundproofing() ) {
                                announcement.soundproofing() = m_pasystem.soundproofing;
                            }
                        }
                    }
                }

			} while( token != ""
				  && token != "endsounds" );

        } // sounds:

        else if( token == "locations:" ) {
            do {
                token = "";
                parser.getTokens(); parser >> token;

                if( token == "doors:" ) {
                    // a list of pairs: offset along vehicle's z-axis and sides on which the door is present; followed with "end"
                    while( (token = parser.getToken<std::string>()) != ""
                        && token != "end" ) {
                        // vehicle faces +Z in 'its' space, for door locations negative value means ahead of centre
                        auto const offset { std::atof( token.c_str() ) * -1.f };
                        // recognized side specifications are "right", "left" and "both"
                        auto const sides { parser.getToken<std::string>() };
                        // NOTE: we skip setting owner of the sounds, it'll be done during individual sound deserialization
                        door_sounds door;
                        // add entries to the list:
                        if( sides == "both"
                         || sides == "left" ) {
                            // left...
                            auto const location { glm::vec3 { MoverParameters->Dim.W * 0.5f, MoverParameters->Dim.H * 0.5f, offset } };
                            door.placement = side::left;
                            door.rsDoorClose.offset( location );
                            door.rsDoorOpen.offset( location );
                            door.lock.offset( location );
                            door.unlock.offset( location );
                            door.step_close.offset( location );
                            door.step_open.offset( location );
                            door.permit_granted.offset( location );
                            m_doorsounds.emplace_back( door );
                        }
                        if( sides == "both"
                         || sides == "right" ) {
                            // ...and right
                            auto const location { glm::vec3 { MoverParameters->Dim.W * -0.5f, 2.f, offset } };
                            door.placement = side::right;
                            door.rsDoorClose.offset( location );
                            door.rsDoorOpen.offset( location );
                            door.lock.offset( location );
                            door.unlock.offset( location );
                            door.step_close.offset( location );
                            door.step_open.offset( location );
                            door.permit_granted.offset( location );
                            m_doorsounds.emplace_back( door );
                        }
                        m_doorspeakers.emplace_back(
                            doorspeaker_sounds {
                                { 0.f, 3.f, offset },
                                {} } );
                    }
                }

                else if( token == "mirrors:" ) {
                    // a list of offsets along vehicle's z-axis of the mirror pairs, one at each cab; followed with "end"
                    while( (token = parser.getToken<std::string>()) != ""
                        && token != "end" ) {
                        // vehicle faces +Z in 'its' space, as for the doors negative value means ahead of centre
                        auto const offset { std::atof( token.c_str() ) * -1.f };
                        mirror_sounds mirror;
                        mirror.end = offset >= 0.f ? end::front : end::rear;
                        for( auto const mirrorside : { side::left, side::right } ) {
                            auto const location { glm::vec3 {
                                MoverParameters->Dim.W * ( mirrorside == side::left ? 0.5f : -0.5f ),
                                MoverParameters->Dim.H * 0.5f,
                                offset } };
                            mirror.placement = mirrorside;
                            mirror.unfold.offset( location );
                            mirror.fold.offset( location );
                            m_mirrorsounds.emplace_back( mirror );
                        }
                    }
                }

                else if( token == "tractionmotors:" ) {
                    // a list of offsets along vehicle's z-axis; followed with "end"
                    while( (token = parser.getToken<std::string>()) != ""
                        && token != "end" ) {
                        // vehicle faces +Z in 'its' space, for motor locations negative value means ahead of centre
                        auto const offset { std::atof( token.c_str() ) * -1.f };
                        // NOTE: we skip setting owner of the sounds, it'll be done during individual sound deserialization
                        sound_source motor { sound_placement::external }; // generic traction motor sounds
                        sound_source acmotor { sound_placement::external }; // inverter-specific traction motor sounds
                        sound_source motorblower { sound_placement::engine }; // associated motor blowers
                        // add entry to the list
                        auto const location { glm::vec3 { 0.f, 0.f, offset } };
                        motor.offset( location );
                        m_powertrainsounds.motors.emplace_back( motor );
                        acmotor.offset( location );
                        m_powertrainsounds.acmotors.emplace_back( acmotor );
                        motorblower.offset( location );
                        m_powertrainsounds.motorblowers.emplace_back( motorblower );
                    }
                }

                else if( token == "bogies:" ) {
                    // a list of offsets along vehicle's z-axis; followed with "end"
                    while( (token = parser.getToken<std::string>()) != ""
                        && token != "end" ) {
                        // vehicle faces +Z in 'its' space, for bogie locations negative value means ahead of centre
                        auto const offset { std::atof( token.c_str() ) * -1.f };
                        // NOTE: we skip setting owner of the sounds, it'll be done during individual sound deserialization
                        sound_source bogienoise { sound_placement::external, EU07_SOUND_RUNNINGNOISECUTOFFRANGE };
                        // add entry to the list
                        auto const location { glm::vec3 { 0.f, 0.f, offset } };
                        bogienoise.offset( location );
#ifdef EU07_SOUND_BOGIESOUNDS
                        m_bogiesounds.emplace_back( bogienoise );
#endif
                    }
                }

            } while( token != ""
                  && token != "endlocations" );

        } // locations:

        else if( token == "internaldata:" ) {
            // dalej nie czytaj
            do {
                // zbieranie informacji o kabinach
                token = "";
                parser.getTokens(); parser >> token;
                if( token == "cab0model:" ) {
                    parser.getTokens(); parser >> token;
                    if( token != "none" ) { iCabs |= 0x2; }
                }
                else if( token == "cab1model:" ) {
                    parser.getTokens(); parser >> token;
                    if( token != "none" ) { iCabs |= 0x1; }
                }
                else if( token == "cab2model:" ) {
                    parser.getTokens(); parser >> token;
                    if( token != "none" ) { iCabs |= 0x4; }
                }
                // engine sounds
                else if( token == "ignition:" ) {
                    // odpalanie silnika
                    m_powertrainsounds.engine_ignition.deserialize( parser, sound_type::single );
                    m_powertrainsounds.engine_ignition.owner( this );
                } else if (token == "shutdown:") {
                    m_powertrainsounds.engine_shutdown.deserialize(parser, sound_type::single);
                    m_powertrainsounds.engine_shutdown.owner(this);
				} else if( token == "engageslippery:" ) {
                    // tarcie tarcz sprzegla:
                    m_powertrainsounds.rsEngageSlippery.deserialize( parser, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency );
                    m_powertrainsounds.rsEngageSlippery.owner( this );

                    m_powertrainsounds.rsEngageSlippery.m_frequencyfactor /= 1 + MoverParameters->nmax;
                }

                // dzwieki wycieraczkuf
                else if (token == "wiperfrompark:")
                {
					sWiperFromPark.deserialize(parser, sound_type::single);
					sWiperFromPark.owner(this);  
                }
				else if (token == "wipertopark:")
				{
					sWiperToPark.deserialize(parser, sound_type::single);
					sWiperToPark.owner(this);  
				}

				else if (token == "retarder:") {
					m_powertrainsounds.retarder.deserialize(parser, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency);
					m_powertrainsounds.retarder.owner(this);
				}
				else if( token == "linebreakerclose:" ) {
                    m_powertrainsounds.linebreaker_close.deserialize( parser, sound_type::single );
                    m_powertrainsounds.linebreaker_close.owner( this );
                }
                else if( token == "linebreakeropen:" ) {
                    m_powertrainsounds.linebreaker_open.deserialize( parser, sound_type::single );
                    m_powertrainsounds.linebreaker_open.owner( this );
                }
                else if( token == "relay:" ) {
                    // styczniki itp:
                    m_powertrainsounds.motor_relay.deserialize( parser, sound_type::single );
                    m_powertrainsounds.motor_relay.owner( this );
                }
                else if( token == "shuntfield:" ) {
                    // styczniki itp:
                    m_powertrainsounds.motor_shuntfield.deserialize( parser, sound_type::single );
                    m_powertrainsounds.motor_shuntfield.owner( this );
                }
                else if( token == "wejscie_na_bezoporow:" ) {
                    // hunter-111211: wydzielenie wejscia na bezoporowa i na drugi uklad do pliku
                    m_powertrainsounds.dsbWejscie_na_bezoporow.deserialize( parser, sound_type::single );
                    m_powertrainsounds.dsbWejscie_na_bezoporow.owner( this );
                }
                // przekazniki od kierunku jazdy
                else if (token == "directionrelay_d:") {
                    // przekaznik od kierunku jazdy do przodu
					sDirectionRelayD.deserialize(parser, sound_type::single);
                    sDirectionRelayD.owner(this);
                }
                else if (token == "directionrelay_r:") {
					// przekaznik od kierunku jazdy do tylu
					sDirectionRelayR.deserialize(parser, sound_type::single);
					sDirectionRelayR.owner(this);
                }
				else if (token == "directionrelay_n:")
				{
					// przekaznik od kierunku jazdy do tylu
					sDirectionRelayN.deserialize(parser, sound_type::single);
					sDirectionRelayN.owner(this);
				}
                else if( token == "wejscie_na_drugi_uklad:" ) {
                    m_powertrainsounds.motor_parallel.deserialize( parser, sound_type::single );
                    m_powertrainsounds.motor_parallel.owner( this );
                }
                else if( token == "pneumaticrelay:" ) {
                    // wylaczniki pneumatyczne:
                    dsbPneumaticRelay.deserialize( parser, sound_type::single );
                    dsbPneumaticRelay.owner( this );
                }
                else if( token == "slipperysound:" ) {
                    // sanie:
                    rsSlippery.deserialize( parser, sound_type::single, sound_parameters::amplitude );
                    rsSlippery.owner( this );

                    rsSlippery.m_amplitudefactor /= 1 + MoverParameters->Vmax;
                }
                // coupler sounds
                else if( token == "couplerattach:" ) {
                    sound_source soundtemplate { sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.attach_coupler = soundtemplate;
                    }
                }
                else if( token == "brakehoseattach:" ) {
                    // laczenie:
                    sound_source soundtemplate { sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.attach_brakehose = soundtemplate;
                    }
                }
                else if( token == "mainhoseattach:" ) {
                    // laczenie:
                    sound_source soundtemplate{ sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.attach_mainhose = soundtemplate;
                    }
                }
                else if( token == "controlattach:" ) {
                    // laczenie:
                    sound_source soundtemplate{ sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.attach_control = soundtemplate;
                    }
                }
                else if( token == "gangwayattach:" ) {
                    // laczenie:
                    sound_source soundtemplate{ sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.attach_gangway = soundtemplate;
                    }
                }
                else if( token == "heatingattach:" ) {
                    // laczenie:
                    sound_source soundtemplate{ sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.attach_heating = soundtemplate;
                    }
                }
                else if( token == "couplerdetach:" ) {
                    // rozlaczanie:
                    sound_source soundtemplate { sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.detach_coupler = soundtemplate;
                    }
                }
                else if( token == "brakehosedetach:" ) {
                    // rozlaczanie:
                    sound_source soundtemplate { sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.detach_brakehose = soundtemplate;
                    }
                }
                else if( token == "mainhosedetach:" ) {
                    // rozlaczanie:
                    sound_source soundtemplate{ sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.detach_mainhose = soundtemplate;
                    }
                }
                else if( token == "controldetach:" ) {
                    // rozlaczanie:
                    sound_source soundtemplate{ sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.detach_control = soundtemplate;
                    }
                }
                else if( token == "gangwaydetach:" ) {
                    // rozlaczanie:
                    sound_source soundtemplate{ sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.detach_gangway = soundtemplate;
                    }
                }
                else if( token == "heatingdetach:" ) {
                    // rozlaczanie:
                    sound_source soundtemplate{ sound_placement::external };
                    soundtemplate.deserialize( parser, sound_type::single );
                    soundtemplate.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.detach_heating = soundtemplate;
                    }
                }
                else if( token == "couplerstretch:" ) {
                    // coupler stretching
                    sound_source couplerstretch { sound_placement::external };
                    couplerstretch.deserialize( parser, sound_type::single );
                    couplerstretch.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.dsbCouplerStretch = couplerstretch;
                    }
                }
                else if( token == "couplerstretch_loud:" ) {
                    // coupler stretching
                    sound_source couplerstretch { sound_placement::external };
                    couplerstretch.deserialize( parser, sound_type::single );
                    couplerstretch.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.dsbCouplerStretch_loud = couplerstretch;
                    }
                }
                else if( token == "bufferclamp:" ) {
                    // buffers hitting one another
                    sound_source bufferclash { sound_placement::external };
                    bufferclash.deserialize( parser, sound_type::single );
                    bufferclash.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.dsbBufferClamp = bufferclash;
                    }
                }
                else if( token == "bufferclamp_loud:" ) {
                    // buffers hitting one another
                    sound_source bufferclash { sound_placement::external };
                    bufferclash.deserialize( parser, sound_type::single );
                    bufferclash.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.dsbBufferClamp_loud = bufferclash;
                    }
                }
                else if( token == "coupleradapterattach:" ) {
                    // laczenie:
                    sound_source adapterattach { sound_placement::external };
                    adapterattach.deserialize( parser, sound_type::single );
                    adapterattach.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.dsbAdapterAttach = adapterattach;
                    }
                }
                else if( token == "coupleradapterremove:" ) {
                    // rozlaczanie:
                    sound_source adapterremove { sound_placement::external };
                    adapterremove.deserialize( parser, sound_type::single );
                    adapterremove.owner( this );
                    for( auto &couplersounds : m_couplersounds ) {
                        couplersounds.dsbAdapterRemove = adapterremove;
                    }
                }
                else if( token == "startjolt:" ) {
                    // movement start jolt
                    m_startjolt.deserialize( parser, sound_type::single );
                    m_startjolt.owner( this );
                }

                else if (token == "mechspring:")
                {
                    // parametry bujania kamery:
                    double ks, kd;
                    parser.getTokens(2, false);
                    parser
                        >> ks
                        >> kd;
                    ShakeSpring.Init(ks, kd);
                    parser.getTokens(6, false);
                    parser
                        >> BaseShake.jolt_scale.x
                        >> BaseShake.jolt_scale.y
                        >> BaseShake.jolt_scale.z
                        >> BaseShake.jolt_limit
                        >> BaseShake.angle_scale.x
                        >> BaseShake.angle_scale.z;
                }
                else if( token == "enginespring:" ) {
                    parser.getTokens( 5, false );
                    parser
                        >> EngineShake.scale
                        >> EngineShake.fadein_offset
                        >> EngineShake.fadein_factor
                        >> EngineShake.fadeout_offset
                        >> EngineShake.fadeout_factor;
                    // offsets values are provided as rpm for convenience
                    EngineShake.fadein_offset /= 60.f;
                    EngineShake.fadeout_offset /= 60.f;
                }
                else if( token == "huntingspring:" ) {
                    parser.getTokens( 4, false );
                    parser
                        >> HuntingShake.scale
                        >> HuntingShake.frequency
                        >> HuntingShake.fadein_begin
                        >> HuntingShake.fadein_end;
                }

                else if( token == "soundproofing:" ) {
                    for( auto &soundproofingtable : m_soundproofing ) {
                        for( auto &soundproofingelement : soundproofingtable ) {
                            auto const value { parser.getToken<float>( false ) };
                            if( value != -1.f ) {
                                soundproofingelement = std::sqrt( std::clamp( value, 0.f, 1.f ) );
                            }
                        }
                    }
                }

                else if( token == "jointcabs:" ) {
                    parser.getTokens();
                    parser >> JointCabs;
                }

                else if( token == "cablight:" ) {
                    parser.getTokens( 3, false ); // low power light, ignore
                    parser.getTokens( 3, false ); // base light
                    parser
                        >> InteriorLight.r
                        >> InteriorLight.g
                        >> InteriorLight.b;
                    InteriorLight = glm::clamp( InteriorLight / 255.f, glm::vec3( 0.f ), glm::vec3( 1.f ) );
                    parser.getTokens( 3, false ); // dimmed light, ignore
                }

                else if( token == "pydestinationsign:" ) {
                    DestinationSign.deserialize( parser );
                    // supply vehicle folder as script path if none is provided
                    if( false == DestinationSign.script.empty()
                     && substr_path(DestinationSign.script).empty() ) {
                        DestinationSign.script = asBaseDir + DestinationSign.script;
                    }
                }
                else if( token == "luadestinationsign:" ) {
                    deserialize_script_sign( parser, DestinationSign.lua, asBaseDir );
                }
                else if( token == "wasmdestinationsign:" ) {
                    deserialize_script_sign( parser, DestinationSign.wasm, asBaseDir );
                }
                // NOTE: legacy key, now expected as optional "background:" parameter in pydestinationsign: { parameter block }
                else if( token == "destinationsignbackground:" ) {
                    parser.getTokens();
                    parser >> DestinationSign.background;
                }

            } while( token != "" );

        } // internaldata:

	} while( token != "" );

    if( !iAnimations ) {
        // if the animations weren't defined the model is likely to be non-functional. warrants a warning.
        ErrorLog( "Animations tag is missing from the .mmd file \"" + asFileName + "\"" );
    }

    if( ReplacableSkin != "none" ) {
        // potentially set blank destination texture
        auto language { script_language::lua };
        auto const *scriptsign { script_destination( language ) };
        auto const &background { ( scriptsign != nullptr && false == scriptsign->background.empty() ) ? scriptsign->background : DestinationSign.background };
        DestinationSign.destination_off = DestinationFind( background.empty() ? "nowhere" : background );
        // a blank sign a script draws, drawn once
        DestinationSign.background_sign = draw_script_texture( background, DestinationSign.destination_off );
    }

    // assign default samples to sound emitters which weren't included in the config file
    if( TestFlag( MoverParameters->CategoryFlag, 1 ) ) {
        // rail vehicles:
        // engine
        if( MoverParameters->Power > 0 ) {
            if( true == m_powertrainsounds.dsbWejscie_na_bezoporow.empty() ) {
                // hunter-111211: domyslne, gdy brak
                m_powertrainsounds.dsbWejscie_na_bezoporow.deserialize( "wejscie_na_bezoporow", sound_type::single );
                m_powertrainsounds.dsbWejscie_na_bezoporow.owner( this );
            }
            if( true == m_powertrainsounds.motor_parallel.empty() ) {
                m_powertrainsounds.motor_parallel.deserialize( "wescie_na_drugi_uklad", sound_type::single );
                m_powertrainsounds.motor_parallel.owner( this );
            }
        }
        // braking sounds
        if( true == rsUnbrake.empty() ) {
            rsUnbrake.deserialize( "[1007]estluz", sound_type::single );
            rsUnbrake.owner( this );
        }
        // couplers
        for( auto &couplersounds : m_couplersounds ) {
            if( true == couplersounds.attach_coupler.empty() ) {
                couplersounds.attach_coupler.deserialize( "couplerattach_default", sound_type::single );
                couplersounds.attach_coupler.owner( this );
            }
            if( true == couplersounds.detach_coupler.empty() ) {
                couplersounds.detach_coupler.deserialize( "couplerdetach_default", sound_type::single );
                couplersounds.detach_coupler.owner( this );
            }
            if( true == couplersounds.dsbCouplerStretch.empty() ) {
                couplersounds.dsbCouplerStretch.deserialize( "couplerstretch_default", sound_type::single );
                couplersounds.dsbCouplerStretch.owner( this );
            }
            if( true == couplersounds.dsbBufferClamp.empty() ) {
                couplersounds.dsbBufferClamp.deserialize( "bufferclamp_default", sound_type::single );
                couplersounds.dsbBufferClamp.owner( this );
            }
        }
        // other sounds
        if( true == m_wheelflat.empty() ) {
            m_wheelflat.deserialize( "lomotpodkucia 0.23 0.0", sound_type::single, sound_parameters::frequency );
            m_wheelflat.owner( this );
        }
        if( true == rscurve.empty() ) {
            // hunter-111211: domyslne, gdy brak
            rscurve.deserialize( "curve", sound_type::single );
            rscurve.owner( this );
        }
    }

    if (mdModel)
        mdModel->Init(); // obrócenie modelu oraz optymalizacja, również zapisanie binarnego
    if (mdLoad)
        mdLoad->Init();
    if (mdLowPolyInt)
        mdLowPolyInt->Init();
    for( auto *attachment : mdAttachments ) {
        attachment->Init();
    }

    Global.asCurrentTexturePath = paths::textures; // kiedyś uproszczone wnętrze mieszało tekstury nieba
    Global.asCurrentDynamicPath = "";

    // position sound emitters which weren't defined in the config file
    // engine sounds, centre of the vehicle
    auto const enginelocation { glm::vec3 {0.f, MoverParameters->Dim.H * 0.5f, 0.f } };
    m_powertrainsounds.position( enginelocation );
    // other engine compartment sounds
    auto const nullvector { glm::vec3() };
    std::vector<sound_source *> enginesounds = {
        &sConverter, &sBRVent, &sCompressor, &sCompressorIdle, &sSmallCompressor, &sHeater, &m_batterysound
    };
    for( auto sound : enginesounds ) {
        if( sound->offset() == nullvector ) {
            sound->offset( enginelocation );
        }
    }

    // pantographs
    if( pants != nullptr ) {
        std::size_t pantographindex { 0 };
        for( auto &pantographsounds : m_pantographsounds ) {
            auto const pantographoffset {
                glm::vec3(
                    0.f,
                    pants[ pantographindex ].fParamPants->vPos.y,
                    -pants[ pantographindex ].fParamPants->vPos.x ) };
            pantographsounds.sPantUp.offset( pantographoffset );
            pantographsounds.sPantDown.offset( pantographoffset );
            ++pantographindex;
        }
    }
    // doors
    std::size_t dooranimationfirstindex { 0 };
    for( std::size_t i = 0; i < ANIM_DOORS; ++i ) {
        // zliczanie wcześniejszych animacji
        dooranimationfirstindex += iAnimType[ i ];
    }
    // couplers
    auto const frontcoupleroffset { glm::vec3{ 0.f, 1.f, MoverParameters->Dim.L * 0.5f } };
    m_couplersounds[ end::front ].attach_coupler.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].attach_brakehose.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].attach_mainhose.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].attach_control.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].attach_gangway.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].attach_heating.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].detach_coupler.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].detach_brakehose.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].detach_mainhose.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].detach_control.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].detach_gangway.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].detach_heating.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].dsbCouplerStretch.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].dsbCouplerStretch_loud.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].dsbBufferClamp.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].dsbBufferClamp_loud.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].dsbAdapterAttach.offset( frontcoupleroffset );
    m_couplersounds[ end::front ].dsbAdapterRemove.offset( frontcoupleroffset );
    auto const rearcoupleroffset { glm::vec3{ 0.f, 1.f, MoverParameters->Dim.L * -0.5f } };
    m_couplersounds[ end::rear ].attach_coupler.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].attach_brakehose.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].attach_mainhose.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].attach_control.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].attach_gangway.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].attach_heating.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].detach_coupler.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].detach_brakehose.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].detach_mainhose.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].detach_control.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].detach_gangway.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].detach_heating.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].dsbCouplerStretch.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].dsbCouplerStretch_loud.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].dsbBufferClamp.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].dsbBufferClamp_loud.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].dsbAdapterAttach.offset( rearcoupleroffset );
    m_couplersounds[ end::rear ].dsbAdapterRemove.offset( rearcoupleroffset );
}

TModel3d *
TDynamicObject::LoadMMediaFile_mdload( std::string const &Name ) const {

    auto const loadname { ( Name.empty() ? "none" : Name ) };
    TModel3d *loadmodel { nullptr };

    // check if we don't have model override for this load type
    if ( loadmodel == nullptr )
    {
        auto const lookup { LoadModelOverrides.find( loadname ) };
        if( lookup != LoadModelOverrides.end() ) {
            loadmodel = TModelsManager::GetModel( asBaseDir + lookup->second, true );
        }
    }
    // regular routine if there's no override or it couldn't be loaded
    // try first specialized version of the load model, vehiclename_loadname
    if ( loadmodel == nullptr )
    {
        auto const specializedloadfilename { asBaseDir + MoverParameters->TypeName + "_" + loadname };
        loadmodel = TModelsManager::GetModel( specializedloadfilename, true, false );
    }
    // try generic version of the load model next, loadname
    if ( loadmodel == nullptr )
    {
        auto const genericloadfilename { asBaseDir + loadname };
        loadmodel = TModelsManager::GetModel( genericloadfilename, true, false );
    }

    if ( loadmodel != nullptr )
        loadmodel->GetSMRoot()->WillBeAnimated();

    return loadmodel;
}
