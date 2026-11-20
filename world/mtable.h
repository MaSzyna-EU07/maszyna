/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>

#include "utilities/Classes.h"
#include "audio/sound.h"
#include "world/timetablereader.h"

namespace Mtable
{

static char const *const hrsd = ".";

// Ra: pozycja zerowa rozkładu chyba nie ma sensu
// Ra: numeracja przystanków jest 1..StationCount

// a station of the timetable as the file gives it (TMTableEntry, world/timetablereader.h), with its name announcement
struct TMTableLine : TMTableEntry
{
    sound_source name_sound{ sound_placement::engine };
};

using TMTable = TMTableLine[MaxTTableSize + 1];

// typedef TTrainParameters *PTrainParameters;

class TTrainParameters
{
  public:
    std::string TrainName;
	std::string TrainCategory;
    std::string TrainLabel;
    double TTVmax;
    std::string Relation1;
    std::string Relation2; // nazwy stacji danego odcinka
    // the two as the timetable spells them, with their letters (see TMTableEntry::StationLabel)
    std::string RelationLabel1;
    std::string RelationLabel2;
    double BrakeRatio;
    std::string LocSeries; // seria (typ) pojazdu
    double LocLoad;
    TMTable TimeTable; // TimeTable[0].flags holds the service flags of the whole train
    int StationCount; // ilość przystanków (0-techniczny)
    int StationIndex; // numer najbliższego (aktualnego) przystanku
    int StationStart; // numer pierwszej stacji pokazywanej na podglądzie rozkładu
    std::string NextStationName;
    double LastStationLatency;
    int Direction; /*kierunek jazdy w/g kilometrazu*/
    double CheckTrainLatency() const;
    /*todo: str hh:mm to int i z powrotem*/
    std::string ShowRelation() const;
    // the relation and the name of given station for the player: utf-8, with the letters the timetable has
    std::string relation_caption() const;
    std::string station_caption( std::string const &Name ) const;
    double WatchMTable(double DistCounter) const;
    std::string NextStop() const;
    // wpis rozkladu Offset pozycji za StationIndex (Offset>=1); poza zakresem 1..StationCount
    // zwraca techniczny wpis zerowy (TimeTable[0], StationName=="nowhere", Ah==-1)
    TMTableLine const &TimeTableEntryAhead( int Offset ) const;
    sound_source next_stop_sound() const;
    sound_source last_stop_sound() const;
    bool IsStop() const;
    bool IsLastStop() const;
    bool IsMaintenance() const;
    bool IsTimeToGo(double hh, double mm) const;
    // returns: difference between specified time and scheduled departure from current stop, in seconds
    double seconds_until_departure( double const Hour, double const Minute ) const;
    bool UpdateMTable(double hh, double mm, std::string_view NewName);
    bool UpdateMTable( scenario_time const &Time, std::string const &NewName );
    bool RewindTimeTable( std::string actualStationName );
    explicit TTrainParameters( std::string const &NewTrainName = "none" );
    void NewName(std::string_view NewTrainName);
    void UpdateVelocity(int StationCount, double vActual);
    bool LoadTTfile(std::string const &scnpath, int iPlus, double vmax);
    bool DirectionChange() const;
    void StationIndexInc();
    void serialize( dictionary_source *Output ) const;
    // returns: radio channel associated with current station, or -1
    int radio_channel() const;
    // returns: sound file associated with current station, or -1
    sound_source current_stop_sound() const;
    // service flags (eu07/maszyna#95) which hold at the stop the train makes for or stands at
    timetable::service_flags const &stop_flags() const;
    // service flags which hold on the way, since the stop the train left last
    timetable::service_flags const &run_flags() const;
private:
    void load_sounds();
};

class TMTableTime

{
  public:
    int dd = 0;
    int hh = 0;
    int mm = 0;
    double mr = 0.0;
    void UpdateMTableTime(double deltaT);
    TMTableTime(int InitH, int InitM ) :
        hh( InitH ),
        mm( InitM )
	{}

	TMTableTime() = default;
};

}

#if !defined(NO_IMPLICIT_NAMESPACE_USE)
using namespace Mtable;
#endif
