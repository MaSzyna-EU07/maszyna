/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include "QueryParserComp.hpp"
#include "usefull.h"

export module eu07.world.curve;

export {



class TCurve
{
  public:
    TCurve();
    ~TCurve();
    bool Init(int n, int c);
    float GetValue(int c, float p);
    bool SetValue(int c, float p, float v);
    bool Load(TQueryParserComp *Parser);
    bool LoadFromFile(AnsiString asName);
    bool SaveToFile(AnsiString asName);

    int iNumValues;
    int iNumCols;

  private:
    float **Values;
};
//---------------------------------------------------------------------------

}  // export
