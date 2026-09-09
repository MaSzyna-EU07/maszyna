/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

//---------------------------------------------------------------------------
module;

export module eu07.console.lpt;

export {

class TLPT
{
  private:
    int address;

  public:
    bool Connect(int port);
    void Out(int x);
};

}  // export
