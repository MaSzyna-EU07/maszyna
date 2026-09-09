/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;

export module eu07.input.driverkeyboardinput;
import eu07.input.keyboardinput;

export {



class driverkeyboard_input : public keyboard_input {

public:
// methods
    bool
        init() override;

protected:
// methods
    void
        default_bindings();
};

//---------------------------------------------------------------------------

}  // export
