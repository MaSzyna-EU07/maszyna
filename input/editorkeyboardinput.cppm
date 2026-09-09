/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;

export module eu07.input.editorkeyboardinput;
import eu07.input.keyboardinput;

export {



class editorkeyboard_input : public keyboard_input {

public:
// methods
    bool
        init() override;
    // re-applies default bindings for the current movement scheme (e.g. after a settings change)
    void
        apply_scheme();

protected:
// methods
    void
        default_bindings() override;
};

//---------------------------------------------------------------------------

}  // export
