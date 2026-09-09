module;

export module eu07.application.driverhints;

export {

// The hint list is an X-macro file expanded once here for the enumerators and
// again, with a different DRIVER_HINT_DEF, for the texts in driverhints_texts.h.
#define DRIVER_HINT_DEF(a, b) a,

enum class driver_hint {
    #include "application/driverhints_def.h"
};

#undef DRIVER_HINT_DEF

}  // export
