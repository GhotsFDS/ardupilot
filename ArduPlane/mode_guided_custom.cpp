#include "Plane.h"

#if AP_SCRIPTING_ENABLED
// constructor registers custom number and names
ModeGuidedCustom::ModeGuidedCustom(const Number _number, const char* _full_name, const char* _short_name):
    number(_number),
    full_name(_full_name),
    short_name(_short_name)
{
}

bool ModeGuidedCustom::_enter()
{
    // Script can block entry
    if (!state.allow_entry) {
        return false;
    }

    // Guided entry checks must also pass
    return ModeGuided::_enter();
}

#endif
