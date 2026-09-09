module;
#include "utilities/crashreporter_macros.h"
#include <string>

export module eu07.utilities.crashreporter;

export {


#ifdef WITH_CRASHPAD
void crashreport_add_info(const char *name, const std::string &value);
const std::string& crashreport_get_provider();
void crashreport_set_autoupload();
bool crashreport_is_pending();
void crashreport_upload_reject();
void crashreport_upload_accept();
#else
#endif

}  // export
