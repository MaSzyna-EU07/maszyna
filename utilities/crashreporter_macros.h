#pragma once

// Macros cannot cross a module boundary, so they live in a plain header
// that consumers pull into their global module fragment. The bodies are
// expanded at the use site, where the imported names are in scope, so this
// header deliberately includes nothing.

#define crashreport_add_info(a,b)
#define crashreport_get_provider() (std::string(""))
#define crashreport_set_autoupload()
#define crashreport_is_pending() (false)
#define crashreport_upload_reject()
#define crashreport_upload_accept()
