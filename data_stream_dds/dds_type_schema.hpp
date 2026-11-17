#pragma once

#include <dds/ddsi/ddsi_xt_typeinfo.h>
#include <map>
#include <string>

// Complete TypeObjects already resolved by the live decoder, keyed by their
// 14-byte equivalence hash. No remote lookup or DDS pointer escapes this call.
using DdsSchemaObjects = std::map<std::string, const DDS_XTypes_CompleteTypeObject*>;
std::string ddsTypeSchema(const DDS_XTypes_TypeIdentifier& root, const DdsSchemaObjects& objects);
