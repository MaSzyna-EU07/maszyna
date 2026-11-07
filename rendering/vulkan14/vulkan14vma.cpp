/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the implementation of the vulkan memory allocator (ref/VulkanMemoryAllocator). the functions of vulkan are given to it by
// vk14::allocator, as volk loaded them (vmaImportVulkanFunctionsFromVolk)

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14common.h"

#define VMA_STATIC_VULKAN_FUNCTIONS 0
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 0
#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>
