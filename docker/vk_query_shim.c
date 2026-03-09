/* LD_PRELOAD shim: intercept vkCreateQueryPool to handle unsupported query types.
 *
 * Mesa DZN only supports OCCLUSION, PIPELINE_STATISTICS, and TIMESTAMP query types.
 * UE5 creates RT acceleration structure compaction query pools during InitGPU()
 * regardless of whether the driver advertises VK_KHR_acceleration_structure.
 *
 * This shim substitutes TIMESTAMP for any unsupported query type, returning a
 * valid pool handle so UE5 doesn't crash. Since DZN doesn't advertise RT extensions,
 * UE5 never issues RT queries — the dummy pool goes unused.
 *
 * Build: gcc -shared -fPIC -o libvk_query_shim.so vk_query_shim.c -ldl
 * Use:   LD_PRELOAD=/path/to/libvk_query_shim.so ./Blocks ...
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Minimal Vulkan types to avoid header dependency */
typedef uint32_t VkFlags;
typedef uint64_t VkDeviceSize;
typedef enum VkResult { VK_SUCCESS = 0 } VkResult;
typedef enum VkStructureType { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO = 11 } VkStructureType;
typedef enum VkQueryType {
    VK_QUERY_TYPE_OCCLUSION = 0,
    VK_QUERY_TYPE_PIPELINE_STATISTICS = 1,
    VK_QUERY_TYPE_TIMESTAMP = 2,
} VkQueryType;
typedef struct VkDevice_T *VkDevice;
typedef struct VkQueryPool_T *VkQueryPool;
typedef struct VkAllocationCallbacks VkAllocationCallbacks;

typedef struct VkQueryPoolCreateInfo {
    VkStructureType sType;
    const void *pNext;
    VkFlags flags;
    VkQueryType queryType;
    uint32_t queryCount;
    VkFlags pipelineStatistics;
} VkQueryPoolCreateInfo;

typedef VkResult (*PFN_vkCreateQueryPool)(VkDevice, const VkQueryPoolCreateInfo *,
                                          const VkAllocationCallbacks *, VkQueryPool *);

VkResult vkCreateQueryPool(VkDevice device,
                           const VkQueryPoolCreateInfo *pCreateInfo,
                           const VkAllocationCallbacks *pAllocator,
                           VkQueryPool *pQueryPool) {
    static PFN_vkCreateQueryPool real_fn = NULL;
    if (!real_fn) {
        real_fn = (PFN_vkCreateQueryPool)dlsym(RTLD_NEXT, "vkCreateQueryPool");
        if (!real_fn) {
            fprintf(stderr, "[vk_query_shim] FATAL: cannot find real vkCreateQueryPool\n");
            return -1;
        }
    }

    /* Supported types pass through unchanged */
    if (pCreateInfo->queryType == VK_QUERY_TYPE_OCCLUSION ||
        pCreateInfo->queryType == VK_QUERY_TYPE_PIPELINE_STATISTICS ||
        pCreateInfo->queryType == VK_QUERY_TYPE_TIMESTAMP) {
        return real_fn(device, pCreateInfo, pAllocator, pQueryPool);
    }

    /* Unsupported type (e.g. VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR):
     * substitute TIMESTAMP so DZN can create a real pool */
    fprintf(stderr, "[vk_query_shim] Substituting query type %d -> TIMESTAMP\n",
            pCreateInfo->queryType);

    VkQueryPoolCreateInfo modified;
    memcpy(&modified, pCreateInfo, sizeof(modified));
    modified.queryType = VK_QUERY_TYPE_TIMESTAMP;

    return real_fn(device, &modified, pAllocator, pQueryPool);
}
