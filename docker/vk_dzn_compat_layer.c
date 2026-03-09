/* Vulkan implicit layer: DZN compatibility shim for UE5.
 *
 * Intercepts vkCreateQueryPool via proper Vulkan dispatch chain.
 * Substitutes TIMESTAMP for unsupported query types (e.g. RT compaction)
 * that Mesa DZN doesn't handle, preventing UNREACHABLE crashes.
 *
 * Build: gcc -shared -fPIC -o libVkLayer_dzn_compat.so vk_dzn_compat_layer.c -ldl
 */
#include <vulkan/vulkan.h>
#include <vulkan/vk_layer.h>
#include <string.h>
#include <stdio.h>

/* Per-device dispatch table (simplified — single device) */
static PFN_vkCreateQueryPool g_real_create_query_pool = NULL;
static PFN_vkGetDeviceProcAddr g_real_get_device_proc_addr = NULL;

static VkResult VKAPI_CALL
dzn_compat_vkCreateQueryPool(VkDevice device,
                             const VkQueryPoolCreateInfo *pCreateInfo,
                             const VkAllocationCallbacks *pAllocator,
                             VkQueryPool *pQueryPool)
{
    if (!g_real_create_query_pool)
        return VK_ERROR_INITIALIZATION_FAILED;

    /* DZN supports: OCCLUSION(0), PIPELINE_STATISTICS(1), TIMESTAMP(2) */
    if (pCreateInfo->queryType <= VK_QUERY_TYPE_TIMESTAMP) {
        return g_real_create_query_pool(device, pCreateInfo, pAllocator, pQueryPool);
    }

    /* Unsupported type — substitute TIMESTAMP */
    fprintf(stderr, "[dzn_compat] Redirecting query type %d -> TIMESTAMP\n",
            pCreateInfo->queryType);

    VkQueryPoolCreateInfo modified = *pCreateInfo;
    modified.queryType = VK_QUERY_TYPE_TIMESTAMP;
    return g_real_create_query_pool(device, &modified, pAllocator, pQueryPool);
}

static PFN_vkVoidFunction VKAPI_CALL
dzn_compat_vkGetDeviceProcAddr(VkDevice device, const char *pName)
{
    if (strcmp(pName, "vkCreateQueryPool") == 0)
        return (PFN_vkVoidFunction)dzn_compat_vkCreateQueryPool;

    if (g_real_get_device_proc_addr)
        return g_real_get_device_proc_addr(device, pName);

    return NULL;
}

static PFN_vkVoidFunction VKAPI_CALL
dzn_compat_vkGetInstanceProcAddr(VkInstance instance, const char *pName);

/* Layer negotiate interface — Vulkan loader calls this first */
VkResult VKAPI_CALL
vkNegotiateLoaderLayerInterfaceVersion(VkNegotiateLayerInterface *pVersionStruct)
{
    if (!pVersionStruct || pVersionStruct->sType != LAYER_NEGOTIATE_INTERFACE_STRUCT)
        return VK_ERROR_INITIALIZATION_FAILED;

    if (pVersionStruct->loaderLayerInterfaceVersion > 2)
        pVersionStruct->loaderLayerInterfaceVersion = 2;

    pVersionStruct->pfnGetInstanceProcAddr = dzn_compat_vkGetInstanceProcAddr;
    pVersionStruct->pfnGetDeviceProcAddr = dzn_compat_vkGetDeviceProcAddr;
    pVersionStruct->pfnGetPhysicalDeviceProcAddr = NULL;

    return VK_SUCCESS;
}

/* Instance-level interception for CreateDevice to capture device dispatch */
static PFN_vkCreateDevice g_real_create_device = NULL;
static PFN_vkGetInstanceProcAddr g_real_get_instance_proc_addr = NULL;

static VkResult VKAPI_CALL
dzn_compat_vkCreateDevice(VkPhysicalDevice physicalDevice,
                          const VkDeviceCreateInfo *pCreateInfo,
                          const VkAllocationCallbacks *pAllocator,
                          VkDevice *pDevice)
{
    /* Walk the chain to find the next layer's CreateDevice */
    VkLayerDeviceCreateInfo *chain_info = (VkLayerDeviceCreateInfo *)pCreateInfo->pNext;
    while (chain_info &&
           !(chain_info->sType == VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO &&
             chain_info->function == VK_LAYER_LINK_INFO)) {
        chain_info = (VkLayerDeviceCreateInfo *)chain_info->pNext;
    }

    if (!chain_info)
        return VK_ERROR_INITIALIZATION_FAILED;

    PFN_vkGetInstanceProcAddr get_instance = chain_info->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    PFN_vkGetDeviceProcAddr get_device = chain_info->u.pLayerInfo->pfnNextGetDeviceProcAddr;

    /* Advance chain for next layer */
    chain_info->u.pLayerInfo = chain_info->u.pLayerInfo->pNext;

    PFN_vkCreateDevice real_create = (PFN_vkCreateDevice)get_instance(NULL, "vkCreateDevice");
    if (!real_create)
        return VK_ERROR_INITIALIZATION_FAILED;

    VkResult result = real_create(physicalDevice, pCreateInfo, pAllocator, pDevice);
    if (result != VK_SUCCESS)
        return result;

    /* Capture real device-level functions */
    g_real_create_query_pool = (PFN_vkCreateQueryPool)get_device(*pDevice, "vkCreateQueryPool");
    g_real_get_device_proc_addr = get_device;

    fprintf(stderr, "[dzn_compat] Layer active — intercepting vkCreateQueryPool\n");
    return VK_SUCCESS;
}

static VkResult VKAPI_CALL
dzn_compat_vkCreateInstance(const VkInstanceCreateInfo *pCreateInfo,
                            const VkAllocationCallbacks *pAllocator,
                            VkInstance *pInstance)
{
    VkLayerInstanceCreateInfo *chain_info = (VkLayerInstanceCreateInfo *)pCreateInfo->pNext;
    while (chain_info &&
           !(chain_info->sType == VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO &&
             chain_info->function == VK_LAYER_LINK_INFO)) {
        chain_info = (VkLayerInstanceCreateInfo *)chain_info->pNext;
    }

    if (!chain_info)
        return VK_ERROR_INITIALIZATION_FAILED;

    PFN_vkGetInstanceProcAddr get_instance = chain_info->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    chain_info->u.pLayerInfo = chain_info->u.pLayerInfo->pNext;

    PFN_vkCreateInstance real_create = (PFN_vkCreateInstance)get_instance(NULL, "vkCreateInstance");
    if (!real_create)
        return VK_ERROR_INITIALIZATION_FAILED;

    VkResult result = real_create(pCreateInfo, pAllocator, pInstance);
    if (result != VK_SUCCESS)
        return result;

    g_real_get_instance_proc_addr = get_instance;
    g_real_create_device = (PFN_vkCreateDevice)get_instance(*pInstance, "vkCreateDevice");

    return VK_SUCCESS;
}

static PFN_vkVoidFunction VKAPI_CALL
dzn_compat_vkGetInstanceProcAddr(VkInstance instance, const char *pName)
{
    if (strcmp(pName, "vkCreateInstance") == 0)
        return (PFN_vkVoidFunction)dzn_compat_vkCreateInstance;
    if (strcmp(pName, "vkCreateDevice") == 0)
        return (PFN_vkVoidFunction)dzn_compat_vkCreateDevice;
    if (strcmp(pName, "vkGetDeviceProcAddr") == 0)
        return (PFN_vkVoidFunction)dzn_compat_vkGetDeviceProcAddr;
    if (strcmp(pName, "vkCreateQueryPool") == 0)
        return (PFN_vkVoidFunction)dzn_compat_vkCreateQueryPool;

    if (g_real_get_instance_proc_addr)
        return g_real_get_instance_proc_addr(instance, pName);

    return NULL;
}
