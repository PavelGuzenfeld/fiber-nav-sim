#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "px4_mavlink_bridge.h"
#include "px4_motor_output.h"

namespace px4_bridge
{
    class PX4BridgeModule : public AZ::Module
    {
    public:
        AZ_RTTI(PX4BridgeModule, "{C3D4E5F6-7890-1234-ABCD-EF567890ABCD}", AZ::Module);
        AZ_CLASS_ALLOCATOR(PX4BridgeModule, AZ::SystemAllocator);

        PX4BridgeModule()
        {
            m_descriptors.insert(m_descriptors.end(), {
                PX4MAVLinkBridge::CreateDescriptor(),
                PX4MotorOutput::CreateDescriptor(),
            });
        }

        // Auto-activate PX4MAVLinkBridge as a system component.
        // System components activate automatically with the game —
        // no need to inject into level prefabs.
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return {
                azrtti_typeid<PX4MAVLinkBridge>(),
            };
        }
    };

} // namespace px4_bridge

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), px4_bridge::PX4BridgeModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_PX4Bridge, px4_bridge::PX4BridgeModule)
#endif
