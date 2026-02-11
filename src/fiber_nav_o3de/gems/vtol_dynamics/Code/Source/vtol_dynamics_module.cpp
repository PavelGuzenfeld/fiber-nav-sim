#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "multicopter_motor_component.h"
#include "aerodynamics_component.h"
#include "vtol_transition_component.h"
#include "sensor_collector_component.h"

namespace vtol_dynamics
{
    class VTOLDynamicsModule : public AZ::Module
    {
    public:
        AZ_RTTI(VTOLDynamicsModule, "{D4E5F6A7-B8C9-0D1E-2F3A-4B5C6D7E8F9A}", AZ::Module);
        AZ_CLASS_ALLOCATOR(VTOLDynamicsModule, AZ::SystemAllocator);

        VTOLDynamicsModule()
        {
            m_descriptors.insert(m_descriptors.end(), {
                MulticopterMotorComponent::CreateDescriptor(),
                AerodynamicsComponent::CreateDescriptor(),
                VTOLTransitionComponent::CreateDescriptor(),
                SensorCollectorComponent::CreateDescriptor(),
            });
        }
    };

} // namespace vtol_dynamics

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), vtol_dynamics::VTOLDynamicsModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_VTOLDynamics, vtol_dynamics::VTOLDynamicsModule)
#endif
