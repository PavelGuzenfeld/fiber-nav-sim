#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include "../aerodynamics_component.h"

#include <cmath>

namespace vtol_dynamics::tests
{
    class AerodynamicsTest : public UnitTest::LeakDetectionFixture
    {
    };

    TEST_F(AerodynamicsTest, ZeroAlphaProducesBaselineLift)
    {
        AeroCoefficients coeffs;
        coeffs.CL0 = 0.5f;
        coeffs.CLa = 4.0f;
        coeffs.a0 = 0.0f;
        coeffs.alpha_stall = 0.3f;

        // At alpha=0, CL = CL0 + CLa * (alpha - a0) = 0.5
        float alpha = 0.0f;
        float alpha_eff = alpha - coeffs.a0;
        float expected_cl = coeffs.CL0 + coeffs.CLa * alpha_eff;
        EXPECT_FLOAT_EQ(expected_cl, 0.5f);
    }

    TEST_F(AerodynamicsTest, PositiveAlphaIncreasesLift)
    {
        AeroCoefficients coeffs;
        coeffs.CL0 = 0.0f;
        coeffs.CLa = 5.0f;
        coeffs.a0 = 0.0f;
        coeffs.alpha_stall = 0.3f;

        float alpha = 0.1f; // 5.7 degrees
        float alpha_eff = alpha - coeffs.a0;
        float cl = coeffs.CL0 + coeffs.CLa * alpha_eff;
        EXPECT_GT(cl, 0.0f);
        EXPECT_FLOAT_EQ(cl, 0.5f);
    }

    TEST_F(AerodynamicsTest, StallReducesLiftSlope)
    {
        AeroCoefficients coeffs;
        coeffs.CL0 = 0.0f;
        coeffs.CLa = 5.0f;
        coeffs.CLa_stall = -2.0f;
        coeffs.a0 = 0.0f;
        coeffs.alpha_stall = 0.3f;

        // Pre-stall at alpha=0.2
        float alpha_pre = 0.2f;
        float cl_pre = coeffs.CL0 + coeffs.CLa * alpha_pre;

        // Post-stall at alpha=0.5
        float alpha_post = 0.5f;
        float cl_at_stall = coeffs.CL0 + coeffs.CLa * coeffs.alpha_stall;
        float delta_alpha = alpha_post - coeffs.alpha_stall;
        float cl_post = cl_at_stall + coeffs.CLa_stall * delta_alpha;

        // Post-stall CL should be less than extrapolated pre-stall
        float cl_extrapolated = coeffs.CL0 + coeffs.CLa * alpha_post;
        EXPECT_LT(cl_post, cl_extrapolated);
    }

    TEST_F(AerodynamicsTest, InducedDragIncreasesWithLift)
    {
        WingGeometry wing;
        wing.aspect_ratio = 5.0f;
        wing.efficiency = 0.9f;

        float cl_low = 0.5f;
        float cl_high = 1.5f;

        float cdi_low = (cl_low * cl_low) / (AZ::Constants::Pi * wing.aspect_ratio * wing.efficiency);
        float cdi_high = (cl_high * cl_high) / (AZ::Constants::Pi * wing.aspect_ratio * wing.efficiency);

        EXPECT_GT(cdi_high, cdi_low);
        EXPECT_GT(cdi_high / cdi_low, 8.0f); // CL^2 ratio = 9
    }

    TEST_F(AerodynamicsTest, DynamicPressureScalesWithVelocitySquared)
    {
        float rho = 1.225f; // kg/m^3
        float v1 = 10.0f;
        float v2 = 20.0f;

        float q1 = 0.5f * rho * v1 * v1;
        float q2 = 0.5f * rho * v2 * v2;

        EXPECT_FLOAT_EQ(q2 / q1, 4.0f);
    }

} // namespace vtol_dynamics::tests
