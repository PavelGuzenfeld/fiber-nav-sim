// Copyright 2026 Pavel Guzenfeld — All rights reserved.
// PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
// Version: 0.0.1

#include <fiber_nav_sensors/wrench_output.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/entity_wrench.pb.h>

namespace fiber_nav_sensors {

class GazeboWrenchOutput final : public WrenchOutput {
public:
    bool initialize(std::string const& world_name,
                    std::string const& /*model_name*/) override {
        wrench_topic_ = "/world/" + world_name + "/wrench";
        gz_pub_ = gz_node_.Advertise<gz::msgs::EntityWrench>(wrench_topic_);
        return true;
    }

    void set_entity_id(int entity_id) override {
        entity_id_ = entity_id;
    }

    void publish(double fx, double fy, double fz,
                 double tx, double ty, double tz) override {
        gz::msgs::EntityWrench msg;
        auto* entity = msg.mutable_entity();
        entity->set_id(entity_id_);
        entity->set_type(gz::msgs::Entity::MODEL);
        auto* wrench = msg.mutable_wrench();
        wrench->mutable_force()->set_x(fx);
        wrench->mutable_force()->set_y(fy);
        wrench->mutable_force()->set_z(fz);
        wrench->mutable_torque()->set_x(tx);
        wrench->mutable_torque()->set_y(ty);
        wrench->mutable_torque()->set_z(tz);
        gz_pub_.Publish(msg);
    }

private:
    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher gz_pub_;
    std::string wrench_topic_;
    int entity_id_{0};
};

std::unique_ptr<WrenchOutput> create_wrench_output() {
    return std::make_unique<GazeboWrenchOutput>();
}

}  // namespace fiber_nav_sensors
