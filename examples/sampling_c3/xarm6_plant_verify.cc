// Baseline verification utility (read-only): builds the xArm6 plant exactly
// as franka_sim does (AddXarm6ToPlant) and prints the frozen-baseline plant
// facts for results/final_oim_c3plus_comparison/provenance/.
#include <iostream>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/systems/framework/diagram_builder.h"

#include "examples/sampling_c3/sampling_c3_utils.h"

int main() {
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0001);
  dairlib::AddXarm6ToPlant(&plant, &scene_graph, true, true, false);
  plant.Finalize();

  std::cout << "[VERIFY] num_actuators=" << plant.num_actuators() << "\n";
  for (drake::multibody::JointActuatorIndex i :
       plant.GetJointActuatorIndices()) {
    const auto& a = plant.get_joint_actuator(i);
    std::cout << "[VERIFY] actuator " << a.name()
              << " effort=" << a.effort_limit()
              << " gear_ratio=" << a.default_gear_ratio()
              << " rotor_inertia=" << a.default_rotor_inertia() << "\n";
  }
  std::cout << "[VERIFY] has_joint6="
            << plant.HasJointNamed("xarm6_joint6") << "\n";
  for (const char* jn : {"xarm6_joint1", "xarm6_joint2", "xarm6_joint3",
                         "xarm6_joint4", "xarm6_joint5"}) {
    const auto& j =
        plant.GetJointByName<drake::multibody::RevoluteJoint>(jn);
    std::cout << "[VERIFY] joint " << jn
              << " vel_limit=" << j.velocity_upper_limits()(0)
              << " damping=" << j.default_damping() << "\n";
  }
  int springs = 0;
  for (drake::multibody::ForceElementIndex i(0);
       i < plant.num_force_elements(); ++i) {
    const auto* s = dynamic_cast<const drake::multibody::RevoluteSpring<double>*>(
        &plant.get_force_element(i));
    if (s) {
      springs++;
      std::cout << "[VERIFY] revolute_spring joint="
                << s->joint().name()
                << " k=" << s->default_stiffness()
                << " ref=" << s->default_nominal_angle() << "\n";
    }
  }
  std::cout << "[VERIFY] num_revolute_springs=" << springs << std::endl;
  return 0;
}
