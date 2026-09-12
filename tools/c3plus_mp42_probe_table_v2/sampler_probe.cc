// Fixed-input test of the real GenerateSampleStates implementation.
// This binary builds plants but never advances a simulator or publishes LCM.
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <gflags/gflags.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/shape_specification.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram_builder.h>
#include "examples/sampling_c3/generate_samples.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/franka_sim_params.h"

DEFINE_string(config, "", "Exact generated controller parameter YAML");
DEFINE_string(dump, "", "New CSV path for deterministic candidate sequence");
DEFINE_int32(batches, 5, "Number of fixed-input sampler calls");

int main(int argc,char** argv) {
  try {
    gflags::ParseCommandLineFlags(&argc,&argv,true);
    const char* seed=std::getenv("SAMPLING_C3_SEED");
    if (!seed || std::string(seed)!="42") throw std::runtime_error("Literal seed 42 is required");
    if (FLAGS_config.empty() || FLAGS_dump.empty()) throw std::runtime_error("config/dump required");
    auto params=drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(FLAGS_config);
    auto sim=drake::yaml::LoadYamlFile<FrankaSimParams>(params.sim_params_file);
    if (params.base_names.size()!=1 || sim.q_init_objects.size()!=1 ||
        params.sampling_params.sampling_strategy!=SamplingStrategy::kRandomOnPerimeter)
      throw std::runtime_error("Probe requires one object and the unchanged perimeter sampler");
    using drake::multibody::MultibodyPlant;
    using drake::geometry::GeometryId;
    using drake::SortedPair;
    drake::systems::DiagramBuilder<double> builder;
    auto [plant,sg]=drake::multibody::AddMultibodyPlantSceneGraph(&builder,0.0);
    dairlib::AddLCSModelsToPlant(&plant,&sg,params.object_models,false,false);
    plant.Finalize();
    if (plant.num_positions()!=10 || plant.num_velocities()!=9 || plant.num_actuators()!=3)
      throw std::runtime_error("Unexpected LCS plant state dimensions");
    auto ad=drake::systems::System<double>::ToAutoDiffXd(plant);
    auto adctx=ad->CreateDefaultContext();
    auto diagram=builder.Build();auto rootctx=diagram->CreateDefaultContext();
    auto& ctx=diagram->GetMutableSubsystemContext(plant,rootctx.get());
    auto object=plant.GetCollisionGeometriesForBody(plant.GetBodyByName(params.base_names[0]));
    if (object.size()!=5) throw std::runtime_error("Expected two convex pieces followed by three supports");
    const auto& inspector=sg.model_inspector();
    for(int i=0;i<5;++i) {
      const auto& shape=inspector.GetShape(object[i]);
      if(i<2 && dynamic_cast<const drake::geometry::Convex*>(&shape)==nullptr)
        throw std::runtime_error("Object collision piece did not parse as Convex");
      if(i>=2 && dynamic_cast<const drake::geometry::Sphere*>(&shape)==nullptr)
        throw std::runtime_error("Last three collision geometries are not support spheres");
    }
    auto ee=plant.GetCollisionGeometriesForBody(plant.GetBodyByName("end_effector_simple")).at(0);
    auto ground=plant.GetCollisionGeometriesForBody(plant.GetBodyByName("ground")).at(0);
    const auto* table_box=dynamic_cast<const drake::geometry::Box*>(&inspector.GetShape(ground));
    if (!table_box || std::abs(table_box->width()-0.8)>1e-9 ||
        std::abs(table_box->depth()-1.523)>1e-9 || std::abs(table_box->height()-0.91)>1e-9)
      throw std::runtime_error("Actual parsed LCS table is not canonical 0.8 x 1.523 x 0.91");
    const auto table_origin=inspector.GetPoseInFrame(ground).translation();
    if ((table_origin-Eigen::Vector3d(0.35,0,-0.455)).norm()>1e-9)
      throw std::runtime_error("Actual parsed table origin is not canonical");
    std::cout<<"[TABLE-PROBE] canonical box and local pose verified in parsed LCS plant\n";
    std::vector<std::vector<SortedPair<GeometryId>>> groups(3);
    groups[0].emplace_back(ee,ground);
    groups[1].emplace_back(ee,object[0]);groups[1].emplace_back(ee,object[1]);
    for(int i=2;i<5;++i)groups[2].emplace_back(object[i],ground);
    for(const auto& resolve:params.sampling_c3_options.resolve_contacts_to_lists) {
      for(size_t i=0;i<groups.size();++i)
        if(groups[i].size()<static_cast<size_t>(resolve.at(i))) throw std::runtime_error("Contact resolution capacity mismatch");
    }
    MultibodyPlant<double> arm(0.0);
    const auto arm_id=dairlib::AddXarm6ToPlant(&arm,nullptr,true,true,false);arm.Finalize();
    auto armctx=arm.CreateDefaultContext();arm.SetPositions(armctx.get(),arm_id,sim.q_init_franka);
    const auto tip=arm.EvalBodyPoseInWorld(*armctx,arm.GetBodyByName(dairlib::kEndEffectorName)).translation();
    Eigen::VectorXd x=Eigen::VectorXd::Zero(19);x.head<3>()=tip;x.segment<7>(3)=sim.q_init_objects.at(0);
    Eigen::MatrixXd unsuccessful=Eigen::MatrixXd::Zero(params.sampling_params.N_unsuccessful_sample_buffer,10);
    std::ofstream out(FLAGS_dump);
    if(!out)throw std::runtime_error("Cannot open output CSV");
    out<<"batch,c3_mode,candidate,x,y,z\n"<<std::hexfloat;
    for(int b=0;b<FLAGS_batches;++b) {
      bool c3_mode=(b%2)!=0;
      auto samples=dairlib::systems::GenerateSampleStates(10,9,3,x,c3_mode,
        params.sampling_params,params.sampling_c3_options,plant,&ctx,*ad,adctx.get(),
        groups,{},{},{},{},{},{false},unsuccessful);
      for(size_t i=0;i<samples.size();++i) {
        if(!samples[i].allFinite())throw std::runtime_error("Nonfinite sampler output");
        out<<b<<','<<c3_mode<<','<<i<<','<<samples[i][0]<<','<<samples[i][1]<<','<<samples[i][2]<<'\n';
      }
    }
    out.flush();if(!out)throw std::runtime_error("Output write failed");
    std::cout<<"[PROBE] parsed shape roles, verified contact capacity, sampled fixed input; no simulator advanced\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<"[PROBE-FAIL] "<<e.what()<<'\n';return 2;}
}
