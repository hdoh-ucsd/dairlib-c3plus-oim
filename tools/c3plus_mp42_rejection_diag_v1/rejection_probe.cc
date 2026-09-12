// Read-only fixed-input rejection census using the production geometry helpers.
// This is not a replacement sampler and does not run C3 or a simulator.
// This binary builds plants but never advances a simulator or publishes LCM.
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
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
DEFINE_int32(draws, 10000, "Fixed number of raw draws in this diagnostic only");
DEFINE_string(summary, "", "New JSON output path");

int main(int argc,char** argv) {
  try {
    gflags::ParseCommandLineFlags(&argc,&argv,true);
    const char* seed=std::getenv("SAMPLING_C3_SEED");
    if (!seed || std::string(seed)!="42") throw std::runtime_error("Literal seed 42 is required");
    if (FLAGS_config.empty() || FLAGS_dump.empty() || FLAGS_summary.empty() || FLAGS_draws < 1 || FLAGS_draws > 10000) throw std::runtime_error("config/dump required");
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

    using namespace dairlib::systems;
    const auto& sp=params.sampling_params;
    const auto& co=params.sampling_c3_options;
    dairlib::UpdateContext(10,9,3,plant,&ctx,*ad,adctx.get(),x);
    const double radius=GetEERadiusFromPlant(plant,ctx,groups);
    std::ofstream out(FLAGS_dump);
    if(!out)throw std::runtime_error("Cannot open new diagnostic CSV");
    out<<std::setprecision(17);
    out<<"draw,raw_x,raw_y,raw_z,closest_piece,raw_pair_gap_m,projected_x,projected_y,projected_z,post_closest_piece,post_pair_gap_m,height_error_m,workspace_ok,bad_spot_ok,clearance_ok,height_ok,stage\n";
    std::map<std::string,long long> counts;
    for(const std::string key : {"no_initial_penetration","nonfinite_projection","clearance_rejected","height_rejected","workspace_rejected","failed_buffer_rejected","accepted"})counts[key]=0;
    long long projected_count=0,height_failed_independent=0,workspace_failed_independent=0,clearance_failed_independent=0;
    double min_abs_dz=std::numeric_limits<double>::infinity();
    double max_abs_dz=0.0;
    double first_accepted_draw=-1;
    auto pair_gap=[&](int i) {
      dairlib::multibody::GeomGeomCollider collider(plant,groups.at(1).at(i));
      const auto pair=collider.EvalPolytope(ctx,co.num_friction_directions.value());
      return pair.first;
    };
    for(int draw=0;draw<FLAGS_draws;++draw) {
      // Same grid, RNG helper, state transform, penetration test and projection
      // as production PerimeterSampling. No extra random numbers are drawn.
      const double gx=RandomUniform(sp.grid_x_limits[0],sp.grid_x_limits[1]);
      const double gy=RandomUniform(sp.grid_y_limits[0],sp.grid_y_limits[1]);
      Eigen::VectorXd candidate=x;
      Eigen::Quaterniond q(x(3),x(4),x(5),x(6));
      candidate.head<3>()=q*Eigen::Vector3d(gx,gy,0)+x.segment<3>(7);
      candidate[2]=sp.sampling_height;
      const Eigen::Vector3d raw=candidate.head<3>();
      int closest=-1;
      const bool penetrating=IsSampleWithinDistanceOfSurface(10,9,3,0.,candidate,
        plant,&ctx,*ad,adctx.get(),groups,co,closest);
      const double raw_gap=pair_gap(closest);
      if(!std::isfinite(raw_gap))throw std::runtime_error("Nonfinite initial signed distance");
      out<<draw<<','<<raw.x()<<','<<raw.y()<<','<<raw.z()<<','<<closest<<','<<raw_gap;
      if(!penetrating) {
        ++counts["no_initial_penetration"];
        for(int col=0;col<10;++col)out<<',';
        out<<",no_initial_penetration\n";
        continue;
      }
      Eigen::VectorXd projected=ProjectSampleOutsideObject(candidate,closest,sp,plant,ctx,groups);
      if(!projected.allFinite()) {
        ++counts["nonfinite_projection"];
        for(int col=0;col<10;++col)out<<',';
        out<<",nonfinite_projection\n";
        continue;
      }
      ++projected_count;
      int post_closest=-1;
      const bool too_close=IsSampleWithinDistanceOfSurface(10,9,3,sp.sample_projection_clearance,projected,
        plant,&ctx,*ad,adctx.get(),groups,co,post_closest);
      const double post_gap=pair_gap(post_closest);
      if(!std::isfinite(post_gap))throw std::runtime_error("Nonfinite projected signed distance");
      const double dz=projected[2]-sp.sampling_height;
      const bool height_ok=(projected[2]>=sp.sampling_height-0.001 && projected[2]<=sp.sampling_height+0.001);
      const bool workspace_ok=IsSampleInWorkspace(projected,co);
      const bool avoids=SampleAvoidsBadSpots(projected,sp,unsuccessful);
      min_abs_dz=std::min(min_abs_dz,std::abs(dz));max_abs_dz=std::max(max_abs_dz,std::abs(dz));
      if(!height_ok)++height_failed_independent;
      if(!workspace_ok)++workspace_failed_independent;
      if(too_close)++clearance_failed_independent;
      std::string stage;
      // Terminal rejection order matches the production perimeter path.
      if(too_close)stage="clearance_rejected";
      else if(!height_ok)stage="height_rejected";
      else if(sp.filter_samples_for_safety && !workspace_ok)stage="workspace_rejected";
      else if(sp.avoid_choosing_unsuccessful_samples && !avoids)stage="failed_buffer_rejected";
      else {stage="accepted";if(first_accepted_draw<0)first_accepted_draw=draw;}
      ++counts[stage];
      out<<','<<projected[0]<<','<<projected[1]<<','<<projected[2]<<','<<post_closest<<','<<post_gap<<','<<dz
         <<','<<workspace_ok<<','<<avoids<<','<<(!too_close)<<','<<height_ok<<','<<stage<<'\n';
    }
    dairlib::UpdateContext(10,9,3,plant,&ctx,*ad,adctx.get(),x);
    out.flush();if(!out)throw std::runtime_error("Diagnostic CSV write failed");
    std::ofstream summary(FLAGS_summary);
    if(!summary)throw std::runtime_error("Cannot open diagnostic summary");
    summary<<std::setprecision(17)
      <<"{\n  \"scope\": \"fixed-input raw-draw rejection census, not a controller run or a completeness proof\",\n"
      <<"  \"seed\": 42, \"draws\": "<<FLAGS_draws<<", \"simulations_run\": 0,\n"
      <<"  \"sampling_height_world_m\": "<<sp.sampling_height<<",\n"
      <<"  \"execution_z_height_world_m\": "<<sp.z_height<<",\n"
      <<"  \"sample_projection_clearance_m\": "<<sp.sample_projection_clearance<<",\n"
      <<"  \"pusher_radius_m\": "<<radius<<",\n"
      <<"  \"height_tolerance_m\": 0.001,\n"
      <<"  \"object_xyz_m\": ["<<x[7]<<','<<x[8]<<','<<x[9]<<"],\n"
      <<"  \"projected_count\": "<<projected_count<<",\n"
      <<"  \"first_accepted_draw_zero_based\": "<<first_accepted_draw<<",\n"
      <<"  \"independent_height_rejections\": "<<height_failed_independent<<",\n"
      <<"  \"independent_workspace_rejections\": "<<workspace_failed_independent<<",\n"
      <<"  \"independent_clearance_rejections\": "<<clearance_failed_independent<<",\n"
      <<"  \"min_abs_projection_height_error_m\": ";
    if(std::isfinite(min_abs_dz))summary<<min_abs_dz;else summary<<"null";
    summary<<",\n  \"max_abs_projection_height_error_m\": "<<max_abs_dz<<",\n  \"terminal_counts\": {";
    bool first=true;for(const auto& kv:counts){if(!first)summary<<',';first=false;summary<<'\n'<<"    \""<<kv.first<<"\": "<<kv.second;}
    summary<<"\n  }\n}\n";summary.flush();if(!summary)throw std::runtime_error("Summary write failed");
    std::cout<<"[DIAGNOSTIC ONLY] raw draws="<<FLAGS_draws<<" acceptable="<<counts["accepted"]
             <<"; no simulator advanced; no C3 solve; baseline source unchanged\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<"[DIAG-ERROR] "<<e.what()<<'\n';return 2;}
}
