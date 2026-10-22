// Copyright 2026 Adaptive Target Tracking Testbed contributors.
// Licensed under the Apache License, Version 2.0.
// A single motor owner, with selectable stock PID instrumentation / DART SERVO.
#include "servo_math.hh"

#include <gz/common/Console.hh>
#include <gz/math/PID.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_simulation
{
namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::string RequiredString(const std::shared_ptr<const sdf::Element> &element,
                           const std::string &key)
{
  if (!element->HasElement(key))
    throw std::invalid_argument("ArmActuator missing " + key);
  const auto value = element->Get<std::string>(key);
  if (value.empty())
    throw std::invalid_argument("ArmActuator empty " + key);
  return value;
}

double RequiredNumber(const std::shared_ptr<const sdf::Element> &element,
                      const std::string &key)
{
  if (!element->HasElement(key))
    throw std::invalid_argument("ArmActuator missing " + key);
  const auto value = element->Get<double>(key);
  if (!std::isfinite(value))
    throw std::invalid_argument("ArmActuator nonfinite " + key);
  return value;
}

std::string CsvString(const std::string &input)
{
  std::string output = "\"";
  for (char value : input)
    output += value == '"' ? "\"\"" : std::string(1, value);
  return output + "\"";
}

struct JointState
{
  std::string name;
  std::string topic;
  gz::sim::Joint joint;
  gz::math::PID pid;
  double initial = 0.0;
  double target = 0.0;
  double incoming = 0.0;
  bool pending = false;
  bool resolved = false;
  bool validStep = false;
  double lower = kNaN;
  double upper = kNaN;
  double speed = kNaN;
  double effort = kNaN;
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double iMin = 0.0;
  double iMax = 0.0;
  double offset = 0.0;
  double preQ = kNaN;
  double preV = kNaN;
  double error = kNaN;
  double velocity = kNaN;
  double previousDemand = 0.0;
  double demandAcceleration = kNaN;
  double pTerm = kNaN;
  double iTerm = kNaN;
  double dTerm = kNaN;
  double requested = kNaN;
  double commanded = kNaN;
  bool saturation = false;
  std::uint64_t invalidCommands = 0;
};
}  // namespace

class ArmActuator final : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate,
                          public gz::sim::ISystemReset
{
 public:
  ~ArmActuator() override
  {
    for (const auto &state : this->joints)
      this->node.Unsubscribe(state->topic);
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(entity);
    if (!this->model.Valid(ecm))
      throw std::invalid_argument("ArmActuator must be attached to a model");
    this->backend = RequiredString(sdf, "backend");
    if (this->backend != "instrumented_pid" && this->backend != "velocity_servo")
      throw std::invalid_argument("ArmActuator unsupported backend");
    if (this->backend == "velocity_servo")
    {
      this->timeConstant = RequiredNumber(sdf, "position_time_constant");
      this->acceleration = RequiredNumber(sdf, "acceleration_limit");
      if (this->timeConstant <= 0.0 || this->acceleration <= 0.0)
        throw std::invalid_argument("ArmActuator invalid demand governor");
    }
    std::set<std::string> names;
    std::set<std::string> topics;
    for (auto element = sdf->FindElement("joint"); element;
         element = element->GetNextElement("joint"))
    {
      auto state = std::make_unique<JointState>();
      state->name = element->HasAttribute("name") ?
          element->Get<std::string>("name") : RequiredString(element, "name");
      state->topic = RequiredString(element, "topic");
      if (state->name.empty() || !names.insert(state->name).second ||
          !topics.insert(state->topic).second)
        throw std::invalid_argument("ArmActuator requires unique joint names/topics");
      state->initial = RequiredNumber(element, "initial_position");
      state->target = state->initial;
      state->p = RequiredNumber(element, "p_gain");
      state->i = RequiredNumber(element, "i_gain");
      state->d = RequiredNumber(element, "d_gain");
      state->iMin = RequiredNumber(element, "i_min");
      state->iMax = RequiredNumber(element, "i_max");
      state->offset = RequiredNumber(element, "cmd_offset");
      if (state->p <= 0.0 || state->i < 0.0 || state->d < 0.0 ||
          state->iMin > state->iMax)
        throw std::invalid_argument("ArmActuator invalid PID parameters");
      this->joints.push_back(std::move(state));
    }
    if (this->joints.empty())
      throw std::invalid_argument("ArmActuator requires at least one joint");
    if (sdf->HasElement("trace_directory"))
    {
      this->traceDirectory = RequiredString(sdf, "trace_directory");
      if (!this->traceDirectory.is_absolute())
        throw std::invalid_argument("ArmActuator trace_directory must be absolute");
      std::filesystem::create_directories(this->traceDirectory);
      this->OpenTrace();
    }
    for (std::size_t index = 0; index < this->joints.size(); ++index)
    {
      const std::function<void(const gz::msgs::Double &)> callback =
          [this, index](const gz::msgs::Double &message)
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        auto &state = *this->joints[index];
        if (!std::isfinite(message.data()))
        {
          ++state.invalidCommands;
          return;
        }
        state.incoming = message.data();
        state.pending = true;
      };
      if (!this->node.Subscribe(this->joints[index]->topic, callback))
        throw std::invalid_argument("ArmActuator could not subscribe to joint target");
    }
    this->configured = true;
    gzmsg << "ArmActuator backend=" << this->backend << " joints="
          << this->joints.size() << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused)
      return;
    if (this->hasTime && info.simTime < this->lastTime)
      this->Reset(info, ecm);
    const double dt = std::chrono::duration<double>(info.dt).count();
    this->lastTime = info.simTime;
    this->hasTime = true;
    this->stepPending = true;
    for (auto &entry : this->joints)
    {
      auto &state = *entry;
      state.validStep = false;
      state.preQ = state.preV = state.error = state.velocity = kNaN;
      state.demandAcceleration = kNaN;
      state.pTerm = state.iTerm = state.dTerm = kNaN;
      state.requested = state.commanded = kNaN;
      state.saturation = false;
      if (!state.resolved && !this->Resolve(state, ecm))
        continue;
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (state.pending)
        {
          if (state.incoming >= state.lower && state.incoming <= state.upper)
            state.target = state.incoming;
          else
            ++state.invalidCommands;
          state.pending = false;
        }
      }
      const auto position = state.joint.Position(ecm);
      const auto velocity = state.joint.Velocity(ecm);
      if (position && position->size() == 1)
        state.preQ = position->front();
      if (velocity && velocity->size() == 1)
        state.preV = velocity->front();
      if (!std::isfinite(state.preQ) || !std::isfinite(state.preV) ||
          !std::isfinite(dt) || dt <= 0.0)
      {
        if (this->backend == "velocity_servo")
        {
          ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
              state.joint.Entity(), {0.0});
          state.velocity = state.previousDemand = 0.0;
        }
        continue;
      }
      state.error = state.preQ - state.target;
      if (this->backend == "velocity_servo")
      {
        state.velocity = GovernedServoVelocity(state.preQ, state.target,
            state.lower, state.upper, state.speed, dt, state.previousDemand,
            this->timeConstant, this->acceleration);
        state.demandAcceleration = (state.velocity - state.previousDemand) / dt;
        state.previousDemand = state.velocity;
        // JointVelocityCmd becomes an effort-limited DART SERVO constraint.
        // It never uses JointVelocityReset or modifies physical measured state.
        ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
            state.joint.Entity(), {state.velocity});
        state.saturation = std::abs((state.target - state.preQ) / this->timeConstant -
                                    state.velocity) > 1e-12;
      }
      else
      {
        state.commanded = state.pid.Update(state.error, info.dt);
        double pe, ie, de;
        state.pid.Errors(pe, ie, de);
        state.pTerm = -state.p * pe;
        state.iTerm = -ie;
        state.dTerm = -state.d * de;
        state.requested = state.offset + state.pTerm + state.iTerm + state.dTerm;
        state.saturation = state.requested < -state.effort ||
                           state.requested > state.effort;
        state.joint.SetForce(ecm, {state.commanded});
      }
      state.validStep = true;
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused || !this->stepPending)
      return;
    this->stepPending = false;
    if (!this->trace.is_open())
      return;
    const double time = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    for (const auto &entry : this->joints)
    {
      const auto &state = *entry;
      double q = kNaN, v = kNaN;
      if (state.resolved)
      {
        const auto position = state.joint.Position(ecm);
        const auto velocity = state.joint.Velocity(ecm);
        if (position && position->size() == 1) q = position->front();
        if (velocity && velocity->size() == 1) v = velocity->front();
      }
      std::uint64_t invalid;
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        invalid = state.invalidCommands;
      }
      this->trace << time << ',' << info.iterations << ',' << dt << ','
          << CsvString(state.name) << ',' << state.preQ << ',' << state.preV << ','
          << q << ',' << v << ',' << state.target << ',' << state.velocity << ','
          << state.error << ',' << state.pTerm << ',' << state.iTerm << ','
          << state.dTerm << ',' << state.requested << ',' << state.commanded << ','
          << (state.validStep && this->backend == "instrumented_pid") << ','
          << state.saturation << ',' << this->epoch << ',' << time - dt << ','
          << invalid << ',' << state.validStep << ',' << state.demandAcceleration << '\n';
    }
    this->trace.flush();
    if (!this->trace)
      throw std::runtime_error("ArmActuator could not write complete physics trace");
  }

  void Reset(const gz::sim::UpdateInfo &,
             gz::sim::EntityComponentManager &ecm) override
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    for (auto &state : this->joints)
    {
      state->pid.Reset();
      state->target = state->initial;
      state->pending = false;
      state->validStep = false;
      state->previousDemand = 0.0;
      if (state->resolved && this->backend == "velocity_servo")
        ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(state->joint.Entity(), {0.0});
      if (state->resolved && this->backend == "instrumented_pid")
        state->joint.SetForce(ecm, {0.0});
      state->resolved = false;
    }
    this->hasTime = false;
    this->stepPending = false;
    ++this->epoch;
    if (!this->traceDirectory.empty())
      this->OpenTrace();
  }

 private:
  bool Resolve(JointState &state, gz::sim::EntityComponentManager &ecm)
  {
    const auto entity = this->model.JointByName(ecm, state.name);
    if (entity == gz::sim::kNullEntity)
      return false;
    state.joint.ResetEntity(entity);
    const auto axes = state.joint.Axis(ecm);
    if (!axes || axes->size() != 1)
      throw std::invalid_argument("ArmActuator requires one authoritative joint axis");
    const auto &axis = axes->front();
    state.lower = axis.Lower();
    state.upper = axis.Upper();
    state.speed = axis.MaxVelocity();
    state.effort = axis.Effort();
    if (!std::isfinite(state.lower) || !std::isfinite(state.upper) ||
        !std::isfinite(state.speed) || !std::isfinite(state.effort) ||
        state.lower >= state.upper || state.speed <= 0.0 || state.effort <= 0.0 ||
        state.initial < state.lower || state.initial > state.upper ||
        std::abs(state.offset) > state.effort)
      throw std::invalid_argument("ArmActuator invalid authoritative limits/posture");
    if (!this->ownedEntities.count(entity) &&
        (ecm.Component<gz::sim::components::JointForceCmd>(entity) ||
         ecm.Component<gz::sim::components::JointVelocityCmd>(entity)))
      throw std::invalid_argument("ArmActuator found an existing motor command writer");
    this->ownedEntities.insert(entity);
    state.pid.Init(state.p, state.i, state.d, state.iMax, state.iMin,
                   state.effort, -state.effort, state.offset);
    state.joint.EnablePositionCheck(ecm);
    state.joint.EnableVelocityCheck(ecm);
    state.resolved = true;
    return true;
  }

  void OpenTrace()
  {
    this->trace.close();
    const auto filename = this->epoch == 0 ? std::string("actuator.csv") :
        "actuator_epoch_" + std::to_string(this->epoch) + ".csv";
    const auto path = this->traceDirectory / filename;
    if (std::filesystem::exists(path))
      throw std::invalid_argument("ArmActuator refuses to overwrite " + path.string());
    this->trace.open(path);
    if (!this->trace)
      throw std::runtime_error("ArmActuator could not create trace " + path.string());
    this->trace << std::setprecision(17)
        << "time,iteration,dt,name,q_pre,v_pre,q_post,v_post,position_reference,"
        << "target_velocity,error,pid_p,pid_i,pid_d,requested_effort,commanded_effort,"
        << "effort_available,saturation,epoch,pre_time,invalid_commands,valid_step,demand_acceleration\n";
    this->trace.flush();
  }

  gz::sim::Model model;
  std::string backend;
  double timeConstant = 0.0;
  double acceleration = 0.0;
  std::vector<std::unique_ptr<JointState>> joints;
  std::set<gz::sim::Entity> ownedEntities;
  std::mutex mutex;
  std::filesystem::path traceDirectory;
  std::ofstream trace;
  bool configured = false;
  bool hasTime = false;
  bool stepPending = false;
  std::chrono::steady_clock::duration lastTime{};
  std::uint64_t epoch = 0;
  // Destroy transport before states captured by subscriber callbacks.
  gz::transport::Node node;
};
}  // namespace robot_simulation

GZ_ADD_PLUGIN(robot_simulation::ArmActuator, gz::sim::System,
              robot_simulation::ArmActuator::ISystemConfigure,
              robot_simulation::ArmActuator::ISystemPreUpdate,
              robot_simulation::ArmActuator::ISystemPostUpdate,
              robot_simulation::ArmActuator::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(robot_simulation::ArmActuator, "robot_simulation::ArmActuator")
