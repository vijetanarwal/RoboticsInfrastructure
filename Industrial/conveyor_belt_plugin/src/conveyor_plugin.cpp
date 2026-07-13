#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>

#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <unordered_map>
#include <chrono>
#include <cmath>

namespace box_mover
{

class BoxMoverPlugin:
  public gz::sim::System,
  public gz::sim::ISystemPreUpdate
{
public:

  void PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm) override
  {
    _ecm.Each<
      gz::sim::components::Model,
      gz::sim::components::Name,
      gz::sim::components::Pose>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Model *,
          const gz::sim::components::Name *_name,
          const gz::sim::components::Pose *_pose)->bool
      {
        std::string name = _name->Data();

        if (name.find("box_") == std::string::npos)
          return true;

        auto pos = _pose->Data().Pos();
        auto rot = _pose->Data().Rot();

        double yaw = rot.Yaw();

        // Eliminar salchichas que han caído al suelo
        if (pos.Z() < 0.1)
        {
          _ecm.RequestRemoveEntity(_entity);
          return true;
        }

        bool inside =
          pos.X() > -0.25 && pos.X() < 0.25 &&
          pos.Y() > -0.6  && pos.Y() < 0.6 &&
          pos.Z() > 0.6 && pos.Z() < 0.90;

        auto links = _ecm.ChildrenByComponents(
          _entity, gz::sim::components::Link());

        for (auto link : links)
        {
          if (inside)
          {
            const double STOP_Y = -0.55;
            if (pos.Y() > STOP_Y)
            {
                //SetVelocity(_ecm, link, yaw);
            }
          }
        }

        return true;
      });
  }

  void SetVelocity(
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::Entity entity,
      double yaw)
  {
      double vx_world = 0.0;
      double vy_world = -0.2;

      // Convertir velocidad del mundo al sistema local de la salchicha
      double vx_local =
          std::cos(yaw) * vx_world +
          std::sin(yaw) * vy_world;

      double vy_local =
        -std::sin(yaw) * vx_world +
          std::cos(yaw) * vy_world;

      gz::math::Vector3d vel(
          vx_local,
          vy_local,
          0.0
      );

      auto cmdComp =
          _ecm.Component<gz::sim::components::LinearVelocityCmd>(entity);

      if (!cmdComp)
      {
          _ecm.CreateComponent(
              entity,
              gz::sim::components::LinearVelocityCmd(vel));
      }
      else
      {
          cmdComp->Data() = vel;
      }
  }

  void StopMotion(
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity entity)
  {
      gz::math::Vector3d vel(0, 0, 0);

      auto cmdComp =
          _ecm.Component<gz::sim::components::LinearVelocityCmd>(entity);

      if (!cmdComp)
      {
          _ecm.CreateComponent(
              entity,
              gz::sim::components::LinearVelocityCmd(vel));
      }
      else
      {
          cmdComp->Data() = vel;
      }
  }
};

}

GZ_ADD_PLUGIN(
  box_mover::BoxMoverPlugin,
  gz::sim::System,
  box_mover::BoxMoverPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(box_mover::BoxMoverPlugin, "box_mover::BoxMoverPlugin")