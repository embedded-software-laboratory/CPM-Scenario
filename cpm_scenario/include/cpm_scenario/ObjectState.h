/**
 * @file CpmObjectState.h
 * @authors Simon Schaefer
 * @date 05.05.2021
 */
#ifndef CPM_SCENARIO_OBJECT_STATE_H_
#define CPM_SCENARIO_OBJECT_STATE_H_

#include <Eigen/Dense>
#include <ostream>
#include <string>
#include <memory>

#include <pugixml.hpp>

namespace cpm_scenario {

class ObjectState {
 private:
  Eigen::Vector2d position_;
  Eigen::Vector2d velocity_;
  double orientation_ = 0.0;
  unsigned long timestamp_ = 0;
  unsigned long frame_ = 0;

 public:
  ObjectState();
  ObjectState(Eigen::Vector2d position, Eigen::Vector2d velocity);
  ObjectState(const ObjectState &other) = default;

  void Parse(pugi::xml_node node);
  void Write(pugi::xml_node *node);

  [[nodiscard]] const Eigen::Vector2d &GetPosition() const;
  void SetPosition(const Eigen::Vector2d &position);
  [[nodiscard]] const Eigen::Vector2d &GetVelocity() const;
  [[nodiscard]] double GetSpeed() const;
  void SetVelocity(const Eigen::Vector2d &velocity);
  [[nodiscard]] unsigned long GetTimestamp() const;
  void SetTimestamp(unsigned long timestamp);
  [[nodiscard]] double GetOrientation() const;
  void SetOrientation(double orientation);
  [[nodiscard]] unsigned long GetFrame() const;
  void SetFrame(unsigned long frame);

  [[nodiscard]] std::string ToString() const;

  friend std::ostream &operator<<(std::ostream &os, const ObjectState &state);
};
typedef std::shared_ptr<ObjectState> ObjectStatePtr;
typedef std::vector<ObjectStatePtr> ObjectStatePtrs;
}
#endif //CPM_SCENARIO_OBJECT_STATE_H_
