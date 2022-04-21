/**
 * @file CpmExtendedObject.h
 * @authors Simon Schaefer
 * @date 05.05.2021
 */
#ifndef CPM_SCENARIO_EXTENDED_OBJECT_H_
#define CPM_SCENARIO_EXTENDED_OBJECT_H_

#include <map>
#include <vector>
#include <ostream>
#include <limits>

#include <Eigen/Dense>

#include "cpm_scenario/ObjectState.h"

#include <pugixml.hpp>

namespace cpm_scenario {

enum class ExtendedObjectType {
  PEDESTRIAN = 0,
  CAR = 1,
  BICYCLE_MOTORCYCLES = 2,
  TRUCK_BUS = 3,
  VAN = 4,
  TRAILER = 5,
  UNKNOWN = 10
};

class ExtendedObject {
 private:
  std::map<long, ObjectStatePtr> states_;
  Eigen::Vector2d dimension_;
  ExtendedObjectType type_;
  long id_;

  long frame_shift_ = 0;
  long time_between_states_ = 0;

  void RestrictToArea(Eigen::Vector2d position, Eigen::Vector2d dimension);
  void Transform(Eigen::Rotation2Dd rotation, const Eigen::Vector2d &shift, const Eigen::AlignedScaling2d &scaling);
  void UpdateSpeedValues();
 public:
  ExtendedObject();
  explicit ExtendedObject(long id, Eigen::Vector2d dimension, ExtendedObjectType type);
  ExtendedObject(const ExtendedObject &other);

  void Write(pugi::xml_node *node);
  void Parse(pugi::xml_node node);

  void AddState(const ObjectStatePtr &state);
  void RemoveState(const ObjectStatePtr &state);
  [[nodiscard]] const std::map<long, ObjectStatePtr> &GetStates() const;

  [[nodiscard]] long GetFirstFrame() const;
  [[nodiscard]] long GetLastFrame() const;
  [[nodiscard]] long GetFrameFromTimeStamp(unsigned long current_timestamp) const;
  [[nodiscard]] unsigned long GetFirstTimestamp() const;
  [[nodiscard]] unsigned long GetLastTimestamp() const;
  [[nodiscard]] unsigned long GetJourneyTime() const;
  [[nodiscard]] double GetMeanSpeed() const;
  [[nodiscard]] double GetMeanAcceleration() const;
  [[nodiscard]] bool ContainsState(long current_frame) const;
  [[nodiscard]] bool ContainsTimestamp(unsigned long current_timestamp) const;
  void ShiftFramesForward(long frames_to_shift);

  [[nodiscard]] ObjectStatePtr GetFirstState() const;
  [[nodiscard]] ObjectStatePtr GetLastState() const;

  [[nodiscard]] ObjectStatePtr GetStateByFrame(long frame) const;
  [[nodiscard]] ObjectStatePtr GetStateByTimestamp(unsigned long timestamp) const;

  [[nodiscard]] const Eigen::Vector2d &GetDimension() const;
  void SetDimension(const Eigen::Vector2d &dimension);

  [[nodiscard]] ExtendedObjectType GetType() const;
  [[nodiscard]] std::string GetTypeString() const;
  void SetType(ExtendedObjectType type);
  void SetTypeByString(const std::string &type_string);

  [[nodiscard]] long GetTimeBetweenStates() const;

  [[nodiscard]] long GetId() const;
  void SetId(long id);

  [[nodiscard]] bool IsInScene(long current_frame) const;

  [[nodiscard]] std::string ToString() const;

  friend std::ostream &operator<<(std::ostream &os, const ExtendedObject &object);
  friend class Scenario;
  [[nodiscard]] double GetMinSpeed() const;
  [[nodiscard]] double GetMaxSpeed() const;
  [[nodiscard]] double GetMinAcceleration() const;
  [[nodiscard]] double GetMaxAcceleration() const;
  [[nodiscard]] double GetTraveledDistance() const;

};
typedef std::shared_ptr<ExtendedObject> ExtendedObjectPtr;
typedef std::vector<ExtendedObjectPtr> ExtendedObjectPtrs;
}
#endif //CPM_SCENARIO_EXTENDED_OBJECT_H_
