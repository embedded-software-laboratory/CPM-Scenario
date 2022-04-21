/**
 * @file CpmScenario.h
 * @authors Simon Schaefer
 * @date 05.05.2021
 */
#ifndef CPM_SCENARIO_SCENARIO_H_
#define CPM_SCENARIO_SCENARIO_H_

#include <string>
#include <vector>

#include "cpm_scenario/ExtendedObject.h"

#include <pugixml.hpp>

namespace cpm_scenario {

class Scenario {
 private:
  std::string name_;
  size_t number_of_frames_ = 0;
  ExtendedObjectPtrs objects_;
  std::string background_image_source_path_;
  double background_image_scale_factor_ = 1.0;

  virtual void Parse(pugi::xml_node *node);
  virtual void Write(pugi::xml_node *node);
  virtual void WriteMeta(pugi::xml_node *node);

  void RestrictToFrames(size_t from_frame, size_t to_frame);
  void SetNewStartingFrame(size_t starting_frame);
  void RestrictToArea(const Eigen::Vector2d& position, const Eigen::Vector2d& dimension);
  void Transform(Eigen::Rotation2Dd rotation, const Eigen::Vector2d& shift, const Eigen::AlignedScaling2d& scaling);
  void RestrictTrajectories();

 public:
  explicit Scenario(std::string name);
  Scenario(const Scenario &other);

  void AddObject(const ExtendedObjectPtr& object);
  [[nodiscard]] const std::vector<ExtendedObjectPtr> &GetObjects() const;
  [[nodiscard]] std::vector<ExtendedObjectPtr> GetObstacles() const;
  [[nodiscard]] std::vector<ExtendedObjectPtr> GetVehicle() const;

  [[nodiscard]] const std::string &GetName() const;
  void SetName(const std::string &name);

  [[nodiscard]] size_t GetNumberOfFrames() const;
  [[nodiscard]] unsigned long GetLengthInNanoSeconds() const;
  [[nodiscard]] size_t GetNumberOfVehicles() const;
  [[nodiscard]] size_t GetNumberOfVehiclesSimultaneously() const;
  [[nodiscard]] size_t GetNumberOfObstacles() const;
  [[nodiscard]] size_t GetNumberOfObstaclesSimultaneously() const;
  void SetNumberOfFrames(long number_of_frames);

  [[nodiscard]] const std::string &GetBackgroundImageSourcePath() const;
  void SetBackgroundImageSourcePath(const std::string &background_image_source_path);

  [[nodiscard]] double GetBackgroundImageScaleFactor() const;
  void SetBackgroundImageScaleFactor(double background_image_scale_factor);

  [[nodiscard]] std::string ToString() const;

  friend std::ostream &operator<<(std::ostream &os, const Scenario &object);
  friend class ScenarioWriter;
  friend class ScenarioParser;
  friend class DatasetParser;
};
typedef std::shared_ptr<Scenario> ScenarioPtr;
typedef std::vector<ScenarioPtr> ScenarioPtrs;
}

#endif //CPM_SCENARIO_SCENARIO_H_
