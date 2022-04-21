#ifndef CPM_SCENARIO_SCENARIO_WRITER_H_
#define CPM_SCENARIO_SCENARIO_WRITER_H_

#include "Scenario.h"

namespace cpm_scenario {
class ScenarioWriter {
 private:
  ScenarioPtr scenario_ptr_;
    void WriteScenario(const std::string &file_path);

 public:
  explicit ScenarioWriter(const ScenarioPtr &scenario_ptr);

  void Write(const std::string &file_path);

  void RestrictToFrames(size_t from_frame, size_t to_frame);
  void RestrictToArea(Eigen::Vector2d position, Eigen::Vector2d dimension);
  void Transform(Eigen::Rotation2Dd rotation, Eigen::Vector2d shift, Eigen::AlignedScaling2d scaling);
  void RestrictTrajectories();
  void SetName(const std::string& name);
};
typedef std::shared_ptr<ScenarioWriter> ScenarioWriterPtr;
typedef std::shared_ptr<ScenarioWriterPtr> ScenarioWriterPtrs;
}
#endif //CPM_SCENARIO_SCENARIO_WRITER_H_
