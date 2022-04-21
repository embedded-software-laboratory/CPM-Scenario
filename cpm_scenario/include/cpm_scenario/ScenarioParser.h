#ifndef CPM_SCENARIO_SCENARIO_PARSER_H_
#define CPM_SCENARIO_SCENARIO_PARSER_H_

#include "Scenario.h"

namespace cpm_scenario {
class ScenarioParser {
 private:
  ScenarioPtr scenario_ptr_;
 public:
  explicit ScenarioParser(std::string name);

  void Parse(const std::string &file_path);

  void Transform(Eigen::Rotation2Dd rotation, const Eigen::Vector2d& shift, const Eigen::AlignedScaling2d& scaling);

  ScenarioPtr GetScenario();
};
typedef std::shared_ptr<ScenarioParser> ScenarioParserPtr;
typedef std::shared_ptr<ScenarioParserPtr> ScenarioParserPtrs;
}
#endif //CPM_SCENARIO_SCENARIO_PARSER_H_
