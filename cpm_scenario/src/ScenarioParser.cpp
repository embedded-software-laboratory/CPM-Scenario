#include "cpm_scenario/ScenarioParser.h"

#include <utility>

namespace cpm_scenario {

ScenarioParser::ScenarioParser(std::string name) {
  this->scenario_ptr_ = std::make_shared<Scenario>(name);
}
void ScenarioParser::Parse(const std::string &file_path) {
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(file_path.c_str());
  if(!result){
    throw std::runtime_error("Parsing xml file: " + file_path + " fail!");
  }
  pugi::xml_node root_node = doc.child("cpmScenario");
  this->scenario_ptr_->Parse(&root_node);
}
void ScenarioParser::Transform(Eigen::Rotation2Dd rotation,
                               const Eigen::Vector2d &shift,
                               const Eigen::AlignedScaling2d &scaling) {
  this->scenario_ptr_->Transform(rotation, shift, scaling);
}
ScenarioPtr ScenarioParser::GetScenario() {
  return this->scenario_ptr_;
}
}