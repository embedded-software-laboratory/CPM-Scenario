#include "cpm_scenario/ScenarioWriter.h"

#include <utility>

namespace cpm_scenario {
ScenarioWriter::ScenarioWriter(const ScenarioPtr &scenario_ptr) {
  // Copy scenario to not change original if transformation is applied
  this->scenario_ptr_ = std::make_shared<Scenario>(*scenario_ptr);
}
void ScenarioWriter::Write(const std::string &file_path) {
  this->WriteScenario(file_path);
  //this->WriteScenarioMeta(file_path+".Meta.xml");
}
void ScenarioWriter::WriteScenario(const std::string &file_path) {// Create xml document
  pugi::xml_document document;

  // Add a custom declaration node
  pugi::xml_node decl = document.append_child(pugi::node_declaration);
  decl.append_attribute("version") = "1.0";
  decl.append_attribute("encoding") = "UTF-8";
  decl.append_attribute("standalone") = "no";

  // Add root node
  pugi::xml_node root_node = document.append_child(pugi::node_element);
  root_node.set_name("cpmScenario");

  // Write scenario to node
  scenario_ptr_->Write(&root_node);
  scenario_ptr_->WriteMeta(&root_node);

  // Save to file
  document.save_file(file_path.c_str());
}
void ScenarioWriter::RestrictToFrames(size_t from_frame, size_t to_frame) {
  this->scenario_ptr_->RestrictToFrames(from_frame, to_frame);
  this->scenario_ptr_->SetNewStartingFrame(from_frame);
}
void ScenarioWriter::RestrictToArea(Eigen::Vector2d position, Eigen::Vector2d dimension) {
  this->scenario_ptr_->RestrictToArea(std::move(position), std::move(dimension));
}
void ScenarioWriter::Transform(Eigen::Rotation2Dd rotation, Eigen::Vector2d shift, Eigen::AlignedScaling2d scaling) {
  this->scenario_ptr_->Transform(rotation, std::move(shift), scaling);
}
void ScenarioWriter::RestrictTrajectories() {
  this->scenario_ptr_->RestrictTrajectories();
}
void ScenarioWriter::SetName(const std::string &name) {
  this->scenario_ptr_->SetName(name);
}
}
