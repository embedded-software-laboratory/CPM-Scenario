#include "cpm_scenario/Scenario.h"

#include <iostream>

namespace cpm_scenario {
Scenario::Scenario(std::string name)
    : name_(std::move(name)) {}

void Scenario::Parse(pugi::xml_node *node) {
  auto name = node->attribute("name").value();
  auto number_of_frame = node->attribute("frames").as_uint();
  this->SetName(name);
  this->SetNumberOfFrames(number_of_frame);

  pugi::xml_node obstacle_node = node->child("obstacles");
  for (auto element : obstacle_node.children("o")) {
    auto object = std::make_shared<ExtendedObject>();
    object->Parse(element);
    this->objects_.push_back(object);
  }

  pugi::xml_node vehicles_node = node->child("vehicles");
  for (auto element : vehicles_node.children("v")) {
    auto object = std::make_shared<ExtendedObject>();
    object->Parse(element);
    this->objects_.push_back(object);
  }

}
void Scenario::Write(pugi::xml_node *node) {
  node->append_attribute("name").set_value(this->name_.c_str());
  node->append_attribute("frames").set_value(this->GetNumberOfFrames());

  // Add root node
  pugi::xml_node obstacle_node = node->append_child(pugi::node_element);
  obstacle_node.set_name("obstacles");

  size_t id = 0;
  for (const auto &element : this->objects_) {
    if (element->GetType() == ExtendedObjectType::CAR) continue;
    //if (element->GetType() == CpmExtendedObjectType::UNKNOWN) continue;

    // Create node
    pugi::xml_node element_node = obstacle_node.append_child(pugi::node_element);
    element_node.set_name("o");

    // Write node
    element->Write(&element_node);

    // Overwrite id to have consistent numbering
    element_node.attribute("id").set_value(id);
    id++;
  }

  // Add root node
  pugi::xml_node vehicle_node = node->append_child(pugi::node_element);
  vehicle_node.set_name("vehicles");

  for (const auto &element : this->objects_) {
    if (element->GetType() != ExtendedObjectType::CAR) continue;
    // Create node
    pugi::xml_node element_node = vehicle_node.append_child(pugi::node_element);
    element_node.set_name("v");

    // Write node
    element->Write(&element_node);

    // Overwrite id to have consistent numbering
    element_node.attribute("id").set_value(id);
    id++;
  }
}
void Scenario::WriteMeta(pugi::xml_node *node) {
  node->append_attribute("runtime").set_value(this->GetLengthInNanoSeconds());
  node->append_attribute("numberOfVehicles").set_value(this->GetNumberOfVehicles());
  node->append_attribute("numberOfVehiclesSimultaneously").set_value(this->GetNumberOfVehiclesSimultaneously());
  node->append_attribute("numberOfObstacles").set_value(this->GetNumberOfObstacles());
  node->append_attribute("numberOfObstaclesSimultaneously").set_value(this->GetNumberOfObstaclesSimultaneously());
  double vehicle_density = static_cast<double>(this->GetNumberOfVehicles()) * static_cast<double>(
      60e9) / static_cast<double>(this->GetLengthInNanoSeconds());
  node->append_attribute("vehicleDensity").set_value(vehicle_density);
}

void Scenario::AddObject(const ExtendedObjectPtr &object) {
  this->objects_.push_back(object);
}
const std::vector<ExtendedObjectPtr> &Scenario::GetObjects() const {
  return this->objects_;
}
std::vector<ExtendedObjectPtr> Scenario::GetVehicle() const {
  cpm_scenario::ExtendedObjectPtrs vehicle;
  std::copy_if(this->objects_.begin(), this->objects_.end(), std::back_inserter(vehicle), [](const auto &object) {
    return object->GetType() == cpm_scenario::ExtendedObjectType::CAR;
  });
  return vehicle;
}
std::vector<ExtendedObjectPtr> Scenario::GetObstacles() const {
  cpm_scenario::ExtendedObjectPtrs obstacles;
  std::copy_if(this->objects_.begin(), this->objects_.end(), std::back_inserter(obstacles), [](const auto &object) {
    return object->GetType() != cpm_scenario::ExtendedObjectType::CAR;
  });
  return obstacles;
}
const std::string &Scenario::GetName() const {
  return this->name_;
}
void Scenario::SetName(const std::string &name) {
  this->name_ = name;
}
size_t Scenario::GetNumberOfFrames() const {

  return number_of_frames_;
}
void Scenario::SetNumberOfFrames(long number_of_frames) {
  number_of_frames_ = number_of_frames;
}
const std::string &Scenario::GetBackgroundImageSourcePath() const {
  return background_image_source_path_;
}
void Scenario::SetBackgroundImageSourcePath(const std::string &background_image_source_path) {
  background_image_source_path_ = background_image_source_path;
}
double Scenario::GetBackgroundImageScaleFactor() const {
  return background_image_scale_factor_;
}
void Scenario::SetBackgroundImageScaleFactor(double background_image_scale_factor) {
  background_image_scale_factor_ = background_image_scale_factor;
}
std::string Scenario::ToString() const {
  std::stringstream ss;
  ss << *this;
  return ss.str();
}
std::ostream &operator<<(std::ostream &os, const Scenario &object) {
  os << "Scenario: {" << std::endl;
  os << "  Name: " << object.name_ << std::endl;
  os << "  Objects: " << std::endl;
  if (object.objects_.empty()) os << "    None" << std::endl;
  for (const auto &objects : object.objects_) {
    os << "    " << objects << std::endl;
  }
  os << "}";
  return os;
}
Scenario::Scenario(const Scenario &other) {
  // Copy static elements
  this->name_ = other.name_;
  this->number_of_frames_ = other.number_of_frames_;
  this->background_image_source_path_ = other.background_image_source_path_;
  this->background_image_scale_factor_ = other.background_image_scale_factor_;

  // Copy dynamic elements
  for (const auto &element : other.objects_) {
    auto object = std::make_shared<ExtendedObject>(*element);
    this->AddObject(object);
  }
}

void Scenario::RestrictToFrames(size_t from_frame, size_t to_frame) {
  std::vector<size_t> objects_to_remove;
  for (size_t i = 0; i < objects_.size(); i++) {
    auto element = this->objects_.at(i);
    if (element->GetFirstFrame() <= from_frame || element->GetLastFrame() >= to_frame)
      objects_to_remove.push_back(i);
  }
  for (auto iter = objects_to_remove.rbegin(); iter != objects_to_remove.rend(); iter++) {
    auto it = this->objects_.begin();
    std::advance(it, (*iter));
    this->objects_.erase(it);
  }
  this->number_of_frames_ = to_frame - from_frame;
}
void Scenario::SetNewStartingFrame(size_t starting_frame) {
  for (auto &object : this->objects_) {
    object->ShiftFramesForward(static_cast<long>(starting_frame));
  }
}
void Scenario::Transform(Eigen::Rotation2Dd rotation,
                         const Eigen::Vector2d &shift,
                         const Eigen::AlignedScaling2d &scaling) {
  for (const auto &element : this->objects_)element->Transform(rotation, shift, scaling);
}
void Scenario::RestrictToArea(const Eigen::Vector2d &position, const Eigen::Vector2d &dimension) {
  for (const auto &element : this->objects_)element->RestrictToArea(position, dimension);
}
void Scenario::RestrictTrajectories() {
  for (const auto &object : this->objects_) {
    // Only do this on cars and not obstacles
    if (object->GetType() != ExtendedObjectType::CAR) continue;

    auto states = object->GetStates();
    auto first_state = states[object->GetFirstFrame()];
    auto last_state = states[object->GetLastFrame()];
    for (auto state_pair : states) {
      auto state = state_pair.second;
      if (!state) continue;
      // Only keep first and last state
      if (state->GetFrame() == first_state->GetFrame() || state->GetFrame() == last_state->GetFrame()) continue;
      object->RemoveState(state);
    }
  }
}
unsigned long Scenario::GetLengthInNanoSeconds() const {
  if (this->objects_.empty()) return 0;
  auto object = this->objects_.front();
  auto time_between_states = object->GetTimeBetweenStates();
  return number_of_frames_ * time_between_states;
}
size_t Scenario::GetNumberOfVehicles() const {
  size_t counter = 0;
  for (const auto &object : this->objects_) {
    if (object->GetType() == ExtendedObjectType::CAR) counter++;
  }
  return counter;
}
size_t Scenario::GetNumberOfObstacles() const {
  size_t counter = 0;
  for (const auto &object : this->objects_) {
    if (object->GetType() != ExtendedObjectType::CAR) counter++;
  }
  return counter;
}
size_t Scenario::GetNumberOfVehiclesSimultaneously() const {
  size_t max_vehicle_count = 0;
  for (long frame = 0; frame < this->GetNumberOfFrames(); frame++) {
    size_t vehicle_count = 0;
    for (const auto &object : this->objects_) {
      if (object->GetType() != ExtendedObjectType::CAR) continue;
      if (object->IsInScene(frame)) vehicle_count++;
    }
    if (vehicle_count > max_vehicle_count)
      max_vehicle_count = vehicle_count;
  }
  return max_vehicle_count;
}
size_t Scenario::GetNumberOfObstaclesSimultaneously() const {
  size_t max_obstacle_count = 0;
  for (long frame = 0; frame < this->GetNumberOfFrames(); frame++) {
    size_t obstacle_count = 0;
    for (const auto &object : this->objects_) {
      if (object->GetType() == ExtendedObjectType::CAR) continue;
      if (object->IsInScene(frame)) obstacle_count++;
    }
    if (obstacle_count > max_obstacle_count)
      max_obstacle_count = obstacle_count;
  }
  return max_obstacle_count;
}
}
