#include "cpm_scenario/ExtendedObject.h"

#include <sstream>
#include <utility>
#include <numeric>

#include <Eigen/Dense>

namespace cpm_scenario {
std::string ExtendedObject::GetTypeString() const {
  switch (this->type_) {
    case ExtendedObjectType::PEDESTRIAN:return "pedestrian";
    case ExtendedObjectType::CAR:return "car";
    case ExtendedObjectType::UNKNOWN:return "unknown";
    case ExtendedObjectType::BICYCLE_MOTORCYCLES:return "bicycle_motorcycles";
    case ExtendedObjectType::TRUCK_BUS:return "truck_bus";
    case ExtendedObjectType::VAN:return "van";
    case ExtendedObjectType::TRAILER:return "trailer";
  }
  return "unknown";
}

ExtendedObject::ExtendedObject()
    : ExtendedObject(0, {0.0, 0.0}, ExtendedObjectType::UNKNOWN) {}
ExtendedObject::ExtendedObject(long id, Eigen::Vector2d dimension, ExtendedObjectType type)
    : id_(id), dimension_(std::move(dimension)), type_(type) {}
ExtendedObject::ExtendedObject(const ExtendedObject &other) {
  // Copy static elements
  this->dimension_ = other.dimension_;
  this->type_ = other.type_;
  this->id_ = other.id_;

  // Copy dynamic elements
  for (const auto &element: other.states_) {
    auto state = std::make_shared<ObjectState>(*element.second);
    this->AddState(state);
  }
}
void ExtendedObject::AddState(const ObjectStatePtr &state) {
  this->states_[state->GetFrame()] = state;
  if (this->states_.size() == 2) {
    this->time_between_states_ =
        this->states_.at(GetLastFrame())->GetTimestamp() - this->states_.at(GetFirstFrame())->GetTimestamp();
  }
}
void ExtendedObject::RemoveState(const ObjectStatePtr &state) {
  this->states_.erase(state->GetFrame());
}
const std::map<long, ObjectStatePtr> &ExtendedObject::GetStates() const {
  return states_;
}
long ExtendedObject::GetFirstFrame() const {
  return this->states_.empty() ? -1 : this->states_.begin()->first + this->frame_shift_;
}
long ExtendedObject::GetLastFrame() const {
  return this->states_.empty() ? -1 : this->states_.rbegin()->first + this->frame_shift_;
}
const Eigen::Vector2d &ExtendedObject::GetDimension() const {
  return dimension_;
}
void ExtendedObject::SetDimension(const Eigen::Vector2d &dimension) {
  dimension_ = dimension;
}
ExtendedObjectType ExtendedObject::GetType() const {
  return type_;
}
void ExtendedObject::SetType(ExtendedObjectType type) {
  type_ = type;
}
long ExtendedObject::GetId() const {
  return id_;
}
void ExtendedObject::SetId(long id) {
  id_ = id;
}
std::string ExtendedObject::ToString() const {
  std::stringstream ss;
  ss << *this;
  return ss.str();
}
bool ExtendedObject::ContainsState(long current_frame) const {
  return this->states_.count(current_frame + this->frame_shift_) >= 1;
}
bool ExtendedObject::IsInScene(long current_frame) const {
  return this->GetFirstFrame() <= current_frame && this->GetLastFrame() >= current_frame;
}
void ExtendedObject::ShiftFramesForward(long frames_to_shift) {
  this->frame_shift_ = -frames_to_shift;

}
std::ostream &operator<<(std::ostream &os, const cpm_scenario::ExtendedObject &object) {
  const Eigen::IOFormat kFormat(2, Eigen::DontAlignCols, "", ", ", "", "", "", "");
  os << "ExtendedObject: {" << std::endl;
  os << "  Dimension: " << object.dimension_.format(kFormat) << std::endl;
  os << "  Type: " << object.GetTypeString() << std::endl;
  os << "  ID: " << object.id_ << std::endl;
  os << "  Frame Shift: " << object.frame_shift_ << std::endl;
  os << "  States: " << std::endl;
  if (object.states_.empty()) os << "    None" << std::endl;
  for (const auto &object_state: object.states_) {
    os << "    " << object_state.second << std::endl;
  }
  os << "}";
  return os;
}
void ExtendedObject::Parse(pugi::xml_node node) {
  auto id = node.attribute("id").as_llong();
  auto length = node.attribute("length").as_double();
  auto width = node.attribute("width").as_double();
  auto type = node.attribute("type").value();

  this->id_ = id;
  this->dimension_.x() = length;
  this->dimension_.y() = width;
  this->SetTypeByString(type);
  pugi::xml_node states_node = node.child("states");
  for (auto element: states_node.children("s")) {
    auto state = std::make_shared<ObjectState>();
    state->Parse(element);
    this->AddState(state);
  }

  this->UpdateSpeedValues();
}
void ExtendedObject::Write(pugi::xml_node *node) {
  node->append_attribute("id").set_value(this->id_);
  node->append_attribute("length").set_value(this->dimension_.x());
  node->append_attribute("width").set_value(this->dimension_.y());
  node->append_attribute("type").set_value(this->GetTypeString().c_str());

  // Add root node
  pugi::xml_node vehicle_node = node->append_child(pugi::node_element);
  vehicle_node.set_name("states");

  size_t state_number = 0;
  for (const auto &element: this->states_) {
    ObjectState copy = *element.second;
    // Create node
    pugi::xml_node element_node = vehicle_node.append_child(pugi::node_element);
    element_node.set_name("s");

    // Shift frame
    long frame = element.second->GetFrame();
    long time_step = element.second->GetTimestamp();
    copy.SetFrame(frame + this->frame_shift_);
    copy.SetTimestamp(time_step + this->frame_shift_ * this->time_between_states_);

    // Write node
    copy.Write(&element_node);

    // Overwrite id to have consistent numbering
    element_node.attribute("id").set_value(state_number);
    state_number++;
  }

}
void ExtendedObject::UpdateSpeedValues() {
  ObjectStatePtr last_state = this->states_.at(this->GetFirstFrame());
  for (const auto &element: this->states_) {
    if (last_state->GetFrame() == element.first) continue;
    if (last_state->GetSpeed() != 0) return;
    auto state = element.second;

    Eigen::Vector2d direction = state->GetPosition() - last_state->GetPosition();
    double distance = (direction).norm();
    direction.normalize();
    unsigned long time_between_states = state->GetTimestamp() - last_state->GetTimestamp();
    double time_in_seconds = static_cast<double>(time_between_states) * 1e-9;
    double speed = distance / time_in_seconds;
    last_state->SetVelocity(direction * speed);

    last_state = element.second;
  }
}
void ExtendedObject::Transform(Eigen::Rotation2Dd rotation,
                               const Eigen::Vector2d &shift,
                               const Eigen::AlignedScaling2d &scaling) {

  for (const auto &element: this->states_) {
    auto state = element.second;

    Eigen::Vector2d position_vector = state->GetPosition();
    position_vector = rotation * position_vector;
    position_vector = position_vector + shift;
    position_vector = scaling * position_vector;
    state->SetPosition(position_vector);

    double orientation = state->GetOrientation();
    orientation = orientation + rotation.angle();
    state->SetOrientation(orientation);

    Eigen::Vector2d velocity_vector = state->GetVelocity();
    velocity_vector = rotation * velocity_vector;
    velocity_vector = scaling * velocity_vector;
    state->SetVelocity(velocity_vector);
  }
}
void ExtendedObject::RestrictToArea(Eigen::Vector2d position, Eigen::Vector2d dimension) {
  std::vector<long> states_to_remove;

  for (const auto &element: this->states_) {
    auto state = element.second;
    auto id = element.first;

    Eigen::Vector2d position_vector = state->GetPosition();
    double pos_x = position_vector.x();
    double pos_y = position_vector.y();

    bool inside = pos_x > position.x() && pos_x < position.x() + dimension.x() && pos_y > position.y()
        && pos_y < position.y() + dimension.y();
    if (!inside) states_to_remove.push_back(id);
  }
  for (auto element: states_to_remove) {
    this->states_.erase(element);
  }

}
void ExtendedObject::SetTypeByString(const std::string &type_string) {
  if (type_string == "pedestrian") this->type_ = ExtendedObjectType::PEDESTRIAN;
  if (type_string == "car") this->type_ = ExtendedObjectType::CAR;
  if (type_string == "unknown") this->type_ = ExtendedObjectType::UNKNOWN;
  if (type_string == "bicycle_motorcycles") this->type_ = ExtendedObjectType::BICYCLE_MOTORCYCLES;
  if (type_string == "truck_bus") this->type_ = ExtendedObjectType::TRUCK_BUS;
  if (type_string == "van") this->type_ = ExtendedObjectType::VAN;
  if (type_string == "trailer") this->type_ = ExtendedObjectType::TRAILER;
}
long ExtendedObject::GetTimeBetweenStates() const {
  return time_between_states_;
}
unsigned long ExtendedObject::GetJourneyTime() const {
  unsigned long last_time = std::prev(this->states_.end())->second->GetTimestamp();
  unsigned long first_time = this->states_.begin()->second->GetTimestamp();
  return last_time - first_time;
}

double ExtendedObject::GetMeanSpeed() const {
//  double mean_speed = std::accumulate(this->states_.begin(), states_.end(), 0,
//                            [](const double &i, const std::map<long, ObjectStatePtr>::value_type &o) {
//                              return i + o.second->GetSpeed();
//                            });
  double mean_speed = 0;
  for (const auto &state: this->states_) mean_speed += state.second->GetSpeed();
  return mean_speed / static_cast<double>(this->states_.size());
}
double ExtendedObject::GetMinSpeed() const {
  double min_speed = std::numeric_limits<double>::max();
  for (const auto &state: this->states_) {
    double speed = state.second->GetSpeed();
    if(speed < min_speed) min_speed = speed;
  }
  return min_speed;
}
double ExtendedObject::GetMaxSpeed() const {
  double max_speed = std::numeric_limits<double>::min();
  for (const auto &state: this->states_) {
    double speed = state.second->GetSpeed();
    if(speed > max_speed) max_speed = speed;
  }
  return max_speed;
}
double ExtendedObject::GetMeanAcceleration() const {
  double mean_acceleration = 0;
  double time_between_state = static_cast<double>(GetTimeBetweenStates()) * 1e-9;
  ObjectStatePtr last_state = nullptr;
  for (const auto &state_pair: this->states_) {
    auto state = state_pair.second;
    if (last_state == nullptr){
      last_state = state;
      continue;
    }
    double last_speed = last_state->GetSpeed();
    double speed = state->GetSpeed();
    double acceleration = std::abs(speed - last_speed) / time_between_state;
    mean_acceleration += acceleration;
    last_state = state;
  }
  return mean_acceleration / static_cast<double>(this->states_.size());
}
double ExtendedObject::GetMinAcceleration() const {
  double min_acceleration = std::numeric_limits<double>::max();
  double time_between_state = static_cast<double>(GetTimeBetweenStates()) * 1e-9;
  ObjectStatePtr last_state = nullptr;
  for (const auto &state_pair: this->states_) {
    auto state = state_pair.second;
    if (last_state == nullptr){
      last_state = state;
      continue;
    }
    double last_speed = last_state->GetSpeed();
    double speed = state->GetSpeed();
    double acceleration = std::abs(speed - last_speed) / time_between_state;
    if(acceleration < min_acceleration) min_acceleration = acceleration;
    last_state = state;
  }
  return min_acceleration;
}
double ExtendedObject::GetMaxAcceleration() const {
  double max_acceleration = std::numeric_limits<double>::min();
  double time_between_state = static_cast<double>(GetTimeBetweenStates()) * 1e-9;
  ObjectStatePtr last_state = nullptr;
  for (const auto &state_pair: this->states_) {
    auto state = state_pair.second;
    if (last_state == nullptr){
      last_state = state;
      continue;
    }
    double last_speed = last_state->GetSpeed();
    double speed = state->GetSpeed();
    double acceleration = std::abs(speed - last_speed) / time_between_state;
    if(acceleration > max_acceleration) max_acceleration = acceleration;
    last_state = state;
  }
  return max_acceleration;
}
unsigned long ExtendedObject::GetFirstTimestamp() const {
  return this->states_.empty() ? 0 : this->states_.begin()->second->GetTimestamp();
}
unsigned long ExtendedObject::GetLastTimestamp() const {
  return this->states_.empty() ? 0 : this->states_.rbegin()->second->GetTimestamp();
}
bool ExtendedObject::ContainsTimestamp(unsigned long current_timestamp) const {
  return current_timestamp >= GetFirstTimestamp() && current_timestamp <= GetLastTimestamp();
}
long ExtendedObject::GetFrameFromTimeStamp(unsigned long current_timestamp) const {
  if (!ContainsTimestamp(current_timestamp)) return -1;
  long closest_frame = -1;
  unsigned long time_difference = std::numeric_limits<unsigned long>::max();
  for (const auto &state: this->states_) {
    unsigned long current_time_difference = std::abs(static_cast<long>(state.second->GetTimestamp()) - static_cast<long>(current_timestamp));
    if (current_time_difference < time_difference) {
      closest_frame = state.first;
      time_difference = current_time_difference;
    }
  }
  return closest_frame;
}
ObjectStatePtr ExtendedObject::GetStateByFrame(long frame) const {
  if (this->states_.find(frame + this->frame_shift_) == this->states_.end()) return nullptr;
  return this->states_.at(frame);
}
ObjectStatePtr ExtendedObject::GetStateByTimestamp(unsigned long timestamp) const {
  auto frame = this->GetFrameFromTimeStamp(timestamp);
  if (frame < 0) return nullptr;
  return this->states_.at(frame);
}
ObjectStatePtr ExtendedObject::GetFirstState() const {
  return this->GetStateByFrame(this->GetFirstFrame());
}
ObjectStatePtr ExtendedObject::GetLastState() const {
  return this->GetStateByFrame(this->GetLastFrame());
}
double ExtendedObject::GetTraveledDistance() const {
  double distance = 0.0;
  ObjectStatePtr last_state = this->GetFirstState();
  for (const auto &element: this->states_) {
    // Skip if it is the same state
    if (last_state->GetFrame() == element.first) continue;
    auto state = element.second;

    Eigen::Vector2d direction = state->GetPosition() - last_state->GetPosition();
    distance += (direction).norm();
    last_state = state;
  }
  return distance;
}

}
