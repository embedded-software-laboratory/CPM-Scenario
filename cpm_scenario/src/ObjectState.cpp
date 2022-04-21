#include "cpm_scenario/ObjectState.h"

#include <sstream>
namespace cpm_scenario {

ObjectState::ObjectState() : ObjectState({0.0, 0.0}, {0.0, 0.0}) {}
ObjectState::ObjectState(Eigen::Vector2d position, Eigen::Vector2d velocity)
    : position_(std::move(position)), velocity_(std::move(velocity)) {}

const Eigen::Vector2d &ObjectState::GetPosition() const {
  return position_;
}
void ObjectState::SetPosition(const Eigen::Vector2d &position) {
  position_ = position;
}
const Eigen::Vector2d &ObjectState::GetVelocity() const {
  return velocity_;
}
void ObjectState::SetVelocity(const Eigen::Vector2d &velocity) {
  velocity_ = velocity;
}
unsigned long ObjectState::GetTimestamp() const {
  return timestamp_;
}
void ObjectState::SetTimestamp(unsigned long timestamp) {
  timestamp_ = timestamp;
}
double ObjectState::GetOrientation() const {
  return orientation_;
}
void ObjectState::SetOrientation(double orientation) {
  orientation_ = orientation;
}
unsigned long ObjectState::GetFrame() const {
  return frame_;
}
void ObjectState::SetFrame(unsigned long frame) {
  frame_ = frame;
}

std::string ObjectState::ToString() const {
  std::stringstream ss;
  ss << *this;
  return ss.str();
}

std::ostream &operator<<(std::ostream &os, const cpm_scenario::ObjectState &state) {
  const Eigen::IOFormat kFmt(2, Eigen::DontAlignCols, "", ", ", "", "", "", "");
  os << "ObjectState: " << " {Position: " << state.position_.format(kFmt) << " Velocity: " << state.velocity_.format(kFmt)
     << " Time: " << state.timestamp_ << " Orientation: " << state.orientation_ << "}";
  return os;
}
void ObjectState::Parse(pugi::xml_node node) {
  double x = node.attribute("x").as_double();
  double y = node.attribute("y").as_double();
  double vx = node.attribute("vx").as_double();
  double vy = node.attribute("vy").as_double();
  unsigned long time = node.attribute("t").as_ullong();
  unsigned long frame = node.attribute("f").as_ullong();
  double orientation = node.attribute("r").as_double();

  this->position_.x() = x;
  this->position_.y() = y;
  this->velocity_.x() = vx;
  this->velocity_.y() = vy;
  this->timestamp_ = time;
  this->frame_ = frame;
  this->orientation_ = orientation;
}
void ObjectState::Write(pugi::xml_node *node) {
  node->append_attribute("x").set_value(this->position_.x());
  node->append_attribute("y").set_value(this->position_.y());
  node->append_attribute("vx").set_value(this->velocity_.x());
  node->append_attribute("vy").set_value(this->velocity_.y());
  node->append_attribute("t").set_value(this->timestamp_);
  node->append_attribute("f").set_value(this->frame_);
  node->append_attribute("r").set_value(this->orientation_);
}
double ObjectState::GetSpeed() const {
  return this->velocity_.norm();
}

}