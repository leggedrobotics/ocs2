//
// Created by rgrandia on 22.09.20.
//

#include <ocs2_anymal_models/AnymalModels.h>

#include <unordered_map>

#include <ocs2_anymal_models/bear/AnymalBearCom.h>
#include <ocs2_anymal_models/bear/AnymalBearKinematics.h>
#include <ocs2_anymal_models/bear/WholebodyDynamicsBear.h>
#include <ocs2_anymal_models/croc/AnymalCrocCom.h>
#include <ocs2_anymal_models/croc/AnymalCrocKinematics.h>
#include <ocs2_anymal_models/croc/WholebodyDynamicsCroc.h>
#include <ocs2_anymal_models/wheels/AnymalWheelsCom.h>
#include <ocs2_anymal_models/wheels/AnymalWheelsKinematics.h>
#include <ocs2_anymal_models/wheels/WholebodyDynamicsWheels.h>

namespace anymal {

std::string toString(AnymalModel model) {
  static const std::unordered_map<AnymalModel, std::string> map{
      {AnymalModel::Bear, "bear"}, {AnymalModel::Croc, "croc"}, {AnymalModel::Wheels, "wheels"}};
  return map.at(model);
}

AnymalModel stringToAnymalModel(const std::string& name) {
  static const std::unordered_map<std::string, AnymalModel> map{
      {"bear", AnymalModel::Bear}, {"croc", AnymalModel::Croc}, {"wheels", AnymalModel::Wheels}};
  return map.at(name);
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getAnymalKinematics(AnymalModel model) {
  switch (model) {
    case AnymalModel::Bear:
      return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>>(new AnymalBearKinematics());
    case AnymalModel::Croc:
      return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>>(new AnymalCrocKinematics());
    case AnymalModel::Wheels:
      return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>>(new AnymalWheelsKinematics());
    default:
      throw std::runtime_error("[AnymalModels] unkown model");
  }
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getAnymalKinematicsAd(AnymalModel model) {
  switch (model) {
    case AnymalModel::Bear:
      return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>>(new AnymalBearKinematicsAd());
    case AnymalModel::Croc:
      return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>>(new AnymalCrocKinematicsAd());
    case AnymalModel::Wheels:
      return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>>(new AnymalWheelsKinematicsAd());
    default:
      throw std::runtime_error("[AnymalModels] unkown model");
  }
}

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getAnymalComModel(AnymalModel model) {
  switch (model) {
    case AnymalModel::Bear:
      return std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>>(new AnymalBearCom());
    case AnymalModel::Croc:
      return std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>>(new AnymalCrocCom());
    case AnymalModel::Wheels:
      return std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>>(new AnymalWheelsCom());
    default:
      throw std::runtime_error("[AnymalModels] unkown model");
  }
}

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getAnymalComModelAd(AnymalModel model) {
  switch (model) {
    case AnymalModel::Bear:
      return std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>>(new AnymalBearComAd());
    case AnymalModel::Croc:
      return std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>>(new AnymalCrocComAd());
    case AnymalModel::Wheels:
      return std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>>(new AnymalWheelsComAd());
    default:
      throw std::runtime_error("[AnymalModels] unkown model");
  }
}

std::unique_ptr<switched_model::WholebodyDynamics<ocs2::scalar_t>> getWholebodyDynamics(AnymalModel model) {
  switch (model) {
    case AnymalModel::Bear:
      return std::unique_ptr<switched_model::WholebodyDynamics<ocs2::scalar_t>>(new WholebodyDynamicsBear());
    case AnymalModel::Croc:
      return std::unique_ptr<switched_model::WholebodyDynamics<ocs2::scalar_t>>(new WholebodyDynamicsCroc());
    case AnymalModel::Wheels:
      return std::unique_ptr<switched_model::WholebodyDynamics<ocs2::scalar_t>>(new WholebodyDynamicsWheels());
    default:
      throw std::runtime_error("[AnymalModels] unkown model");
  }
}

std::unique_ptr<switched_model::WholebodyDynamics<ocs2::ad_scalar_t>> getWholebodyDynamicsAd(AnymalModel model) {
  switch (model) {
    case AnymalModel::Bear:
      return std::unique_ptr<switched_model::WholebodyDynamics<ocs2::ad_scalar_t>>(new WholebodyDynamicsBearAd());
    case AnymalModel::Croc:
      return std::unique_ptr<switched_model::WholebodyDynamics<ocs2::ad_scalar_t>>(new WholebodyDynamicsBearAd());
    case AnymalModel::Wheels:
      return std::unique_ptr<switched_model::WholebodyDynamics<ocs2::ad_scalar_t>>(new WholebodyDynamicsBearAd());
    default:
      throw std::runtime_error("[AnymalModels] unkown model");
  }
}

}  // namespace anymal