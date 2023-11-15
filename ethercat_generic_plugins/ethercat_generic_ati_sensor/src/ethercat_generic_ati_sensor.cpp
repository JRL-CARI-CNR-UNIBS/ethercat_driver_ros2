// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#include <numeric>

#include "ethercat_generic_plugins/ethercat_generic_ati_sensor.hpp"

namespace ethercat_generic_plugins
{

  EcAtiSensor::EcAtiSensor()
    : GenericEcSlave() {}
  EcAtiSensor::~EcAtiSensor() {}

  void EcAtiSensor::processData(size_t index, uint8_t * domain_address)
  {
    GenericEcSlave::processData(index, domain_address);

    iter_ ++;

    if ( iter_ == domain_map_.size() )
    {
      iter_ = 0;

      for (auto i = 0ul; i < FT_CH_NUM; i++)
      {
        sensor_output_[i] = state_interface_ptr_->at(std::stoi(paramters_["state_interface/sensor_channel_" + std::to_string(i)]));
        unbiased_sensor_output_[i] = sensor_output_[i] - ft_offset_[i];
        unbiased_sensor_output_volt_[i] = (unbiased_sensor_output_[i] * 10.0) / 32768.0;
      }

      state_interface_ptr_->at(std::stoi(paramters_["state_interface/force_x"])) =
          std::inner_product(std::begin(sensor_matrix_[0]),
                             std::end(sensor_matrix_[0]),
                             std::begin(unbiased_sensor_output_volt_),
                             0.0);

      state_interface_ptr_->at(std::stoi(paramters_["state_interface/force_y"])) =
          std::inner_product(std::begin(sensor_matrix_[1]),
                             std::end(sensor_matrix_[1]),
                             std::begin(unbiased_sensor_output_volt_),
                             0.0);

      state_interface_ptr_->at(std::stoi(paramters_["state_interface/force_z"])) =
          std::inner_product(std::begin(sensor_matrix_[2]),
                             std::end(sensor_matrix_[2]),
                             std::begin(unbiased_sensor_output_volt_),
                             0.0);

      state_interface_ptr_->at(std::stoi(paramters_["state_interface/torque_x"])) =
          std::inner_product(std::begin(sensor_matrix_[3]),
                             std::end(sensor_matrix_[3]),
                             std::begin(unbiased_sensor_output_volt_),
                             0.0);

      state_interface_ptr_->at(std::stoi(paramters_["state_interface/torque_y"])) =
          std::inner_product(std::begin(sensor_matrix_[4]),
                             std::end(sensor_matrix_[4]),
                             std::begin(unbiased_sensor_output_volt_),
                             0.0);

      state_interface_ptr_->at(std::stoi(paramters_["state_interface/torque_z"])) =
          std::inner_product(std::begin(sensor_matrix_[5]),
                             std::end(sensor_matrix_[5]),
                             std::begin(unbiased_sensor_output_volt_),
                             0.0);

      bool new_value;
      if (std::isnan(command_interface_ptr_->at(std::stoi(paramters_["command_interface/sensor_reset"]))))
      {
        new_value = false;
      }
      else
      {
        new_value = (command_interface_ptr_->at(std::stoi(paramters_["command_interface/sensor_reset"])) != 0);
      }

      if ( new_value && !zeroing_command_)
      {
        sensor_zeroed_ = false;
      }
      zeroing_command_ = new_value;

      if (!sensor_zeroed_)
      {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("EcAtiSensor"),
                           "Sensor zeroed");

        ft_offset_[0] = state_interface_ptr_->at(ft_si_idxs_[0]);
        ft_offset_[1] = state_interface_ptr_->at(ft_si_idxs_[1]);
        ft_offset_[2] = state_interface_ptr_->at(ft_si_idxs_[2]);
        ft_offset_[3] = state_interface_ptr_->at(ft_si_idxs_[3]);
        ft_offset_[4] = state_interface_ptr_->at(ft_si_idxs_[4]);
        ft_offset_[5] = state_interface_ptr_->at(ft_si_idxs_[5]);

        sensor_zeroed_ = true;
      }
    }
  }

  bool EcAtiSensor::setupSlave(
      std::unordered_map<std::string, std::string> slave_paramters,
      std::vector<double> * state_interface,
      std::vector<double> * command_interface)
  {


    GenericEcSlave::setupSlave(slave_paramters, state_interface, command_interface);
    if (paramters_.find("command_interface/sensor_reset") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "Command_interface with name reset_offset is required");
      return false;
    }

    if (paramters_.find("state_interface/sensor_channel_0") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name sensor_channel_0 is required");
      return false;
    }
    else
    {
      ft_si_idxs_[0] = std::stoi(paramters_["state_interface/sensor_channel_0"]);
    }

    if (paramters_.find("state_interface/sensor_channel_1") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name sensor_channel_1 is required");
      return false;
    }
    else
    {
      ft_si_idxs_[1] = std::stoi(paramters_["state_interface/sensor_channel_1"]);
    }

    if (paramters_.find("state_interface/sensor_channel_2") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name sensor_channel_2 is required");
      return false;
    }
    else
    {
      ft_si_idxs_[2] = std::stoi(paramters_["state_interface/sensor_channel_2"]);
    }

    if (paramters_.find("state_interface/sensor_channel_3") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name sensor_channel_3 is required");
      return false;
    }
    else
    {
      ft_si_idxs_[3] = std::stoi(paramters_["state_interface/sensor_channel_3"]);
    }

    if (paramters_.find("state_interface/sensor_channel_4") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name sensor_channel_4 is required");
      return false;
    }
    else
    {
      ft_si_idxs_[4] = std::stoi(paramters_["state_interface/sensor_channel_4"]);
    }

    if (paramters_.find("state_interface/sensor_channel_5") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name sensor_channel_5 is required");
      return false;
    }
    else
    {
      ft_si_idxs_[5] = std::stoi(paramters_["state_interface/sensor_channel_5"]);
    }

    if (paramters_.find("state_interface/force_x") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name force_x is required");
      return false;
    }

    if (paramters_.find("state_interface/force_y") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name force_y is required");
      return false;
    }

    if (paramters_.find("state_interface/force_z") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name force_z is required");
      return false;
    }

    if (paramters_.find("state_interface/torque_x") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name torque_x is required");
      return false;
    }

    if (paramters_.find("state_interface/torque_y") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name torque_y is required");
      return false;
    }

    if (paramters_.find("state_interface/torque_z") == paramters_.end())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"),
                          "State_interface with name torque_z is required");
      return false;
    }

    if (paramters_.find("sensor_runtime_matrix") != paramters_.end()) {
      if (!setup_sensor_matrix_from_file(paramters_["sensor_runtime_matrix"])) {
        return false;
      }
    } else {
      std::cerr << "EcAtiSensor: failed to find 'sensor_runtime_matrix' tag in URDF." << std::endl;
      return false;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("EcAtiSensor"),
                       "ft max values: ["
                       << ft_max_values_[0]
                       << ", "
                       << ft_max_values_[1]
                       << ", "
                       << ft_max_values_[2]
                       << ", "
                       << ft_max_values_[3]
                       << ", "
                       << ft_max_values_[4]
                       << ", "
                       << ft_max_values_[5]
                       << "]");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("EcAtiSensor"),
                       "ft runtime matrix values: ");
   for (auto i = 0ul; i < sensor_matrix_.size(); i++)
   {
     RCLCPP_INFO_STREAM(rclcpp::get_logger("EcAtiSensor"),
                        "                         ["
                        << sensor_matrix_[i][0]
                        << ", "
                        << sensor_matrix_[i][1]
                        << ", "
                        << sensor_matrix_[i][2]
                        << ", "
                        << sensor_matrix_[i][3]
                        << ", "
                        << sensor_matrix_[i][4]
                        << ", "
                        << sensor_matrix_[i][5]
                        << "]");
   }
    return true;
  }

  bool EcAtiSensor::setup_sensor_matrix(YAML::Node matrix_yaml)
  {
    sensor_matrix_.resize(6);
    ft_max_values_.resize(6);
    if (matrix_yaml.size() != 0)
    {
      if (matrix_yaml["useraxis"]) {
        if (matrix_yaml["useraxis"]["fx"]) {
          if (matrix_yaml["useraxis"]["fx"]["value"].size() == 6) {
            for (auto i = 0ul; i < matrix_yaml["useraxis"]["fx"]["value"].size(); i++)
              sensor_matrix_[0].push_back(matrix_yaml["useraxis"]["fx"]["value"][i].as<double>());
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "fx value vector hasn't size 6.");
            return false;
          }
          if (matrix_yaml["useraxis"]["fx"]["max"]) {
            ft_max_values_[0] = matrix_yaml["useraxis"]["fx"]["max"].as<double>();
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "fx max not found.");
            return false;
          }
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load sensor matrix, fx not found.");
          return false;
        }
        if (matrix_yaml["useraxis"]["fy"]) {
          if (matrix_yaml["useraxis"]["fy"]["value"].size() == 6) {
            for (auto i = 0ul; i < matrix_yaml["useraxis"]["fy"]["value"].size(); i++)
              sensor_matrix_[1].push_back(matrix_yaml["useraxis"]["fy"]["value"][i].as<double>());
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "fy vector hasn't size 6.");
            return false;
          }
          if (matrix_yaml["useraxis"]["fy"]["max"]) {
            ft_max_values_[1] = matrix_yaml["useraxis"]["fy"]["max"].as<double>();
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "fy max not found.");
            return false;
          }
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load sensor matrix, fy not found.");
          return false;
        }
        if (matrix_yaml["useraxis"]["fz"]) {
          if (matrix_yaml["useraxis"]["fz"]["value"].size() == 6) {
            for (auto i = 0ul; i < matrix_yaml["useraxis"]["fz"]["value"].size(); i++)
              sensor_matrix_[2].push_back(matrix_yaml["useraxis"]["fz"]["value"][i].as<double>());
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "fz vector hasn't size 6.");
            return false;
          }
          if (matrix_yaml["useraxis"]["fz"]["max"]) {
            ft_max_values_[2] = matrix_yaml["useraxis"]["fz"]["max"].as<double>();
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "fz max not found.");
            return false;
          }
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load sensor matrix, fz not found.");
          return false;
        }
        if (matrix_yaml["useraxis"]["tx"]) {
          if (matrix_yaml["useraxis"]["tx"]["value"].size() == 6) {
            for (auto i = 0ul; i < matrix_yaml["useraxis"]["tx"]["value"].size(); i++)
              sensor_matrix_[3].push_back(matrix_yaml["useraxis"]["tx"]["value"][i].as<double>());
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "tx vector hasn't size 6.");
            return false;
          }
          if (matrix_yaml["useraxis"]["tx"]["max"]) {
            ft_max_values_[3] = matrix_yaml["useraxis"]["tx"]["max"].as<double>();
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "tx max not found.");
            return false;
          }
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load sensor matrix, tx not found.");
          return false;
        }
        if (matrix_yaml["useraxis"]["ty"]) {
          if (matrix_yaml["useraxis"]["ty"]["value"].size() == 6) {
            for (auto i = 0ul; i < matrix_yaml["useraxis"]["ty"]["value"].size(); i++)
              sensor_matrix_[4].push_back(matrix_yaml["useraxis"]["ty"]["value"][i].as<double>());
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "ty vector hasn't size 6.");
            return false;
          }
          if (matrix_yaml["useraxis"]["ty"]["max"]) {
            ft_max_values_[4] = matrix_yaml["useraxis"]["ty"]["max"].as<double>();
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "ty max not found.");
            return false;
          }
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load sensor matrix, ty not found.");
          return false;
        }
        if (matrix_yaml["useraxis"]["tz"]) {
          if (matrix_yaml["useraxis"]["tz"]["value"].size() == 6) {
            for (auto i = 0ul; i < matrix_yaml["useraxis"]["tz"]["value"].size(); i++)
              sensor_matrix_[5].push_back(matrix_yaml["useraxis"]["tz"]["value"][i].as<double>());
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "tz vector hasn't size 6.");
            return false;
          }
          if (matrix_yaml["useraxis"]["tz"]["max"]) {
            ft_max_values_[5] = matrix_yaml["useraxis"]["tz"]["max"].as<double>();
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "tz max not found.");
            return false;
          }
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load sensor matrix, tz not found.");
          return false;
        }
      } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load sensor matrix, useraxis not found.");
        return false;
      }
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load sensor matrix: empty configuration.");
      return false;
    }
    return true;
  }

  bool EcAtiSensor::setup_sensor_matrix_from_file(std::string matrix_file)
  {
    YAML::Node matrix_yaml;
    try {
      matrix_yaml = YAML::LoadFile(matrix_file);
    } catch (const YAML::ParserException & ex) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load drive configuration: " << ex.what() );
      return false;
    } catch (const YAML::BadFile & ex) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EcAtiSensor"), "failed to load drive configuration: " << ex.what() );
      return false;
    }
    if (!setup_sensor_matrix(matrix_yaml)) {
      return false;
    }
    return true;
  }

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcAtiSensor, ethercat_interface::EcSlave)
