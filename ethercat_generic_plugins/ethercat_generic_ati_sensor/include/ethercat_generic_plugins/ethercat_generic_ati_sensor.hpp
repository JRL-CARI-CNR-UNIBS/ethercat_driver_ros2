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

#ifndef ETHERCAT_GENERIC_PLUGINS__ETHERCAT_GENERIC_ATI_SENSOR_HPP_
#define ETHERCAT_GENERIC_PLUGINS__ETHERCAT_GENERIC_ATI_SENSOR_HPP_

//#include <vector>
//#include <string>
//#include <unordered_map>
//#include <limits>
#include "rclcpp/macros.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <bitset>

//#include "yaml-cpp/yaml.h"
//#include "ethercat_interface/ec_slave.hpp"
//#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "ethercat_generic_plugins/generic_ec_slave.hpp"

#define FT_CH_NUM 6

namespace ethercat_generic_plugins
{

class EcAtiSensor : public GenericEcSlave
{
public:
  EcAtiSensor();
  virtual ~EcAtiSensor();

  virtual void processData(size_t index, uint8_t * domain_address);

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

protected:
  bool zeroing_command_ = false;
  bool sensor_zeroed_ = false;
  std::size_t iter_ = 0;
  std::vector<int> ft_si_idxs_ = {-1, -1, -1, -1, -1, -1};
  std::vector<int> ft_offset_ = {0, 0, 0, 0, 0, 0};
  std::vector<double> sensor_output_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> unbiased_sensor_output_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> unbiased_sensor_output_volt_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ft_max_values_;
  std::vector<std::vector<double>> sensor_matrix_;
  bool setup_sensor_matrix(YAML::Node matrix_yaml);
  bool setup_sensor_matrix_from_file(std::string matrix_file);
  };

}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__ETHERCAT_GENERIC_ATI_SENSOR_HPP_
