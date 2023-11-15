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

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace ethercat_generic_plugins
{

  EcCiA402Drive::EcCiA402Drive()
    : GenericEcSlave() {}
  EcCiA402Drive::~EcCiA402Drive() {}

  bool EcCiA402Drive::initialized() const {return initialized_;}

  void EcCiA402Drive::processData(size_t index, uint8_t * domain_address)
  {
    // Special case: ControlWord

    //  RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"), "pdo index: " << pdo_channels_info_[index].index <<" value: "<< pdo_channels_info_[index].default_value << " last_value " << pdo_channels_info_[index]);
    if (pdo_channels_info_[index].index == CiA402D_RPDO_CONTROLWORD) {
      //RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"), "is operational: " << is_operational_ <<" ctrl_word: "<< pdo_channels_info_[index].default_value);

      if (is_operational_) {
        if (fault_reset_command_interface_index_ >= 0) {
          if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
            last_fault_reset_command_ = false;
          }
          if (last_fault_reset_command_ == false &&
              command_interface_ptr_->at(fault_reset_command_interface_index_) != 0 &&
              !std::isnan(command_interface_ptr_->at(fault_reset_command_interface_index_)))
          {
            last_fault_reset_command_ = true;
            fault_reset_ = true;
          }
        }

        if (auto_state_transitions_) {
          pdo_channels_info_[index].default_value = transition(
                                                      state_,
                                                      pdo_channels_info_[index].ec_read(domain_address));
        }

//        if (home_sequence_==HOMING_SELECTED && state_==STATE_OPERATION_ENABLED)
//        {
//          uint16_t control_word_tmp = pdo_channels_info_[index].default_value;
//          RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"), "prima "  <<  std::bitset<16>(control_word_tmp)<< " " << pdo_channels_info_[index].default_value);
//          control_word_tmp = control_word_tmp | 0b00010000;
//          pdo_channels_info_[index].default_value  = control_word_tmp;
//          home_sequence_=HOMING_IMPULSE;
//          RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"), "attivo bit 4 "  <<  std::bitset<16>(control_word_tmp));
//        }
//        else if (home_sequence_==HOMING_IMPULSE && state_==STATE_OPERATION_ENABLED)
//        {
//          uint16_t control_word_tmp = pdo_channels_info_[index].default_value;
//          control_word_tmp = control_word_tmp & 0b11101111;
//          pdo_channels_info_[index].default_value  = control_word_tmp;
//          RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"), "disattivo bit 4 "  <<  std::bitset<16>(control_word_tmp));
//          home_sequence_=HOMING_ACTIVE;
//        }


      }
    }

//      mode_of_operation_display_=ModeOfOperation::MODE_PROFILED_POSITION;
//    mode_of_operation_display_=ModeOfOperation::MODE_PROFILED_VELOCITY;

    //  RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),
    //                      "index: " << std::hex << pdo_channels_info_[index].index );

    if (pdo_channels_info_[index].index == CiA402D_RPDO_VELOCITY) {


//        RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),
//                            " default_value: " << pdo_channels_info_[index].default_value <<
//                            " last_value: " << pdo_channels_info_[index].last_value
//                            );


//      pdo_channels_info_[index].override_command =
//          (mode_of_operation_display_ != ModeOfOperation::MODE_PROFILED_VELOCITY) ? true : false;
    }

    // setup current position as default position
    if (pdo_channels_info_[index].index == CiA402D_RPDO_POSITION) {

      //    RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),
      //                        "mode_of_operation_display_: " << (int)mode_of_operation_display_);

      if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE) {
        pdo_channels_info_[index].default_value =
            pdo_channels_info_[index].factor * last_position_ +
            pdo_channels_info_[index].offset;


        //        RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),
        //                            " last_position_: " << last_position_ <<
        //                            " default_value: " << pdo_channels_info_[index].default_value <<
        //                            " last_value: " << pdo_channels_info_[index].last_value
        //                            );

      }
      pdo_channels_info_[index].override_command =
          ( (mode_of_operation_display_ != ModeOfOperation::MODE_PROFILED_POSITION) &&
            (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION)) ? true : false;
      //(mode_of_operation_display_ != ModeOfOperation::MODE_PROFILED_VELOCITY) ? true : false;
    }


    // setup mode of operation
    if (pdo_channels_info_[index].index == CiA402D_RPDO_MODE_OF_OPERATION) {
      if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10) {
        pdo_channels_info_[index].default_value = mode_of_operation_;

//        if (mode_of_operation_ != MODE_HOMING)
//          home_sequence_ = HOMING_NO_MODE;
//        else if (home_sequence_ == HOMING_NO_MODE)
//        {
//          RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"), "pronto per l'homing" );
//          home_sequence_ = HOMING_SELECTED;
//        }
      }

    }

    pdo_channels_info_[index].ec_update(domain_address);

    // get mode_of_operation_display_
    if (pdo_channels_info_[index].index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY) {
      if (mode_of_operation_display_ != pdo_channels_info_[index].last_value)
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),"mode of operation display = " << pdo_channels_info_[index].last_value);
      mode_of_operation_display_ = pdo_channels_info_[index].last_value;

      if (mode_of_operation_display_ != MODE_HOMING)
        home_sequence_ = HOMING_NO_MODE;
      else if (home_sequence_ == HOMING_NO_MODE)
      {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"), "pronto per l'homing" );
        home_sequence_ = HOMING_SELECTED;
      }

    }

    //  if (pdo_channels_info_[index].index == CiA402D_TPDO_POSITION) {
    if (pdo_channels_info_[index].index == CiA402D_TPDO_VELOCITY) {
      //    RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),
      //                          "last_position__: " << last_position_ );
      last_position_ = pdo_channels_info_[index].last_value;
      //    RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),
      //                        "last_position__: " << last_position_ );
    }

    // Special case: StatusWord
    if (pdo_channels_info_[index].index == CiA402D_TPDO_STATUSWORD) {
      if (status_word_ != pdo_channels_info_[index].last_value)
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),"status = " << std::bitset<16>(pdo_channels_info_[index].last_value));
      status_word_ = pdo_channels_info_[index].last_value;
    }


    // CHECK FOR STATE CHANGE
    if (index == all_channels_.size() - 1) {  // if last entry  in domain
      if (status_word_ != last_status_word_) {
        state_ = deviceState(status_word_);
        if (state_ != last_state_) {
          std::cout << "STATE: " << DEVICE_STATE_STR.at(state_)
                    << " ,with status word :" << status_word_ << std::endl;
        }
      }
      initialized_ = ((state_ == STATE_OPERATION_ENABLED) &&
                      (last_state_ == STATE_OPERATION_ENABLED)) ? true : false;

      last_status_word_ = status_word_;
      last_state_ = state_;
      counter_++;
    }
  }

  bool EcCiA402Drive::setupSlave(
      std::unordered_map<std::string, std::string> slave_paramters,
      std::vector<double> * state_interface,
      std::vector<double> * command_interface)
  {
    state_interface_ptr_ = state_interface;
    command_interface_ptr_ = command_interface;
    paramters_ = slave_paramters;

    if (paramters_.find("slave_config") != paramters_.end()) {
      if (!setup_from_config_file(paramters_["slave_config"])) {
        return false;
      }
    } else {
      std::cerr << "EcCiA402Drive: failed to find 'slave_config' tag in URDF." << std::endl;
      return false;
    }

    setup_interface_mapping();
    setup_syncs();

    if (paramters_.find("mode_of_operation") != paramters_.end()) {
      mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);

      RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),
                          "Mode of Operation: " <<  std::to_string(mode_of_operation_));
    }

    if (paramters_.find("command_interface/reset_fault") != paramters_.end()) {
      fault_reset_command_interface_index_ = std::stoi(paramters_["command_interface/reset_fault"]);
    }

    return true;
  }

  bool EcCiA402Drive::setup_from_config(YAML::Node drive_config)
  {
    if (!GenericEcSlave::setup_from_config(drive_config)) {return false;}
    // additional configuration parameters for CiA402 Drives
    if (drive_config["auto_fault_reset"]) {
      auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
    }
    if (drive_config["auto_state_transitions"]) {
      auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
    }
    return true;
  }

  bool EcCiA402Drive::setup_from_config_file(std::string config_file)
  {
    // Read drive configuration from YAML file
    try {
      slave_config_ = YAML::LoadFile(config_file);
    } catch (const YAML::ParserException & ex) {
      std::cerr << "EcCiA402Drive: failed to load drive configuration: " << ex.what() << std::endl;
      return false;
    } catch (const YAML::BadFile & ex) {
      std::cerr << "EcCiA402Drive: failed to load drive configuration: " << ex.what() << std::endl;
      return false;
    }
    if (!setup_from_config(slave_config_)) {
      return false;
    }
    return true;
  }

  /** returns device state based upon the status_word */
  DeviceState EcCiA402Drive::deviceState(uint16_t status_word)
  {
    if ((status_word & 0b01001111) == 0b00000000) {
      return STATE_NOT_READY_TO_SWITCH_ON;
    } else if ((status_word & 0b01001111) == 0b01000000) {
      return STATE_SWITCH_ON_DISABLED;
    } else if ((status_word & 0b01101111) == 0b00100001) {
      return STATE_READY_TO_SWITCH_ON;
    } else if ((status_word & 0b01101111) == 0b00100011) {
      return STATE_SWITCH_ON;
    } else if ((status_word & 0b01101111) == 0b00100111) {
      return STATE_OPERATION_ENABLED;
    } else if ((status_word & 0b01101111) == 0b00000111) {
      return STATE_QUICK_STOP_ACTIVE;
    } else if ((status_word & 0b01001111) == 0b00001111) {
      return STATE_FAULT_REACTION_ACTIVE;
    } else if ((status_word & 0b01001111) == 0b00001000) {
      return STATE_FAULT;
    }
    return STATE_UNDEFINED;
  }

  /** returns the control word that will take device from state to next desired state */
  uint16_t EcCiA402Drive::transition(DeviceState state, uint16_t control_word)
  {
    //    RCLCPP_FATAL_STREAM(rclcpp::get_logger("DS402"),
    //                        "State: " << std::hex << state << ", Control word: "<< control_word);

    switch (state) {
      case STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
        return control_word;
      case STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
        return control_word;
      case STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
        return (control_word & 0b01111110) | 0b00000110;
      case STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
        return (control_word & 0b01110111) | 0b00000111;
      case STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
        return (control_word & 0b01111111) | 0b00001111;
      case STATE_OPERATION_ENABLED:         // -> GOOD
        return control_word;
      case STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
        return (control_word & 0b01111111) | 0b00001111;
      case STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
        return control_word;
      case STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
        if (auto_fault_reset_ || fault_reset_) {
          fault_reset_ = false;

          return (control_word & 0b11111111) | 0b10000000;     // automatic reset
        } else {
          return control_word;
        }
      default:
        break;
    }
    return control_word;
  }

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave)
