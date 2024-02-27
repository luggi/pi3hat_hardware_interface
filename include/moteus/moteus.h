// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "moteus_optional.h"
#include "moteus_protocol.h"
#include "moteus_tokenizer.h"

namespace mjbots {
namespace moteus {

/// * This file has been modified from the MJBots Github repository to remove
/// * any dependency on the moteus_transport class. This is because the
/// * the transport conflicts with the Pi3HatHardwareInterface class.

/// This is the primary interface to a moteus controller.  One
/// instance of this class should be created per controller that is
/// commanded or monitored.
///
/// The primary control functions each have 3 possible forms:
///
///  1. A "Make" variant which constructs a CanFdFrame to be used in a
///     later call to Transport::Cycle.
/// 
/// ! The following 2 forms have been removed.
///  2. A "Set" variant which sends a command to the controller and
///     waits for a response in a blocking manner.
///
///  3. An "Async" variant which starts the process of sending a
///     command and potentially waiting for a response.  When this
///     operation is finished, a user-provided callback is invoked.
///     This callback may be called either:
///      a) from an arbitrary thread
///      b) recursively from the calling thread before returning
///
/// While any async operation is outstanding, it is undefined behavior
/// to start another async operation or execute a blocking operation.
class Controller {
 public:
  struct Options {
    // The ID of the servo to communicate with.
    int id = 1;

    // The source ID to use for the commanding node (i.e. the host or
    // master).
    int source = 0;

    // Which CAN bus to send commands on and look for responses on.
    // This may not be used on all transports.
    int bus = 0;

    // For each possible primary command, the resolution for all
    // command fields has a default set at construction time.
    Query::Format query_format;
    PositionMode::Format position_format;
    VFOCMode::Format vfoc_format;
    CurrentMode::Format current_format;
    StayWithinMode::Format stay_within_format;

    // Use the given prefix for all CAN IDs.
    uint32_t can_prefix = 0x0000;

    // Request the configured set of registers as a query with every
    // command.
    bool default_query = true;

    int64_t diagnostic_retry_sleep_ns = 200000;

    Options() {}
  };

  Controller(const Options& options = {}) : options_(options) {

    WriteCanData query_write(&query_frame_);
    query_reply_size_ = Query::Make(&query_write, options_.query_format);
  }

  const Options& options() const { return options_; }

  struct Result {
    CanFdFrame frame;
    Query::Result values;
  };


  /////////////////////////////////////////
  // Query

  CanFdFrame MakeQuery(const Query::Format* format_override = nullptr) {
    // We force there to always be an override for the query format,
    // because if we're directly asking for a query, we should get it
    // no matter the Options::default_query state.
    return MakeFrame(EmptyMode(), {}, {},
                     format_override == nullptr ?
                     &options_.query_format : format_override);
  }

  /////////////////////////////////////////
  // StopMode

  CanFdFrame MakeStop(const Query::Format* query_override = nullptr) {
    return MakeFrame(StopMode(), {}, {}, query_override);
  }

  /////////////////////////////////////////
  // BrakeMode

  CanFdFrame MakeBrake(const Query::Format* query_override = nullptr) {
    return MakeFrame(BrakeMode(), {}, {}, query_override);
  }

  /////////////////////////////////////////
  // PositionMode

  CanFdFrame MakePosition(const PositionMode::Command& cmd,
                          const PositionMode::Format* command_override = nullptr,
                          const Query::Format* query_override = nullptr) {
    return MakeFrame(
        PositionMode(), cmd,
        (command_override == nullptr ?
         options_.position_format : *command_override),
        query_override);
  }

  /////////////////////////////////////////
  // VFOCMode

  CanFdFrame MakeVFOC(const VFOCMode::Command& cmd,
                      const VFOCMode::Format* command_override = nullptr,
                      const Query::Format* query_override = nullptr) {
    return MakeFrame(
        VFOCMode(), cmd,
        command_override == nullptr ? options_.vfoc_format : *command_override,
        query_override);
  }

  /////////////////////////////////////////
  // CurrentMode

  CanFdFrame MakeCurrent(const CurrentMode::Command& cmd,
                         const CurrentMode::Format* command_override = nullptr,
                         const Query::Format* query_override = nullptr) {
    return MakeFrame(CurrentMode(), cmd,
                     (command_override == nullptr ?
                      options_.current_format : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // StayWithinMode

  CanFdFrame MakeStayWithin(
      const StayWithinMode::Command& cmd,
      const StayWithinMode::Format* command_override = nullptr,
      const Query::Format* query_override = nullptr) {
    return MakeFrame(StayWithinMode(), cmd,
                     (command_override == nullptr ?
                      options_.stay_within_format : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // OutputNearest

  CanFdFrame MakeOutputNearest(const OutputNearest::Command& cmd,
                               const OutputNearest::Format* command_override = nullptr,
                               const Query::Format* query_override = nullptr) {
    return MakeFrame(OutputNearest(), cmd,
                     (command_override == nullptr ?
                      OutputNearest::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // OutputExact

  CanFdFrame MakeOutputExact(const OutputExact::Command& cmd,
                             const OutputExact::Format* command_override = nullptr,
                             const Query::Format* query_override = nullptr) {
    return MakeFrame(OutputExact(), cmd,
                     (command_override == nullptr ?
                      OutputExact::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // RequireReindex

  CanFdFrame MakeRequireReindex(const RequireReindex::Command& cmd = {},
                                const RequireReindex::Format* command_override = nullptr,
                                const Query::Format* query_override = nullptr) {
    return MakeFrame(RequireReindex(), cmd,
                     (command_override == nullptr ?
                      RequireReindex::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // RecapturePositionVelocity

  CanFdFrame MakeRecapturePositionVelocity(
      const RecapturePositionVelocity::Command& cmd = {},
      const RecapturePositionVelocity::Format* command_override = nullptr,
      const Query::Format* query_override = nullptr) {
    return MakeFrame(RecapturePositionVelocity(), cmd,
                     (command_override == nullptr ?
                      RecapturePositionVelocity::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // ClockTrim

  CanFdFrame MakeClockTrim(const ClockTrim::Command& cmd,
                           const ClockTrim::Format* command_override = nullptr,
                           const Query::Format* query_override = nullptr) {
    return MakeFrame(ClockTrim(), cmd,
                     (command_override == nullptr ?
                      ClockTrim::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // Schema version checking

  CanFdFrame MakeSchemaVersionQuery() {
    GenericQuery::Format query;
    query.values[0].register_number = Register::kRegisterMapVersion;
    query.values[0].resolution = kInt32;

    return MakeFrame(GenericQuery(), {}, query);
  }

  //////////////////////////////////////////////////

  static std::string FinalName(const std::string& name) {
    const size_t pos = name.find_last_of("/");
    if (pos != std::string::npos) { return name.substr(pos + 1); }
    return name;
  }


 private:
  
  static void CheckRegisterMapVersion(const Result& result) {
    if (result.values.extra[0].register_number !=
        Register::kRegisterMapVersion) {
      throw std::runtime_error("Malformed response to schema version query");
    }

    const auto int_version = static_cast<int>(result.values.extra[0].value);
    if (kCurrentRegisterMapVersion != int_version) {
      std::ostringstream ostr;
      ostr << "Register map version mismatch device is "
           << int_version
           << " but library requires "
           << kCurrentRegisterMapVersion;

      throw std::runtime_error(ostr.str());
    }
  }

  Optional<Result> FindResult(const std::vector<CanFdFrame>& replies) const {
    // Pick off the last reply we got from our target ID.
    for (auto it = replies.rbegin(); it != replies.rend(); ++it) {
      if (it->source == options_.id &&
          it->destination == options_.source &&
          it->can_prefix == options_.can_prefix) {

        Result result;
        result.frame = *it;
        result.values = Query::Parse(it->data, it->size);
        return result;
      }
    }

    // We didn't get anything.
    return {};
  }

  enum ReplyMode {
    kNoReply,
    kReplyRequired,
  };

  CanFdFrame DefaultFrame(ReplyMode reply_mode = kReplyRequired) {
    CanFdFrame result;
    result.destination = options_.id;
    result.reply_required = (reply_mode == kReplyRequired);

    result.arbitration_id =
        (result.destination) |
        (result.source << 8) |
        (result.reply_required ? 0x8000 : 0x0000) |
        (options_.can_prefix << 16);
    result.bus = options_.bus;

    return result;
  }

  template <typename CommandType>
  CanFdFrame MakeFrame(const CommandType&,
                       const typename CommandType::Command& cmd,
                       const typename CommandType::Format& fmt,
                       const Query::Format* query_format_override = nullptr) {
    auto result = DefaultFrame(
        query_format_override != nullptr ? kReplyRequired :
        options_.default_query ? kReplyRequired : kNoReply);

    WriteCanData write_frame(result.data, &result.size);
    result.expected_reply_size = CommandType::Make(&write_frame, cmd, fmt);

    if (query_format_override) {
      result.expected_reply_size =
          Query::Make(&write_frame, *query_format_override);
    } else if (options_.default_query) {
      std::memcpy(&result.data[result.size],
                  &query_frame_.data[0],
                  query_frame_.size);
      result.size += query_frame_.size;
      result.expected_reply_size = query_reply_size_;
    }

    return result;
  }

  const Options options_;
  CanData query_frame_;
  uint8_t query_reply_size_ = 0;
  CanFdFrame output_frame_;

  // This is a member variable so we can avoid re-allocating it on
  // every call.
  std::vector<CanFdFrame> single_command_replies_;
};


}  // namespace moteus
}  // namespace mjbots
