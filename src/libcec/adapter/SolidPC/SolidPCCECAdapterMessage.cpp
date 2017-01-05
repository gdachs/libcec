/*
 * This file is part of the libCEC(R) library.
 *
 * libCEC(R) is Copyright (C) 2011-2015 Pulse-Eight Limited.  All rights reserved.
 * libCEC(R) is an original work, containing original code.
 *
 * libCEC(R) is a trademark of Pulse-Eight Limited.
 *
 * This program is dual-licensed; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301  USA
 *
 *
 * Alternatively, you can license this library under a commercial license,
 * please contact Pulse-Eight Licensing for more information.
 *
 * For more information contact:
 * Pulse-Eight Licensing       <license@pulse-eight.com>
 *     http://www.pulse-eight.com/
 *     http://www.pulse-eight.net/
 */

#include "env.h"
#include "SolidPCCECAdapterMessage.h"

#include "LibCEC.h"

using namespace CEC;
using namespace P8PLATFORM;

namespace SolidPCCEC
{

static const char hex_asc[] = "0123456789abcdef";

CCECAdapterMessage::CCECAdapterMessage(void)
{
  Clear();
}

CCECAdapterMessage::CCECAdapterMessage(const cec_command &command,
    uint8_t iLineTimeout /* = 3 */)
{
  Clear();

  PushBack('!');
  PushBack('x');
  PushBackHexLo(command.destination);
  PushBackHex(command.opcode);
  PushBackHex(command.parameters);
  PushBack('~');

  // set timeout
  transmit_timeout = command.transmit_timeout;

  lineTimeout = iLineTimeout;
}

std::string CCECAdapterMessage::ToString(void) const
{
  std::string strMsg;
  if (Size() == 0)
  {
    strMsg = "empty message";
  }
  else
  {
    strMsg = ToString(Message());

    switch (Message())
    {
    case MSGCODE_FRAME_START:
      if (Size() >= 3)
        strMsg += StringUtils::Format(
            " initiator:%1x destination:%1x", Initiator(),
            Destination());
      break;
    case MSGCODE_FRAME_DATA:
      if (Size() >= 3)
        strMsg += StringUtils::Format(" %02x", At(2));
      break;
    default:
      if (Size() >= 2
          && (Message() == MSGCODE_COMMAND_ACCEPTED
              || Message() == MSGCODE_COMMAND_REJECTED))
        strMsg += StringUtils::Format(": %s",
            ToString((cec_adapter_messagecode) At(2)));
      else
      {
        for (uint8_t iPtr = 2; iPtr < Size(); iPtr++)
          if (At(iPtr) != MSGEND)
            strMsg += StringUtils::Format(" %02x", At(iPtr));
      }
      break;
    }
  }

  return std::string(strMsg.c_str());
}

const char *CCECAdapterMessage::ToString(cec_adapter_messagecode msgCode)
{
  switch (msgCode)
  {
  case MSGCODE_NOTHING:
    return "NOTHING";
  case MSGCODE_PING:
    return "PING";
  case MSGCODE_TIMEOUT_ERROR:
    return "TIMEOUT";
  case MSGCODE_HIGH_ERROR:
    return "HIGH_ERROR";
  case MSGCODE_LOW_ERROR:
    return "LOW_ERROR";
  case MSGCODE_FRAME_START:
    return "FRAME_START";
  case MSGCODE_FRAME_DATA:
    return "FRAME_DATA";
  case MSGCODE_RECEIVE_FAILED:
    return "RECEIVE_FAILED";
  case MSGCODE_COMMAND_ACCEPTED:
    return "COMMAND_ACCEPTED";
  case MSGCODE_COMMAND_REJECTED:
    return "COMMAND_REJECTED";
  case MSGCODE_SET_ACK_MASK:
    return "SET_ACK_MASK";
  case MSGCODE_TRANSMIT:
    return "TRANSMIT";
  case MSGCODE_TRANSMIT_EOM:
    return "TRANSMIT_EOM";
  case MSGCODE_TRANSMIT_IDLETIME:
    return "TRANSMIT_IDLETIME";
  case MSGCODE_TRANSMIT_ACK_POLARITY:
    return "CEC transmission";
  case MSGCODE_TRANSMIT_LINE_TIMEOUT:
    return "TRANSMIT_LINE_TIMEOUT";
  case MSGCODE_TRANSMIT_SUCCEEDED:
    return "TRANSMIT_SUCCEEDED";
  case MSGCODE_TRANSMIT_FAILED_LINE:
    return "TRANSMIT_FAILED_LINE";
  case MSGCODE_TRANSMIT_FAILED_ACK:
    return "TRANSMIT_FAILED_ACK";
  case MSGCODE_TRANSMIT_FAILED_TIMEOUT_DATA:
    return "TRANSMIT_FAILED_TIMEOUT_DATA";
  case MSGCODE_TRANSMIT_FAILED_TIMEOUT_LINE:
    return "TRANSMIT_FAILED_TIMEOUT_LINE";
  case MSGCODE_FIRMWARE_VERSION:
    return "FIRMWARE_VERSION";
  case MSGCODE_START_BOOTLOADER:
    return "START_BOOTLOADER";
  case MSGCODE_FRAME_EOM:
    return "FRAME_EOM";
  case MSGCODE_FRAME_ACK:
    return "FRAME_ACK";
  case MSGCODE_GET_BUILDDATE:
    return "GET_BUILDDATE";
  case MSGCODE_SET_CONTROLLED:
    return "SET_CONTROLLED";
  case MSGCODE_GET_AUTO_ENABLED:
    return "GET_AUTO_ENABLED";
  case MSGCODE_SET_AUTO_ENABLED:
    return "SET_AUTO_ENABLED";
  case MSGCODE_GET_DEFAULT_LOGICAL_ADDRESS:
    return "GET_DEFAULT_LOGICAL_ADDRESS";
  case MSGCODE_SET_DEFAULT_LOGICAL_ADDRESS:
    return "SET_DEFAULT_LOGICAL_ADDRESS";
  case MSGCODE_GET_LOGICAL_ADDRESS_MASK:
    return "GET_LOGICAL_ADDRESS_MASK";
  case MSGCODE_SET_LOGICAL_ADDRESS_MASK:
    return "SET_LOGICAL_ADDRESS_MASK";
  case MSGCODE_GET_PHYSICAL_ADDRESS:
    return "GET_PHYSICAL_ADDRESS";
  case MSGCODE_SET_PHYSICAL_ADDRESS:
    return "SET_PHYSICAL_ADDRESS";
  case MSGCODE_GET_DEVICE_TYPE:
    return "GET_DEVICE_TYPE";
  case MSGCODE_SET_DEVICE_TYPE:
    return "SET_DEVICE_TYPE";
  case MSGCODE_GET_HDMI_VERSION:
    return "GET_HDMI_VERSION";
  case MSGCODE_SET_HDMI_VERSION:
    return "SET_HDMI_VERSION";
  case MSGCODE_GET_OSD_NAME:
    return "GET_OSD_NAME";
  case MSGCODE_SET_OSD_NAME:
    return "SET_OSD_NAME";
  case MSGCODE_WRITE_EEPROM:
    return "WRITE_EEPROM";
  case MSGCODE_GET_ADAPTER_TYPE:
    return "GET_ADAPTER_TYPE";
  default:
    break;
  }

  return "unknown";
}

uint8_t CCECAdapterMessage::operator[](uint8_t pos) const
{
  return pos < packet.size ? packet[pos] : 0;
}

uint8_t CCECAdapterMessage::At(uint8_t pos) const
{
  return pos < packet.size ? packet[pos] : 0;
}

uint8_t CCECAdapterMessage::Size(void) const
{
  return packet.size;
}

bool CCECAdapterMessage::IsEmpty(void) const
{
  return packet.IsEmpty();
}

void CCECAdapterMessage::Clear(void)
{
  state = ADAPTER_MESSAGE_STATE_UNKNOWN;
  transmit_timeout = CEC_DEFAULT_TRANSMIT_TIMEOUT;
  response.Clear();
  packet.Clear();
  lineTimeout = 3;
  bNextByteIsEscaped = false;
  bFireAndForget = false;
}

void CCECAdapterMessage::Shift(uint8_t iShiftBy)
{
  packet.Shift(iShiftBy);
}

int8_t CCECAdapterMessage::HexToBin(uint8_t ch) const
{
    if ((ch >= '0') && (ch <= '9'))
        return ch - '0';
    ch = tolower(ch);
    if ((ch >= 'a') && (ch <= 'f'))
        return ch - 'a' + 10;
    return -1;
}

void CCECAdapterMessage::Append(CCECAdapterMessage &data)
{
  Append(data.packet);
}

void CCECAdapterMessage::Append(cec_datapacket &data)
{
  for (uint8_t iPtr = 0; iPtr < data.size; iPtr++)
    PushBack(data[iPtr]);
}

void CCECAdapterMessage::PushBack(uint8_t byte)
{
  packet.PushBack(byte);
}

void CCECAdapterMessage::PushBackHexLo(uint8_t byte)
{
  packet.PushBack(hex_asc[byte & 0x0f]);
}

void CCECAdapterMessage::PushBackHex(uint8_t byte)
{
  packet.PushBack(hex_asc[byte >> 4]);
  packet.PushBack(hex_asc[byte & 0x0f]);
}

void CCECAdapterMessage::PushBackHex(const cec_datapacket &data)
{
  for (uint8_t iPtr = 0; iPtr < data.size; iPtr++)
    PushBackHex(data[iPtr]);
}

bool CCECAdapterMessage::PushReceivedByte(uint8_t byte)
{
  if (byte == '?' && HasStartMessage())
  {
    //TODO CLibCEC::AddLog(CEC_LOG_WARNING, "received message start before message end, removing previous buffer contents");
    Clear();
  }
  PushBack(byte);

  return byte == '\n';
}

bool CCECAdapterMessage::StartsWith(const char *str)
{
   return !memcmp(packet.data, str, strlen(str));
}

cec_adapter_messagecode CCECAdapterMessage::Message(void) const
{
  return
      packet.size >= 2 ?
          (cec_adapter_messagecode) (packet.At(1)
              & ~(MSGCODE_FRAME_EOM | MSGCODE_FRAME_ACK)) :
          MSGCODE_NOTHING;
}

cec_adapter_messagecode CCECAdapterMessage::ResponseTo(void) const
{
  return
      packet.size >= 3 ?
          (cec_adapter_messagecode) (packet.At(2)
              & ~(MSGCODE_FRAME_EOM | MSGCODE_FRAME_ACK)) :
          MSGCODE_NOTHING;
}

bool CCECAdapterMessage::IsTransmission(void) const
{
  cec_adapter_messagecode msgCode = Message();
  return msgCode == MSGCODE_FRAME_ACK || msgCode == MSGCODE_FRAME_DATA
      || msgCode == MSGCODE_FRAME_EOM || msgCode == MSGCODE_FRAME_START
      || msgCode == MSGCODE_HIGH_ERROR || msgCode == MSGCODE_LOW_ERROR
      || msgCode == MSGCODE_RECEIVE_FAILED
      || msgCode == MSGCODE_TRANSMIT_ACK_POLARITY
      || msgCode == MSGCODE_TRANSMIT_EOM
      || msgCode == MSGCODE_TRANSMIT_FAILED_ACK
      || msgCode == MSGCODE_TRANSMIT_FAILED_LINE
      || msgCode == MSGCODE_TRANSMIT_FAILED_TIMEOUT_DATA
      || msgCode == MSGCODE_TRANSMIT_FAILED_TIMEOUT_LINE
      || msgCode == MSGCODE_TRANSMIT_LINE_TIMEOUT
      || msgCode == MSGCODE_TRANSMIT_SUCCEEDED;
}

bool CCECAdapterMessage::MessageCodeIsError(const cec_adapter_messagecode code)
{
  return (code == MSGCODE_HIGH_ERROR || code == MSGCODE_LOW_ERROR
      || code == MSGCODE_RECEIVE_FAILED || code == MSGCODE_COMMAND_REJECTED
      || code == MSGCODE_TRANSMIT_LINE_TIMEOUT
      || code == MSGCODE_TRANSMIT_FAILED_LINE
      || code == MSGCODE_TRANSMIT_FAILED_ACK
      || code == MSGCODE_TRANSMIT_FAILED_TIMEOUT_DATA
      || code == MSGCODE_TRANSMIT_FAILED_TIMEOUT_LINE);
}

bool CCECAdapterMessage::IsError(void) const
{
  return MessageCodeIsError(Message());
}

bool CCECAdapterMessage::ReplyIsError(void) const
{
  return MessageCodeIsError(Reply());
}

bool CCECAdapterMessage::NeedsRetry(void) const
{
  return Reply() == MSGCODE_NOTHING || Reply() == MSGCODE_RECEIVE_FAILED
      || Reply() == MSGCODE_TIMEOUT_ERROR
      || Reply() == MSGCODE_TRANSMIT_FAILED_LINE
      || Reply() == MSGCODE_TRANSMIT_FAILED_TIMEOUT_DATA
      || Reply() == MSGCODE_TRANSMIT_FAILED_TIMEOUT_LINE
      || Reply() == MSGCODE_TRANSMIT_LINE_TIMEOUT;
}

cec_logical_address CCECAdapterMessage::Initiator(void) const
{
  cec_logical_address address;

  return
      (packet.size >= 7)
          && ((address = (cec_logical_address) HexToBin(packet.At(5))) > 0) ?
          address : CECDEVICE_UNKNOWN;
}

cec_logical_address CCECAdapterMessage::Destination(void) const
{
  cec_logical_address address;

  return
      (packet.size >= 7)
          && ((address = (cec_logical_address) HexToBin(packet.At(6))) > 0) ?
          address : CECDEVICE_UNKNOWN;
}

bool CCECAdapterMessage::HasStartMessage(void) const
{
  return packet.size >= 1 && packet.At(0) == '?';
}

bool CCECAdapterMessage::PushToCecCommand(cec_command &command) const
{
  // empty message
  if (IsEmpty())
    return false;

  if (StartsWith("?REC"))
  {
    for (int i = 4; i < Size(); ++i)
    {
      if (!isxdigit(At(i)))
        continue;
      if (i + 1 < Size() && isxdigit(At(i + 1)))
      {

      }
    }
    command.Clear();
    if (Size() >= 3)
    {
      command.initiator = Initiator();
      command.destination = Destination();
    }
    return !IsError();
  }
  else if (msgCode == MSGCODE_FRAME_DATA)
  {
    if (Size() >= 3)
    {
      command.PushBack(At(2));
    }
    return !IsError();
  }

  return false;
}

cec_adapter_messagecode CCECAdapterMessage::Reply(void) const
{
  return
      response.size >= 2 ?
          (cec_adapter_messagecode) (response.At(1)
              & ~(MSGCODE_FRAME_EOM | MSGCODE_FRAME_ACK)) :
          MSGCODE_NOTHING;
}
}
