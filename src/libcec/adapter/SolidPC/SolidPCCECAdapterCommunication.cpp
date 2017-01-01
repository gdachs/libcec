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
#include "SolidPCCECAdapterCommunication.h"

#include "SolidPCCECAdapterCommands.h"
#include "SolidPCCECAdapterMessageQueue.h"
#include "SolidPCCECAdapterMessage.h"
#include "SolidPCCECAdapterDetection.h"
#include "platform/sockets/serialport.h"
#include <p8-platform/util/timeutils.h>
#include <p8-platform/util/util.h>
#include "platform/util/edid.h"
#include "platform/adl/adl-edid.h"
#include "platform/nvidia/nv-edid.h"
#include "platform/drm/drm-edid.h"
#include "LibCEC.h"
#include "CECProcessor.h"

using namespace CEC;
using namespace P8PLATFORM;

// firmware version 1
#define CEC_LATEST_ADAPTER_FW_VERSION 1

#define LIB_CEC m_callback->GetLib()

namespace SolidPCCEC
{

CSolidPCCECAdapterCommunication::CSolidPCCECAdapterCommunication(
    IAdapterCommunicationCallback *callback, const char *strPort,
    uint16_t iBaudRate /* = CEC_SERIAL_DEFAULT_BAUDRATE */) :
    IAdapterCommunication(callback), m_port(NULL), m_iLineTimeout(0), m_lastPollDestination(
        CECDEVICE_UNKNOWN), m_bInitialised(false), m_commands(NULL), m_adapterMessageQueue(
        NULL)
{
  m_logicalAddresses.Clear();
  for (unsigned int iPtr = CECDEVICE_TV; iPtr < CECDEVICE_BROADCAST; iPtr++)
    m_bWaitingForAck[iPtr] = false;
  m_port = new CSerialPort(strPort, iBaudRate);
  m_commands = new CSolidPCCECAdapterCommands(this);
}

CSolidPCCECAdapterCommunication::~CSolidPCCECAdapterCommunication(void)
{
  Close();
  SAFE_DELETE(m_commands);
  SAFE_DELETE(m_adapterMessageQueue);
  SAFE_DELETE(m_port);
}

void CSolidPCCECAdapterCommunication::ResetMessageQueue(void)
{
  SAFE_DELETE(m_adapterMessageQueue);
  m_adapterMessageQueue = new CCECAdapterMessageQueue(this);
  m_adapterMessageQueue->CreateThread();
}

bool CSolidPCCECAdapterCommunication::Open(
    uint32_t iTimeoutMs /* = CEC_DEFAULT_CONNECT_TIMEOUT */,
    bool bSkipChecks /* = false */, bool bStartListening /* = true */)
{
  bool bConnectionOpened(false);
  {
    CLockObject lock(m_mutex);

    /* we need the port settings here */
    if (!m_port)
    {
      LIB_CEC->AddLog(CEC_LOG_ERROR, "port is NULL");
      return bConnectionOpened;
    }

    /* return true when the port is already open */
    if (IsOpen())
    {
      LIB_CEC->AddLog(CEC_LOG_WARNING, "port is already open");
      return true;
    }

    ResetMessageQueue();

    /* try to open the connection */
    std::string strError;
    CTimeout timeout(iTimeoutMs);
    while (!bConnectionOpened && timeout.TimeLeft() > 0)
    {
      if ((bConnectionOpened = m_port->Open(timeout.TimeLeft())) == false)
      {
        strError = StringUtils::Format("error opening serial port '%s': %s",
            m_port->GetName().c_str(), m_port->GetError().c_str());
        Sleep(250);
      }
      /* and retry every 250ms until the timeout passed */
    }

    /* return false when we couldn't connect */
    if (!bConnectionOpened)
    {
      LIB_CEC->AddLog(CEC_LOG_ERROR, strError.c_str());

      if (m_port->GetErrorNumber() == EACCES)
      {
        libcec_parameter param;
        param.paramType = CEC_PARAMETER_TYPE_STRING;
        param.paramData = (void*) "No permission to open the device";
        LIB_CEC->Alert(CEC_ALERT_PERMISSION_ERROR, param);
      }
      else if (m_port->GetErrorNumber() == EBUSY)
      {
        libcec_parameter param;
        param.paramType = CEC_PARAMETER_TYPE_STRING;
        param.paramData =
            (void*) "The serial port is busy. Only one program can access the device directly.";
        LIB_CEC->Alert(CEC_ALERT_PORT_BUSY, param);
      }
      return false;
    }

    LIB_CEC->AddLog(CEC_LOG_DEBUG,
        "connection opened, clearing any previous input and waiting for active transmissions to end before starting");
    ClearInputBytes();
  }

  // always start by setting the ackmask to 0, to clear previous values
  cec_logical_addresses addresses;
  addresses.Clear();
  SetLogicalAddresses(addresses);

  if (!CreateThread())
  {
    bConnectionOpened = false;
    LIB_CEC->AddLog(CEC_LOG_ERROR, "could not create a communication thread");
  }
  else if (!bSkipChecks && !CheckAdapter())
  {
    bConnectionOpened = false;
    LIB_CEC->AddLog(CEC_LOG_ERROR, "the adapter failed to pass basic checks");
  }

  if (!bConnectionOpened || !bStartListening)
    StopThread(0);

  return bConnectionOpened;
}

void CSolidPCCECAdapterCommunication::Close(void)
{
  /* stop the reader thread */
  StopThread(0);

  CLockObject lock(m_mutex);

  /* set the ackmask to 0 before closing the connection */
  if (IsOpen() && m_port->GetErrorNumber() == 0)
  {
    LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s - closing the connection", __FUNCTION__);
    cec_logical_addresses addresses;
    addresses.Clear();
    SetLogicalAddresses(addresses);
    if (m_commands->GetFirmwareVersion() >= 2)
      SetControlledMode(false);
  }

  m_adapterMessageQueue->Clear();

  /* close and delete the com port connection */
  if (m_port)
    m_port->Close();
}

cec_adapter_message_state CSolidPCCECAdapterCommunication::Write(
    const cec_command &data, bool &bRetry, uint8_t iLineTimeout, bool bIsReply)
{
  cec_adapter_message_state retVal(ADAPTER_MESSAGE_STATE_UNKNOWN);
  if (!IsRunning())
    return retVal;

  CCECAdapterMessage *output = new CCECAdapterMessage(data, iLineTimeout);
  output->bFireAndForget = bIsReply;

  /* mark as waiting for an ack from the destination */
  MarkAsWaiting(data.destination);

  /* send the message */
  if (bIsReply)
  {
    retVal =
        m_adapterMessageQueue->Write(output) ?
            ADAPTER_MESSAGE_STATE_WAITING_TO_BE_SENT :
            ADAPTER_MESSAGE_STATE_ERROR;
  }
  else
  {
    bRetry = (!m_adapterMessageQueue->Write(output) || output->NeedsRetry())
        && output->transmit_timeout > 0;
    if (bRetry)
      Sleep (CEC_DEFAULT_TRANSMIT_RETRY_WAIT);
    retVal = output->state;

    delete output;
  }
  return retVal;
}

void *CSolidPCCECAdapterCommunication::Process(void)
{
  CCECAdapterMessage msg;
  LIB_CEC->AddLog(CEC_LOG_DEBUG, "communication thread started");

  while (!IsStopped())
  {
    /* read from the serial port */
    if (!ReadFromDevice(50, 5))
    {
      libcec_parameter param;
      param.paramData = NULL;
      param.paramType = CEC_PARAMETER_TYPE_UNKOWN;
      LIB_CEC->Alert(CEC_ALERT_CONNECTION_LOST, param);

      break;
    }

    /* TODO sleep 5 ms so other threads can get a lock */
    if (!IsStopped())
      Sleep(5);
  }

  m_adapterMessageQueue->Clear();
  LIB_CEC->AddLog(CEC_LOG_DEBUG, "communication thread ended");
  return NULL;
}
void CSolidPCCECAdapterCommunication::MarkAsWaiting(
    const cec_logical_address dest)
{
  /* mark as waiting for an ack from the destination */
  if (dest < CECDEVICE_BROADCAST)
  {
    CLockObject waitingLock(m_waitingMutex);
    m_bWaitingForAck[dest] = true;
  }
}

void CSolidPCCECAdapterCommunication::ClearInputBytes(
    uint32_t iTimeout /* = CEC_CLEAR_INPUT_DEFAULT_WAIT */)
{
  CTimeout timeout(iTimeout);
  uint8_t buff[1024];
  ssize_t iBytesRead(0);
  bool bGotMsgEnd(true);

  while (timeout.TimeLeft() > 0
      && ((iBytesRead = m_port->Read(buff, 1024, 5)) > 0 || !bGotMsgEnd))
  {
    bGotMsgEnd = false;
    /* if something was received, wait for MSGEND */
    for (ssize_t iPtr = 0; iPtr < iBytesRead; iPtr++)
      bGotMsgEnd = buff[iPtr] == MSGEND;
  }
}

bool CSolidPCCECAdapterCommunication::SetLineTimeout(uint8_t iTimeout)
{
  bool bReturn(true);
  bool bChanged(false);

  /* only send the command if the timeout changed */
  {
    CLockObject lock(m_mutex);
    bChanged = (m_iLineTimeout != iTimeout);
    m_iLineTimeout = iTimeout;
  }

  if (bChanged)
    bReturn = m_commands->SetLineTimeout(iTimeout);

  return bReturn;
}

bool CSolidPCCECAdapterCommunication::WriteToDevice(CCECAdapterMessage *message)
{
  CLockObject adapterLock(m_mutex);
  if (!IsOpen())
  {
    LIB_CEC->AddLog(CEC_LOG_DEBUG,
        "error writing command '%s' to serial port '%s': the connection is closed",
        CCECAdapterMessage::ToString(message->Message()),
        m_port->GetName().c_str());
    message->state = ADAPTER_MESSAGE_STATE_ERROR;
    return false;
  }

  /* write the message */
  if (m_port->Write(message->packet.data, message->Size())
      != (ssize_t) message->Size())
  {
    LIB_CEC->AddLog(CEC_LOG_DEBUG,
        "error writing command '%s' to serial port '%s': %s",
        CCECAdapterMessage::ToString(message->Message()),
        m_port->GetName().c_str(), m_port->GetError().c_str());
    message->state = ADAPTER_MESSAGE_STATE_ERROR;
    // let the higher level close the port
    return false;
  }

#ifdef CEC_DEBUGGING
  LIB_CEC->AddLog(CEC_LOG_DEBUG, "command '%s' sent", message->IsTransmission() ? "CEC transmission" : CCECAdapterMessage::ToString(message->Message()));
#endif
  message->state = ADAPTER_MESSAGE_STATE_SENT;
  return true;
}

bool CSolidPCCECAdapterCommunication::ReadFromDevice(uint32_t iTimeout,
    size_t iSize /* = 256 */)
{
  ssize_t iBytesRead(0);
  uint8_t buff[256];
  if (iSize > 256)
    iSize = 256;

  /* read from the serial port */
  {
    CLockObject lock(m_mutex);
    if (!IsOpen())
      return false;

    do
    {
      /* retry Read() if it was interrupted */
      iBytesRead = m_port->Read(buff, sizeof(uint8_t) * iSize, iTimeout);
    } while (m_port->GetErrorNumber() == EINTR);

    if (m_port->GetErrorNumber())
    {
      LIB_CEC->AddLog(CEC_LOG_ERROR, "error reading from serial port: %s",
          m_port->GetError().c_str());
      // let the higher level close the port
      return false;
    }
  }

  if (iBytesRead < 0 || iBytesRead > 256)
    return false;
  else if (iBytesRead > 0)
  {
    /* add the data to the current frame */
    m_adapterMessageQueue->AddData(buff, iBytesRead);
  }

  return true;
}

CCECAdapterMessage *CSolidPCCECAdapterCommunication::SendCommand(
    cec_adapter_messagecode msgCode, CCECAdapterMessage &params,
    bool bIsRetry /* = false */)
{
  if (!IsOpen() || !m_adapterMessageQueue)
    return NULL;

  /* create the adapter message for this command */
  CCECAdapterMessage *output = new CCECAdapterMessage;
  output->PushBack('!');
//  output->PushEscaped((uint8_t) msgCode);
  output->Append(params);
  output->PushBack(MSGEND);

  /* write the command */
  if (!m_adapterMessageQueue->Write(output))
  {
    // this will trigger an alert in the reader thread
    if (output->state == ADAPTER_MESSAGE_STATE_ERROR)
      m_port->Close();
    return output;
  }
  else
  {
    if (!bIsRetry && output->Reply() == MSGCODE_COMMAND_REJECTED
        && msgCode != MSGCODE_SET_CONTROLLED
        && msgCode
            != MSGCODE_GET_BUILDDATE /* same messagecode value had a different meaning in older fw builds */)
    {
      /* if the controller reported that the command was rejected, and we didn't send the command
       to set controlled mode, then the controller probably switched to auto mode. set controlled
       mode and retry */
      LIB_CEC->AddLog(CEC_LOG_DEBUG, "setting controlled mode and retrying");
      delete output;
      if (SetControlledMode(true))
        return SendCommand(msgCode, params, true);
    }
  }

  return output;
}

bool CSolidPCCECAdapterCommunication::CheckAdapter(
    uint32_t iTimeoutMs /* = CEC_DEFAULT_CONNECT_TIMEOUT */)
{
  bool bReturn(false);
  CTimeout timeout(iTimeoutMs > 0 ? iTimeoutMs : CEC_DEFAULT_TRANSMIT_WAIT);

  /* try to ping the adapter */
  bool bPinged(false);
  unsigned iPingTry(0);
  while (timeout.TimeLeft() > 0 && (bPinged = PingAdapter()) == false)
  {
    LIB_CEC->AddLog(CEC_LOG_ERROR,
        "the adapter did not respond correctly to a ping (try %d)", ++iPingTry);
    CEvent::Sleep(500);
  }

  /* try to read the firmware version */
  if (bPinged && timeout.TimeLeft() > 0
      && m_commands->RequestFirmwareVersion() >= 2)
  {
    /* try to set controlled mode for v2+ firmwares */
    unsigned iControlledTry(0);
    bool bControlled(false);
    while (timeout.TimeLeft() > 0
        && (bControlled = SetControlledMode(true)) == false)
    {
      LIB_CEC->AddLog(CEC_LOG_ERROR,
          "the adapter did not respond correctly to setting controlled mode (try %d)",
          ++iControlledTry);
      CEvent::Sleep(500);
    }
    bReturn = bControlled;
  }
  else
    bReturn = true;

  SetInitialised(bReturn);
  return bReturn;
}

bool CSolidPCCECAdapterCommunication::IsOpen(void)
{
  /* thread is not being stopped, the port is open and the thread is running */
  return !IsStopped() && m_port->IsOpen() && IsRunning();
}

std::string CSolidPCCECAdapterCommunication::GetError(void) const
{
  return m_port->GetError();
}

void CSolidPCCECAdapterCommunication::SetInitialised(bool bSetTo /* = true */)
{
  CLockObject lock(m_mutex);
  m_bInitialised = bSetTo;
}

bool CSolidPCCECAdapterCommunication::IsInitialised(void)
{
  CLockObject lock(m_mutex);
  return m_bInitialised;
}

bool CSolidPCCECAdapterCommunication::SetLogicalAddresses(
    const cec_logical_addresses &addresses)
{
  {
    CLockObject lock(m_mutex);
    if (m_logicalAddresses == addresses)
      return true;
  }

  if (IsOpen() && m_commands->SetAckMask(addresses.AckMask()))
  {
    CLockObject lock(m_mutex);
    m_logicalAddresses = addresses;
    return true;
  }

  LIB_CEC->AddLog(CEC_LOG_DEBUG,
      "couldn't change the ackmask: the connection is closed");
  return false;
}

cec_logical_addresses CSolidPCCECAdapterCommunication::GetLogicalAddresses(void)
{
  cec_logical_addresses addresses;
  CLockObject lock(m_mutex);
  addresses = m_logicalAddresses;
  return addresses;
}

uint16_t CSolidPCCECAdapterCommunication::GetFirmwareVersion(void)
{
  return m_commands ? m_commands->GetFirmwareVersion() : CEC_FW_VERSION_UNKNOWN;
}

uint16_t CSolidPCCECAdapterCommunication::GetAdapterVendorId(void) const
{
  return CEC_VID;
}

uint16_t CSolidPCCECAdapterCommunication::GetAdapterProductId(void) const
{
  return CEC_PID;
}

void CSolidPCCECAdapterCommunication::SetActiveSource(bool bSetTo,
    bool bClientUnregistered)
{
  if (m_commands)
    m_commands->SetActiveSource(bSetTo, bClientUnregistered);
}

bool CSolidPCCECAdapterCommunication::IsRunningLatestFirmware(void)
{
  return GetFirmwareVersion() >= CEC_LATEST_ADAPTER_FW_VERSION;
}

bool CSolidPCCECAdapterCommunication::PersistConfiguration(
    const libcec_configuration &configuration)
{
  return IsOpen() ? m_commands->PersistConfiguration(configuration) : false;
}

bool CSolidPCCECAdapterCommunication::GetConfiguration(
    libcec_configuration &configuration)
{
  return IsOpen() ? m_commands->GetConfiguration(configuration) : false;
}

std::string CSolidPCCECAdapterCommunication::GetPortName(void)
{
  return m_port->GetName();
}

bool CSolidPCCECAdapterCommunication::SetControlledMode(bool controlled)
{
  return IsOpen() ? m_commands->SetControlledMode(controlled) : false;
}

uint16_t CSolidPCCECAdapterCommunication::GetPhysicalAddress(void)
{
  uint16_t iPA(0);

  // try to get the PA from ADL
#if defined(HAS_ADL_EDID_PARSER)
  {
    LIB_CEC->AddLog(CEC_LOG_DEBUG,
        "%s - trying to get the physical address via ADL", __FUNCTION__);
    CADLEdidParser adl;
    iPA = adl.GetPhysicalAddress();
    LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s - ADL returned physical address %04x",
        __FUNCTION__, iPA);
  }
#endif

  // try to get the PA from the nvidia driver
#if defined(HAS_NVIDIA_EDID_PARSER)
  if (iPA == 0)
  {
    LIB_CEC->AddLog(CEC_LOG_DEBUG,
        "%s - trying to get the physical address via nvidia driver",
        __FUNCTION__);
    CNVEdidParser nv;
    iPA = nv.GetPhysicalAddress();
    LIB_CEC->AddLog(CEC_LOG_DEBUG,
        "%s - nvidia driver returned physical address %04x", __FUNCTION__, iPA);
  }
#endif

// try to get the PA from the intel driver
#if defined(HAS_DRM_EDID_PARSER)
  if (iPA == 0)
  {
    LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s - trying to get the physical address via drm files", __FUNCTION__);
    CDRMEdidParser nv;
    iPA = nv.GetPhysicalAddress();
    LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s - drm files returned physical address %04x", __FUNCTION__, iPA);
  }
#endif

  // try to get the PA from the OS
  if (iPA == 0)
  {
    LIB_CEC->AddLog(CEC_LOG_DEBUG,
        "%s - trying to get the physical address from the OS", __FUNCTION__);
    iPA = CEDIDParser::GetPhysicalAddress();
    LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s - OS returned physical address %04x",
        __FUNCTION__, iPA);
  }

  return iPA;
}
}
;
