/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of Adjacent Link, LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "dlepclientimpl.h"
#include "dlep/DlepInit.h"

#include "boost/lexical_cast.hpp"
#include "emane/logserviceprovider.h"

namespace {
  const char * __MODULE__ = "DLEPClientImpl";
}


EMANE::R2RI::DLEP::DlepClientImpl::DlepClientImpl(NEMId id, 
                                                  PlatformServiceProvider * pPlatformService,
                                                  const ConfigParameterMapType & config) :
  id_{id},
  pPlatformService_{pPlatformService},
  pDlepService_{},
  internalConfigItems_{config}
{
  load_config_i();

  print_config_i();

  // config is loaded, ready to call init, let the call backs begin
  pDlepService_ = LLDLEP::DlepInit(*this);

  sleep(1); // XXX avoid race condition on the dlep serivce 
            // creation in DlepInit seen in early stages of integration
            // need to check if this is still an issue JG March 23, 2016

  assert(pDlepService_);
}



EMANE::R2RI::DLEP::DlepClientImpl::~DlepClientImpl()
{
  if(pDlepService_)
    {
      // implies peer going down
      pDlepService_->terminate();
    }
}


LLDLEP::ProtocolConfig * 
EMANE::R2RI::DLEP::DlepClientImpl::get_protocol_config()
{
  return pDlepService_->get_protocol_config();
}


void 
EMANE::R2RI::DLEP::DlepClientImpl::get_config_parameter(const std::string & parameterName,
                                                        LLDLEP::DlepClient::ConfigValue *value)
{
   const auto iter = dlepConfigItems_.find(parameterName);

   if(iter != dlepConfigItems_.end())
     {
       *value = iter->second;

       LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                               DEBUG_LEVEL,
                               "SHIMI %03hu %s::%s found parameter name %s", 
                               id_, __MODULE__, __func__, 
                               parameterName.c_str());
     }
    else
     {
       LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           ERROR_LEVEL,
                           "SHIMI %03hu %s::%s parameter name %s not found", 
                           id_, __MODULE__, __func__, 
                           parameterName.c_str());

       throw BadParameterName(parameterName);
     }
}



void
EMANE::R2RI::DLEP::DlepClientImpl::get_config_parameter(const std::string & parameterName,
                                                        bool *value)
{
  const auto result = lookup_parameter_i(parameterName, value);

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s parameter name %s, result %s, bool value [%s]", 
                          id_, __MODULE__, __func__, 
                          parameterName.c_str(), 
                          result ? "found" : "not-found", 
                          *value ? "true" : " false");
}



void
EMANE::R2RI::DLEP::DlepClientImpl::get_config_parameter(const std::string & parameterName,
                                                        unsigned int *value)
{
  const auto result = lookup_parameter_i(parameterName, value);

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s parameter name %s, result %s, uint value [%u]", 
                          id_, __MODULE__, __func__, 
                          parameterName.c_str(), 
                          result ? "found" : "not-found", 
                          *value);
}



void
EMANE::R2RI::DLEP::DlepClientImpl::get_config_parameter(const std::string & parameterName,
                                                        std::string *value)
{
  const auto result = lookup_parameter_i(parameterName, value);

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s parameter name %s, result %s, string value [%s]", 
                          id_, __MODULE__, __func__, 
                          parameterName.c_str(), 
                          result ? "found" : "not-found", 
                          value->c_str());
}



void 
EMANE::R2RI::DLEP::DlepClientImpl::get_config_parameter(const std::string & parameterName,
                                                        boost::asio::ip::address *value)
{
  const auto result = lookup_parameter_i(parameterName, value);

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s parameter name %s, result %s, ipaddress value %s", 
                          id_, __MODULE__, __func__, 
                          parameterName.c_str(), 
                          result ? "found" : "not-found", 
                          value->to_string().c_str());
}



void 
EMANE::R2RI::DLEP::DlepClientImpl::get_config_parameter(const std::string & parameterName,
                                                        std::vector<unsigned int> *value)
{
  const auto result = lookup_parameter_i(parameterName, value);

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s parameter name %s, result %s, vector %zu", 
                          id_, __MODULE__, __func__, 
                          parameterName.c_str(), 
                          result ? "found" : "not-found", 
                          value->size());
}


void
EMANE::R2RI::DLEP::DlepClientImpl::peer_up(const LLDLEP::PeerInfo & peerInfo)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s, type %s, extensions %zu, experiment name %zu, dataItems %zu", 
                          id_, __MODULE__, __func__, 
                          peerInfo.peer_id.c_str(), 
                          peerInfo.peer_type.c_str(), 
                          peerInfo.extensions.size(), 
                          peerInfo.experiment_names.size(),
                          peerInfo.data_items.size());

    for(auto & item : peerInfo.data_items)
      {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s item metric type %hhu, value [%s]", 
                          id_, __MODULE__, __func__, 
                          item.id, 
                          item.to_string().c_str());
      }
 
    // XXX model specific operations here
}



void EMANE::R2RI::DLEP::DlepClientImpl::peer_down(const std::string & peerId)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s", 
                          id_, __MODULE__, __func__, 
                          peerId.c_str());

    // XXX model specific operations here
}


void EMANE::R2RI::DLEP::DlepClientImpl::peer_update(const std::string & peerId, 
                                                    const LLDLEP::DataItems & dataItems)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s, %zd dataItems", 
                          id_, __MODULE__, __func__, 
                          peerId.c_str(),
                          dataItems.size());

    for(auto & item : dataItems)
      {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s item metric type %hhu, value [%s]", 
                          id_, __MODULE__, __func__, 
                          item.id, 
                          item.to_string().c_str());
      }

    // XXX model specific operations here
}

std::string
EMANE::R2RI::DLEP::DlepClientImpl::destination_up(const std::string & peerId, 
                                                  const LLDLEP::DlepMac & macAddress,
                                                  const LLDLEP::DataItems & dataItems)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s, mac %s, dataItems %zu", 
                          id_, __MODULE__, __func__, 
                          peerId.c_str(), 
                          macAddress.to_string().c_str(), 
                          dataItems.size());

    for(auto & item : dataItems)
      {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s item metric type %hhu, value [%s]", 
                          id_, __MODULE__, __func__, 
                          item.id, 
                          item.to_string().c_str());
      }

    // XXX model specific operations here
   
    // return success for now
    return LLDLEP::ProtocolStrings::Success;
}


void
EMANE::R2RI::DLEP::DlepClientImpl::destination_update(const std::string & peerId, 
                                                      const LLDLEP::DlepMac & macAddress,
                                                      const LLDLEP::DataItems & dataItems)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s, mac %s, dataItems %zu", 
                          id_, __MODULE__, __func__, 
                          peerId.c_str(), 
                          macAddress.to_string().c_str(), 
                          dataItems.size());

    for(auto & item : dataItems)
      {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s item metric type %hhu, value [%s]", 
                          id_, __MODULE__, __func__, 
                          item.id, 
                          item.to_string().c_str());
      }

    // XXX model specific operations here
}



void
EMANE::R2RI::DLEP::DlepClientImpl::destination_down(const std::string & peerId,
                                                    const LLDLEP::DlepMac & macAddress)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s, mac %s", 
                          id_, __MODULE__, __func__, 
                          peerId.c_str(), 
                          macAddress.to_string().c_str());

    // XXX model specific operations here
}



void
EMANE::R2RI::DLEP::DlepClientImpl::credit_request(const std::string & peerId,
                                                  const LLDLEP::DlepMac & macAddress)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s, mac %s", 
                          id_, __MODULE__, __func__, 
                          peerId.c_str(), 
                          macAddress.to_string().c_str());

    // XXX model specific operations here
}



void EMANE::R2RI::DLEP::DlepClientImpl::linkchar_request(const std::string & peerId,
                                                         const LLDLEP::DlepMac & macAddress,
                                                         const LLDLEP::DataItems & dataItems)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s, mac %s, items %zu", 
                          id_, __MODULE__, __func__, 
                          peerId.c_str(), 
                          macAddress.to_string().c_str(), 
                          dataItems.size());

    for(auto & item : dataItems)
      {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s item metric type %hhu, value [%s]", 
                          id_, __MODULE__, __func__, 
                          item.id, 
                          item.to_string().c_str());
      }

    // XXX model specific operations here
}


void EMANE::R2RI::DLEP::DlepClientImpl::linkchar_reply(const std::string & peerId,
                                                       const LLDLEP::DlepMac & macAddress,
                                                       const LLDLEP::DataItems & dataItems)
{
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s peer %s, mac %s, items %zu", 
                          id_, __MODULE__, __func__, 
                          peerId.c_str(), 
                          macAddress.to_string().c_str(), 
                          dataItems.size());

    for(auto & item : dataItems)
      {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s item metric type %hhu, value [%s]", 
                          id_, __MODULE__, __func__, 
                          item.id, 
                          item.to_string().c_str());
      }

    // XXX model specific operations here
}



bool EMANE::R2RI::DLEP::DlepClientImpl::send_peer_update(const LLDLEP::DataItems & dataItems)
{
   const auto status = pDlepService_->peer_update(dataItems);

   LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           DEBUG_LEVEL,
                           "SHIMI %03hu %s::%s status %s", 
                           id_, __MODULE__, __func__, 
                           ReturnStatusToString(status));

   return status == LLDLEP::DlepService::ReturnStatus::ok;
}


bool EMANE::R2RI::DLEP::DlepClientImpl::send_destination_up(const LLDLEP::DlepMac & macAddress,
                                                            const LLDLEP::DataItems & dataItems)
{
   const auto status = pDlepService_->destination_up(macAddress, dataItems);

   LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           DEBUG_LEVEL,
                           "SHIMI %03hu %s::%s status %s", 
                           id_, __MODULE__, __func__, 
                           ReturnStatusToString(status));

   return status == LLDLEP::DlepService::ReturnStatus::ok;
}


bool EMANE::R2RI::DLEP::DlepClientImpl::send_destination_update(const LLDLEP::DlepMac & macAddress,
                                                                const LLDLEP::DataItems & dataItems)
{
   const auto status = pDlepService_->destination_update(macAddress, dataItems);

   LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           DEBUG_LEVEL,
                           "SHIMI %03hu %s::%s status %s", 
                           id_, __MODULE__, __func__, 
                           ReturnStatusToString(status));

   return status == LLDLEP::DlepService::ReturnStatus::ok;
}


bool EMANE::R2RI::DLEP::DlepClientImpl::send_destination_down(const LLDLEP::DlepMac & macAddress)
{
   const auto status = pDlepService_->destination_down(macAddress);

   LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           DEBUG_LEVEL,
                           "SHIMI %03hu %s::%s status %s", 
                           id_, __MODULE__, __func__, 
                           ReturnStatusToString(status));

   return status == LLDLEP::DlepService::ReturnStatus::ok;
}


bool 
EMANE::R2RI::DLEP::DlepClientImpl::load_config_i()
{
  for(auto const & item : internalConfigItems_)
    {
      // new item
      if(dlepConfigItems_.count(item.second.alias) == 0)
        {
          // int
          if(item.second.type == "i")
            {
              dlepConfigItems_[item.second.alias] = boost::lexical_cast<unsigned int>(item.second.value);
            }
          // bool
          else if(item.second.type == "b")
            {
              dlepConfigItems_[item.second.alias] = boost::lexical_cast<bool>(item.second.value);
            }
          // string /filename
          else if(item.second.type == "s" || item.second.type == "f")
            {
              dlepConfigItems_[item.second.alias] = std::string(item.second.value);
            }
          // address 
          else if (item.second.type == "a")
            {
              dlepConfigItems_[item.second.alias] = boost::asio::ip::address::from_string(item.second.value);
            }
          // int vector
          else if (item.second.type == "iv")
            {
               dlepConfigItems_[item.second.alias] = item.second.to_iv();
            }
          else
            {
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          ERROR_LEVEL,
                          "SHIMI %03hu %s::%s parameter name %s, has invalid type %s", 
                          id_, __MODULE__, __func__, 
                          item.second.alias.c_str(), 
                          item.second.type.c_str());

              return false;
            }
        }
      else
        {
          // duplicate
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "SHIMI %03hu %s::%s duplicate parameter name %s", 
                                  id_, __MODULE__, __func__, 
                                  item.second.alias.c_str());
         
          return false;
        }
    }        

    return true;
}



void
EMANE::R2RI::DLEP::DlepClientImpl::print_config_i() const
{
  for(auto const & item : dlepConfigItems_)
    {
       LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  "SHIMI %03hu %s::%s parameter name %s", 
                                  id_, __MODULE__, __func__, 
                                  item.first.c_str());
    }
}
