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

#include "dlepmodemservice.h"

#include "emane/configureexception.h"
#include "emane/utils/parameterconvert.h"
#include "emane/controls/serializedcontrolmessage.h"
#include "emane/controls/r2rineighbormetriccontrolmessageformatter.h"
#include "emane/controls/r2riqueuemetriccontrolmessageformatter.h"
#include "emane/controls/r2riselfmetriccontrolmessageformatter.h"

namespace
{
  const char * __MODULE__ = "DLEP::ModemService";

  const char * EtherAddrFormat = "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx";

  const char * EtherOUIFormat  = "%02hhx:%02hhx:%02hhx";

  template <typename T> 
  T clampit(T min, T max, T val)
   {
      if(val < min)
        {
          return min;
        }
      else if(val > max)
        {
          return max;
        }
      else
        {
          return val;
        }
   }


  const EMANE::R2RI::DLEP::ConfigParameterMapType DefaultDlepConfiguration =
    {
      // id (no special chars)      alias (special chars ok)   type  value          description
      { "acktimeout",              {"ack-timeout",             "i",  "3",           "seconds to wait for ack signals" } },
      { "ackprobability",          {"ack-probability",         "i",  "100",         "ack probabilty percent 0-100" } },
      { "configfile",              {"config-file",             "f",  "",            "xml config file containing parameter settings" } },
      { "heartbeatinterval",       {"heartbeat-interval",      "i",  "5",           "time between sending heartbeat signals" } },
      { "heartbeatthreshold",      {"heartbeat-threshold",     "i",  "2",           "number of missed heartbeats to tolerate" } },
      { "localtype",               {"local-type",              "s",  "modem",       "which dlep role to play, modem or router?" } },
      { "loglevel",                {"log-level",               "i",  "1",           "1=most logging, 5=least" } },
      { "logfile",                 {"log-file",                "s",  "dlep.log",    "file to write log messages to" } },
      { "peertype",                {"peer-type",               "s",  "emane",       "peer type data item value" } },
      { "sendtries",               {"send-tries",              "i",  "3",           "number of times to send a signal before giving up" } },
      { "sessionport",             {"session-port",            "i",  "30003",       "tcp port number session connections" } },
      { "extensions",              {"extensions",              "iv", "",            "list of extension ids to support" } },

      { "protocolconfigfile",      {"protocol-config-file",    "s",  "protocol-config.xml", "xml file containing dlep protocol config." } },
      { "protocolconfigschema",    {"protocol-config-schema",  "s",  "protocol-config.xsd", "xml protocol config schema." } },

      { "discoveryiface",          {"discovery-iface",         "s",  "eth0",        "interface for the peerdiscovery protocol" } },
      { "discoveryinterval",       {"discovery-interval",      "i",  "5",           "time between sending peerdiscovery signals" } },
      { "discoverymcastaddress",   {"discovery-mcast-address", "a",  "225.0.0.45",  "address to send peerdiscovery signals to" } },
      { "discoveryport",           {"discovery-port",          "i",  "30002",       "port to send peerdiscovery signals to" } },
      { "discoveryenable",         {"discovery-enable",        "b",  "1",           "should the router run the peerdiscovery protocol?" } },

      { "destinationadvertenable",       {"destination-advert-enable",        "b",   "1",         "dest advert enable/disable" } },
      { "destinationadvertiface",        {"destination-advert-iface",         "s",   "emane0",    "dest advert interface" } },
      { "destinationadvertsendinterval", {"destination-advert-send-interval", "i",   "5",         "dest advert tx interval" } },
      { "destinationadvertmcastaddress", {"destination-advert-mcast-address", "a",   "225.6.7.8", "dest advert multicast address" } },
      { "destinationadvertport",         {"destination-advert-port",          "i",   "33445",     "dest advert port" } },
      { "destinationadvertholdinterval", {"destination-advert-hold-interval", "i",   "0",         "dest advert hold value, 0 = hold" } },
      { "destinationadvertexpirecount",  {"destination-advert-expire-count",  "i",   "0",         "dest advert expire count, 0 = hold" } },
      { "destinationadvertrfid",         {"destination-advert-rf-id",         "iv",  "",          "dest advert mac id (NEMId)" } },
   };

}


EMANE::R2RI::DLEP::ModemService::ModemService(NEMId id,
                                              PlatformServiceProvider * pPlatformService,
                                              RadioServiceProvider * pRadioService) :
  id_{id},
  pPlatformService_{pPlatformService},
  pRadioService_{pRadioService},
  pDlepClient_{},
  avgQueueDelayMicroseconds_{},
  fSINRMin_{0.0f},
  fSINRMax_{20.0f},
  destinationAdvertisementEnable_{false}
{
  memset(&etherOUI_, 0x0, sizeof(etherOUI_)); 

  memset(&etherBroadcastAddr_, 0xFF, sizeof(etherBroadcastAddr_));

  dlepConfiguration_ = DefaultDlepConfiguration;
}



EMANE::R2RI::DLEP::ModemService::~ModemService() 
{ }



const EMANE::R2RI::DLEP::ConfigParameterMapType & EMANE::R2RI::DLEP::ModemService::getConfigItems() const
{
  return dlepConfiguration_;
}



void EMANE::R2RI::DLEP::ModemService::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s", id_, __MODULE__, __func__);

  for(const auto & item : update)
    {
      if(item.first == "macaddress")
        {
          for(const auto & any : item.second)
            {
              const std::string str{any.asString()};

              const size_t pos{str.find(" ")};
 
              if(pos != std::string::npos)
                {
                  const std::uint16_t nem{EMANE::Utils::ParameterConvert(std::string{str.data(), pos}).toUINT16()};

                  EMANE::Utils::EtherAddr eth;

                  if(sscanf(str.data() + pos + 1, 
                            EtherAddrFormat, 
                            eth.bytes.buff + 0,
                            eth.bytes.buff + 1,
                            eth.bytes.buff + 2,
                            eth.bytes.buff + 3,
                            eth.bytes.buff + 4,
                            eth.bytes.buff + 5) != 6)
                    {
                      throw makeException<ConfigureException>(__MODULE__,
                                                             "Invalid configuration item %s, expected <NEMId> <%s>",
                                                              item.first.c_str(),
                                                              EtherAddrFormat);
                    }

                  if(nemEtherAddrMap_.insert(std::make_pair(nem, eth)).second)
                    {
                       LOGGER_STANDARD_LOGGING(pPlatformService_->logService(), 
                                               INFO_LEVEL,
                                               "SHIMI %03hu %s::%s %s = %s",
                                               id_, 
                                               __MODULE__, 
                                               __func__, 
                                               item.first.c_str(),
                                               str.c_str());

                    }
                  else
                    {
                       throw makeException<ConfigureException>(__MODULE__,
                                                              "Invalid configuration item %s, duplicate NEMId %hu",
                                                               item.first.c_str(), nem);
                    }
                }
              else
                {
                   throw makeException<ConfigureException>(__MODULE__,
                                                          "Invalid configuration item %s, expected <NEMId> <%s>",
                                                           item.first.c_str(),
                                                           EtherAddrFormat);
                }
            }
         }
       else if(item.first == "etheraddroui")
         {
            const std::string str{item.second[0].asString()};

            if(sscanf(str.data(), 
                      EtherOUIFormat,
                      etherOUI_.bytes.buff + 0,
                      etherOUI_.bytes.buff + 1,
                      etherOUI_.bytes.buff + 2) != 3)
              {
                 throw makeException<ConfigureException>(__MODULE__,
                                                        "Invalid configuration item %s, expected prefix format <%s>",
                                                         item.first.c_str(),
                                                         EtherOUIFormat);
              }
            else
              {
                 LOGGER_STANDARD_LOGGING(pPlatformService_->logService(), 
                                         INFO_LEVEL,
                                         "SHIMI %03hu %s::%s %s = %s",
                                         id_, 
                                         __MODULE__, 
                                         __func__, 
                                         item.first.c_str(),
                                         str.c_str());
              }
         }
       else if(item.first == "sinrmin")
         {
            fSINRMin_ = item.second[0].asFloat();

            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    INFO_LEVEL,
                                    "SHIMI %03d %s::%s %s = %f dBm",
                                     id_,
                                     __MODULE__,
                                     __func__,
                                     item.first.c_str(),
                                     fSINRMin_);
         }
       else if(item.first == "sinrmax")
         {
            fSINRMax_ = item.second[0].asFloat();

            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    INFO_LEVEL,
                                    "SHIMI %03d %s::%s %s = %f dBm",
                                    id_,
                                    __MODULE__,
                                    __func__,
                                    item.first.c_str(),
                                    fSINRMax_);
         }
       else
         {
           auto iter = dlepConfiguration_.find(item.first);

           if(iter != dlepConfiguration_.end())
             {
                iter->second.value = item.second[0].asString();

                if(item.first == "destinationadvertenable")
                  {
                     destinationAdvertisementEnable_ = iter->second.value != "0";
                  }

                LOGGER_STANDARD_LOGGING(pPlatformService_->logService(), 
                                        INFO_LEVEL,
                                        "SHIMI %03hu %s::%s %s = %s",
                                        id_, 
                                        __MODULE__, 
                                        __func__, 
                                        item.first.c_str(),
                                        iter->second.value.c_str());
             }
           else
             { 
               throw makeException<ConfigureException>(__MODULE__,
                                                       "Unexpected configuration item %s",
                                                       item.first.c_str());
             }
         }
    }
}


void EMANE::R2RI::DLEP::ModemService::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s", id_, __MODULE__, __func__);

  // sinr saniity check
  if(fSINRMin_ > fSINRMax_)
    {
      throw makeException<ConfigureException>(__MODULE__,
                                              "sinrmin > ", 
                                              std::to_string(fSINRMin_).c_str(),
                                              " sinrmax", 
                                              std::to_string(fSINRMax_).c_str());
    }
  else
    {
      // configuration complete, create client impl
      pDlepClient_.reset(new DlepClientImpl{id_, pPlatformService_, dlepConfiguration_});
    }
}



void EMANE::R2RI::DLEP::ModemService::handleControlMessages(const EMANE::ControlMessages & controlMessages)
{
  try {
    for(const auto & pMessage : controlMessages)
      {
        switch(pMessage->getId())
          {
            case EMANE::Controls::R2RINeighborMetricControlMessage::IDENTIFIER:
              {
                handleMetricMessage_i(reinterpret_cast<const EMANE::Controls::R2RINeighborMetricControlMessage *>(pMessage));
              }
            break;

            case EMANE::Controls::R2RIQueueMetricControlMessage::IDENTIFIER:
              {
                handleMetricMessage_i(reinterpret_cast<const EMANE::Controls::R2RIQueueMetricControlMessage *>(pMessage));
              }
            break;

            case EMANE::Controls::R2RISelfMetricControlMessage::IDENTIFIER:
              {
                handleMetricMessage_i(reinterpret_cast<const EMANE::Controls::R2RISelfMetricControlMessage *>(pMessage));
              }
            break;

            case EMANE::Controls::FlowControlControlMessage::IDENTIFIER:
              {
                handleFlowControlMessage_i();
              }
            break;

            case EMANE::Controls::SerializedControlMessage::IDENTIFIER:
              {
                const auto pSerializedControlMessage =
                  static_cast<const EMANE::Controls::SerializedControlMessage *>(pMessage); 
        
                switch(pSerializedControlMessage->getSerializedId())
                  {
                    case EMANE::Controls::R2RINeighborMetricControlMessage::IDENTIFIER:
                      {
                        auto p = EMANE::Controls::R2RINeighborMetricControlMessage::create(
                                              pSerializedControlMessage->getSerialization());

                        handleMetricMessage_i(p);

                        delete p;
                      }
                    break;

                    case EMANE::Controls::R2RIQueueMetricControlMessage::IDENTIFIER:
                      {
                        auto p = EMANE::Controls::R2RIQueueMetricControlMessage::create(
                                              pSerializedControlMessage->getSerialization());

                        handleMetricMessage_i(p);

                        delete p;
                      }
                    break;

                    case EMANE::Controls::R2RISelfMetricControlMessage::IDENTIFIER:
                      {
                        auto p = EMANE::Controls::R2RISelfMetricControlMessage::create(
                                              pSerializedControlMessage->getSerialization());

                        handleMetricMessage_i(p);

                        delete p;
                      }
                    break;

                    case EMANE::Controls::FlowControlControlMessage::IDENTIFIER:
                      {
                        handleFlowControlMessage_i();
                      }
                    break;
                  }
              }
            break;
         }
      }
   }
  catch(const LLDLEP::ProtocolConfig::BadDataItemName & ex)
   {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "SHIMI %03hu %s::%s caught bad data item name exception %s", 
                              id_, 
                              __MODULE__, 
                              __func__, 
                              ex.what());
    }

  catch(const LLDLEP::ProtocolConfig::BadDataItemId & ex)
   {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "SHIMI %03hu %s::%s caught bad data item id exception %s", 
                              id_, 
                              __MODULE__, 
                              __func__, 
                              ex.what());
    }


  catch(const std::exception & ex)
   {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "SHIMI %03hu %s::%s caught std exception %s", 
                              id_, 
                              __MODULE__, 
                              __func__, 
                              ex.what());
    }
}



// self metrics
void EMANE::R2RI::DLEP::ModemService::handleMetricMessage_i(const EMANE::Controls::R2RISelfMetricControlMessage * pMessage)
{
   LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                               DEBUG_LEVEL, 
                                               EMANE::Controls::R2RISelfMetricControlMessageFormatter(pMessage),
                                               "SHIMI %03hu %s::%s R2RISelfMetricControlMessage",
                                               id_, __MODULE__, __func__);
   
   // update self metrics
   selfMetrics_.valMaxDataRateRx     = pMessage->getMaxDataRatebps();

   selfMetrics_.valMaxDataRateTx     = pMessage->getMaxDataRatebps();

   selfMetrics_.valCurrentDataRateRx = pMessage->getMaxDataRatebps();

   selfMetrics_.valCurrentDataRateTx = pMessage->getMaxDataRatebps();

   // now send a peer update
   send_peer_update_i();
}



// neighbor metrics
void EMANE::R2RI::DLEP::ModemService::handleMetricMessage_i(const EMANE::Controls::R2RINeighborMetricControlMessage * pMessage)
{
   LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                    DEBUG_LEVEL, 
                                    EMANE::Controls::R2RINeighborMetricControlMessageFormatter(pMessage),
                                    "SHIMI %03hu %s::%s R2RINeighborMetricControlMessage",
                                    id_, __MODULE__, __func__);
 
   const EMANE::Controls::R2RINeighborMetrics metrics{pMessage->getNeighborMetrics()};

   // possible set of nbrs to delete
   auto delNeighbors = currentNeighbors_;

   for(const auto & metric : metrics)
     {
       bool isNewNbr = false;

       const auto nbr = metric.getId();
  
       auto rxDataRate = metric.getRxAvgDataRatebps();

       auto txDataRate = metric.getTxAvgDataRatebps();

       auto iter = currentNeighbors_.find(nbr);

       // new nbr
       if(iter == currentNeighbors_.end())
         {
            NeighborInfo nbrInfo;

            // set is a new nbr
            isNewNbr = true;

            // set ether mac addr
            nbrInfo.macAddress_ = getEthernetAddress_i(nbr);

            // set Rx data rate
            nbrInfo.lastRxDataRate_ = rxDataRate ? rxDataRate : selfMetrics_.valCurrentDataRateRx;

            // set Tx data rate
            nbrInfo.lastTxDataRate_ = rxDataRate ? rxDataRate : selfMetrics_.valCurrentDataRateRx;

            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    DEBUG_LEVEL,
                                   "SHIMI %03hu %s::%s new nbr %hu, ether addr %s, txdr %lu, rxdr %lu", 
                                    id_, 
                                    __MODULE__, 
                                    __func__, 
                                    nbr, 
                                    nbrInfo.macAddress_.to_string().c_str(),
                                    nbrInfo.lastRxDataRate_,
                                    nbrInfo.lastTxDataRate_);
 
            // add to the current set
            iter = currentNeighbors_.insert(std::make_pair(nbr, nbrInfo)).first;
         } 
       else
         {
            // still active, remove from the candidate delete set
            delNeighbors.erase(nbr);

            // update Rx data rate
            if(rxDataRate > 0)
              {
                iter->second.lastRxDataRate_ = rxDataRate;
              }

            // update Tx data rate
            if(txDataRate > 0)
              {
                iter->second.lastTxDataRate_ = txDataRate;
              }

            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    DEBUG_LEVEL,
                                   "SHIMI %03hu %s::%s update nbr %hu, txdr %lu, rxdr %lu", 
                                    id_, 
                                    __MODULE__, 
                                    __func__, 
                                    nbr, 
                                    iter->second.lastRxDataRate_,
                                    iter->second.lastTxDataRate_);
         }

        // update destination metrics
        iter->second.metrics_.dataRateInfo.valCurrentDataRateRx = iter->second.lastRxDataRate_;

        iter->second.metrics_.dataRateInfo.valCurrentDataRateTx = iter->second.lastTxDataRate_;

        iter->second.metrics_.dataRateInfo.valMaxDataRateRx     = selfMetrics_.valMaxDataRateRx;

        iter->second.metrics_.dataRateInfo.valMaxDataRateTx     = selfMetrics_.valMaxDataRateTx;

        iter->second.metrics_.valLatency = avgQueueDelayMicroseconds_;

        iter->second.metrics_.valRLQRx   = getRLQ_i(metric.getId(),
                                                    metric.getSINRAvgdBm(),
                                                    metric.getNumRxFrames(),
                                                    metric.getNumMissedFrames());

        // send nbr up/update
        send_destination_update_i(iter->second, isNewNbr);
     }


   // any nbrs still in the delete set
   for(const auto & nbr : delNeighbors)
     {
        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                DEBUG_LEVEL,
                               "SHIMI %03hu %s::%s no longer reported, delete nbr %hu", 
                                id_, __MODULE__, __func__, nbr.first);
 
        // send del nbr
        send_destination_down_i(nbr.second);

        // remove from the current set
        currentNeighbors_.erase(nbr.first);
     }
}


// queue metrics
void EMANE::R2RI::DLEP::ModemService::handleMetricMessage_i(const EMANE::Controls::R2RIQueueMetricControlMessage * pMessage)
{
   LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                    DEBUG_LEVEL, 
                                    EMANE::Controls::R2RIQueueMetricControlMessageFormatter(pMessage),
                                    "SHIMI %03hu %s::%s R2RIQueueMetricControlMessage",
                                    id_, __MODULE__, __func__);

  // avg queue delay sum
  std::uint64_t delaySum{};

  size_t count{};

  // since there may be multiple Q's, get the overall avg
  for(const auto & metric : pMessage->getQueueMetrics())
    {
       delaySum += metric.getAvgDelay().count(); // in useconds
       
       ++count;
    }

  if(count)
    {
      avgQueueDelayMicroseconds_ = delaySum / count;
    }
  else
    {
      avgQueueDelayMicroseconds_ = 0;
    }
}


void EMANE::R2RI::DLEP::ModemService::handleFlowControlMessage_i()
{
   LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           DEBUG_LEVEL, 
                           "SHIMI %03hu %s::%s XXX_TODO credits",
                           id_, __MODULE__, __func__);
}

 

LLDLEP::DlepMac EMANE::R2RI::DLEP::ModemService::getEthernetAddress_i(const EMANE::NEMId nbr) const
{
  LLDLEP::DlepMac mac;

  if(destinationAdvertisementEnable_)
    {
      mac.mac_addr.push_back((nbr >> 8) & 0xFF);
      mac.mac_addr.push_back(nbr & 0xFF);
    }
  else
    {
      const auto iter = nemEtherAddrMap_.find(nbr);

      EMANE::Utils::EtherAddr etherAddr;

      // check the specific nbr to ether addr mapping first
      if(iter != nemEtherAddrMap_.end())
       {
          etherAddr = iter->second;
       } 
      // otherwise use the OUI prefix (if any)
      else
       {
         etherAddr = etherOUI_;

         etherAddr.words.word3 = ntohs(nbr);
       }

      mac.mac_addr.assign(etherAddr.bytes.buff, etherAddr.bytes.buff + 6); 
   }

  return mac;
}



void EMANE::R2RI::DLEP::ModemService::send_peer_update_i()
{
  if(pDlepClient_)
    {
      LLDLEP::DataItems metrics{};

      load_datarate_metrics_i(metrics, selfMetrics_);

      const bool result = pDlepClient_->send_peer_update(metrics);

      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL,
                              "SHIMI %03hu %s::%s entries %zu, result %s", 
                              id_, __MODULE__, __func__, 
                              metrics.size(), 
                              result ? "success" : "failed");
    }
  else
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "SHIMI %03hu %s::%s DlepClient not ready", 
                              id_, __MODULE__, __func__);
    }
}


void EMANE::R2RI::DLEP::ModemService::send_destination_update_i(const EMANE::R2RI::DLEP::ModemService::NeighborInfo & nbrInfo, bool isNewNbr)
{
  if(pDlepClient_)
    {
      LLDLEP::DataItems metrics{};

      load_destination_metrics_i(metrics, nbrInfo.metrics_);

      bool result{};

      if(isNewNbr)
        {
          // dest up
          result = pDlepClient_->send_destination_up(nbrInfo.macAddress_, metrics);
        }
      else
        {
          // dest update
          result = pDlepClient_->send_destination_update(nbrInfo.macAddress_, metrics);
        }

      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL,
                              "SHIMI %03hu %s::%s %s mac %s, entries %zu, result %s", 
                              id_, __MODULE__, __func__, 
                              isNewNbr ? "new" : "existing",
                              nbrInfo.macAddress_.to_string().c_str(),
                              metrics.size(), 
                              result ? "success" : "failed");
    }
  else
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "SHIMI %03hu %s::%s DlepClient not ready", 
                              id_, __MODULE__, __func__);
    }
}



void EMANE::R2RI::DLEP::ModemService::send_destination_down_i(const EMANE::R2RI::DLEP::ModemService::NeighborInfo & nbrInfo)
{
  if(pDlepClient_)
    {
      const bool result = pDlepClient_->send_destination_down(nbrInfo.macAddress_);

      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL,
                              "SHIMI %03hu %s::%s mac %s, result %s", 
                              id_, __MODULE__, __func__, 
                              nbrInfo.macAddress_.to_string().c_str(),
                              result ? "success" : "failed");
    }
  else
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "SHIMI %03hu %s::%s DlepClient not ready", 
                              id_, __MODULE__, __func__);
    }
}



void EMANE::R2RI::DLEP::ModemService::load_datarate_metrics_i(LLDLEP::DataItems & dataItems, const EMANE::R2RI::DLEP::DataRateMetricInfo & values)
{
   // max data rate Rx
   dataItems.push_back(getDataItem_i(values.idMaxDataRateRx, values.valMaxDataRateRx));

   // max data rate Tx
   dataItems.push_back(getDataItem_i(values.idMaxDataRateTx, values.valMaxDataRateTx));

   // curr data rate Rx
   dataItems.push_back(getDataItem_i(values.idCurrentDataRateRx, values.valCurrentDataRateRx));

   // curr data rate Tx
   dataItems.push_back(getDataItem_i(values.idCurrentDataRateTx, values.valCurrentDataRateTx));

   for(auto & item : dataItems)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                       DEBUG_LEVEL,
                       "SHIMI %03hu %s::%s item metric [%s]", 
                       id_, __MODULE__, __func__, 
                       item.to_string().c_str());
    }
}



void EMANE::R2RI::DLEP::ModemService::load_destination_metrics_i(LLDLEP::DataItems & dataItems, const EMANE::R2RI::DLEP::DestinationMetricInfo & values)
{
   // load data rate values
   load_datarate_metrics_i(dataItems, values.dataRateInfo);

   // latency
   dataItems.push_back(getDataItem_i(values.idLatency, values.valLatency));

   // resources Rx
   dataItems.push_back(getDataItem_i(values.idResourcesRx, values.valResourcesRx));

   // resources Tx
   dataItems.push_back(getDataItem_i(values.idResourcesTx, values.valResourcesTx));

   // rlq Rx
   dataItems.push_back(getDataItem_i(values.idRLQRx, values.valRLQRx));

   // rlq Tx
   dataItems.push_back(getDataItem_i(values.idRLQTx, values.valRLQTx));

   for(auto & item : dataItems)
    {
       LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                               DEBUG_LEVEL,
                               "SHIMI %03hu %s::%s item metric [%s]", 
                               id_, __MODULE__, __func__, 
                               item.to_string().c_str());
    }
}


int EMANE::R2RI::DLEP::ModemService::getRLQ_i(const std::uint16_t nbr, 
                                              const float fSINRAvg, 
                                              const size_t numRxFrames, 
                                              const size_t numMissedFrames)
{
  const auto numRxAndMissedFrames = numRxFrames + numMissedFrames;

  float fNewSINR{};

  float fReceiveRatio{};

  int iRLQ{};

  // lets find some history on this nbr 
  // even if this is the first report of this nbr it should 
  // be stored with initial values of (RR = 0, and SINR = -256)
  const auto & iter = currentNeighbors_.find(nbr);

  if(iter != currentNeighbors_.end())
    {
      // we have no pkt info for this interval
      if(numRxAndMissedFrames == 0.0f)
       {
         // reduce sinr by 3db
         fNewSINR = iter->second.fSINRlast_ - 3.0f;

         fReceiveRatio = iter->second.fRRlast_;
       }
      else
       {
         fNewSINR = fSINRAvg;

         fReceiveRatio = (float) numRxFrames / (float) numRxAndMissedFrames;
       }

      // check sinr is above min configured value 
      if(fNewSINR > fSINRMin_)
        {
          const auto fDeltaConfigSINR = fSINRMax_ - fSINRMin_;

          // the min to avg sinr delta 
          const auto fDeltaSINR = fNewSINR - fSINRMin_;

          // calculate rlq
          const auto fValue = 100.0f * (fDeltaSINR / fDeltaConfigSINR) * fReceiveRatio;

          // clamp between 0 and 100
          iRLQ = clampit(0.0f, 100.0f, fValue);
        }

      // save the sinr
      iter->second.fSINRlast_ = fNewSINR;

      // save the rr
      iter->second.fRRlast_ = fReceiveRatio;
   }

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03hu %s::%s nbr %hu. sinr %f, receive ratio %f, RLQ %d", 
                          id_, __MODULE__, __func__, 
                          nbr, 
                          fNewSINR, 
                          fReceiveRatio, 
                          iRLQ); 

  return iRLQ;
}
