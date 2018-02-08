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


#ifndef EMANEMODEL_DELPLAYER_DLEPCLIENTIMPL_HEADER
#define EMANEMODEL_DELPLAYER_DLEPCLIENTIMPL_HEADER

#include "emane/platformserviceprovider.h"

#include "dlep/DlepClient.h"
#include "dlep/DlepService.h"

#include <vector>

namespace EMANE
{
  namespace R2RI
  {
    namespace DLEP
   {
     inline const char * ReturnStatusToString(const LLDLEP::DlepService::ReturnStatus & status)
      {
        switch(status)
         {
           case LLDLEP::DlepService::ReturnStatus::ok:
           return "OK";

           case LLDLEP::DlepService::ReturnStatus::invalid_data_item:
           return "INVALID_DATA_ITEM";

           case LLDLEP::DlepService::ReturnStatus::invalid_mac_address:
           return "INVALID_MAC_ADDRESS";

           case LLDLEP::DlepService::ReturnStatus::destination_exists:
           return "DESTINATION_EXISTS";
  
           case LLDLEP::DlepService::ReturnStatus::destination_does_not_exist:
           return "DESTINATION_DOES_NOT_EXIST";

           case LLDLEP::DlepService::ReturnStatus::peer_does_not_exist:
           return "PEER_DOES_NOT_EXIST";

           default:
           return "UNKNOWN_STATUS_CODE";
         }
      }


     struct ConfigParameterInfo
       {
         const std::string alias;
         const std::string type;
         std::string       value;
         const std::string description;

         // to int vector
         std::vector<unsigned int> to_iv() const
          {
            // taken from ExampleDlepClientImpl.cpp
            std::vector<unsigned int> vui;

            unsigned int ui{};

            std::stringstream ss(value);

            while (ss >> ui)
             {
               vui.push_back(ui);

               if(ss.peek() == ',')
                 ss.ignore();
             }

            if(! ss.eof()) 
             {
               throw std::invalid_argument(value);
             }

            return vui;
          }
       };

     using ConfigParameterMapType = std::map<std::string, ConfigParameterInfo>;

      struct DataRateMetricInfo {
        std::string  idMaxDataRateRx;
        std::string  idMaxDataRateTx;
        std::string  idCurrentDataRateRx;
        std::string  idCurrentDataRateTx;

        std::uint64_t  valMaxDataRateRx;
        std::uint64_t  valMaxDataRateTx;
        std::uint64_t  valCurrentDataRateRx;
        std::uint64_t  valCurrentDataRateTx;

        DataRateMetricInfo() :
         idMaxDataRateRx{LLDLEP::ProtocolStrings::Maximum_Data_Rate_Receive},
         idMaxDataRateTx{LLDLEP::ProtocolStrings::Maximum_Data_Rate_Transmit},
         idCurrentDataRateRx{LLDLEP::ProtocolStrings::Current_Data_Rate_Receive},
         idCurrentDataRateTx{LLDLEP::ProtocolStrings::Current_Data_Rate_Transmit},
         valMaxDataRateRx{0},
         valMaxDataRateTx{0},
         valCurrentDataRateRx{0},
         valCurrentDataRateTx{0}
        { }
      };


      struct DestinationMetricInfo {

         DataRateMetricInfo  dataRateInfo;

         std::string idLatency;
         std::string idResourcesRx;
         std::string idResourcesTx;
         std::string idResources;
         std::string idRLQRx;
         std::string idRLQTx;

         std::uint64_t valLatency;
         std::uint8_t  valResourcesRx;
         std::uint8_t  valResourcesTx;
         std::uint8_t  valResources;
         std::uint8_t  valRLQRx;
         std::uint8_t  valRLQTx;

        DestinationMetricInfo() :
         idLatency{LLDLEP::ProtocolStrings::Latency},
         idResourcesRx{LLDLEP::ProtocolStrings::Resources_Receive},
         idResourcesTx{LLDLEP::ProtocolStrings:: Resources_Transmit},
         idResources{LLDLEP::ProtocolStrings:: Resources},
         idRLQRx{LLDLEP::ProtocolStrings::Relative_Link_Quality_Receive},
         idRLQTx{LLDLEP::ProtocolStrings::Relative_Link_Quality_Transmit},
         valLatency{0},
         valResourcesRx{100},
         valResourcesTx{100},
         valResources{100},
         valRLQRx{0},
         valRLQTx{100}
       { }
      };


     class DlepClientImpl : public LLDLEP::DlepClient
      {
        public:
          DlepClientImpl(NEMId id, 
                         PlatformServiceProvider * pPlatformService, 
                         const ConfigParameterMapType & config);

          virtual ~DlepClientImpl();

          // from DLEP Service
          void get_config_parameter(const std::string &parameterName, LLDLEP::DlepClient::ConfigValue *value) override;
          void get_config_parameter(const std::string &parameterName, bool *value) override;
          void get_config_parameter(const std::string &parameterName, unsigned int *value) override;
          void get_config_parameter(const std::string &parameterName, std::string *value) override;
          void get_config_parameter(const std::string &parameterName, boost::asio::ip::address *value) override;
          void get_config_parameter(const std::string &parameterName, std::vector<unsigned int> *value) override;


          void peer_up(const LLDLEP::PeerInfo & peerInfo) override;

          void peer_down(const std::string &peerId);

          void peer_update(const std::string & peerId, const LLDLEP::DataItems & dataItems);

          std::string destination_up(const std::string & peerId,
                              const LLDLEP::DlepMac & macAddress,
                              const LLDLEP::DataItems & dataItems) override;

          void destination_update(const std::string & peerId,
                                  const LLDLEP::DlepMac & macAddress,
                                  const LLDLEP::DataItems & dataItems) override;


          void destination_down(const std::string & peerId,
                                const LLDLEP::DlepMac & macAddress) override;

          void credit_request(const std::string & peerId,
                              const LLDLEP::DlepMac & macAddress) override;

          void linkchar_request(const std::string & peerId,
                                const LLDLEP::DlepMac & macAddress,
                                const LLDLEP::DataItems & dataItems) override;

          void linkchar_reply(const std::string & peer_id,
                              const LLDLEP::DlepMac & macAddress,
                              const LLDLEP::DataItems & dataItems) override;

          // from modem to DLEP
          bool send_peer_update(const LLDLEP::DataItems & dataItems);

          bool send_destination_up(const LLDLEP::DlepMac & macAddress, const LLDLEP::DataItems & dataItems);

          bool send_destination_update(const LLDLEP::DlepMac & macAddress, const LLDLEP::DataItems & dataItems);

          bool send_destination_down(const LLDLEP::DlepMac & macAddress);

          // access to protocol config
          LLDLEP::ProtocolConfig * get_protocol_config();

        private:
          NEMId id_;

          PlatformServiceProvider * pPlatformService_;

          LLDLEP::DlepService * pDlepService_;

          LLDLEP::ProtocolConfig * pProtocolConfig_;

          std::map<std::string, LLDLEP::DlepClient::ConfigValue> dlepConfigItems_;

          ConfigParameterMapType internalConfigItems_;

          bool load_config_i();

          void print_config_i() const;

          template <typename T> 
          bool lookup_parameter_i(const std::string &parameterName, T * value) const 
           {
              const auto iter = dlepConfigItems_.find(parameterName);

              if(iter != dlepConfigItems_.end())
                {
                   *value = boost::get<T>(iter->second);

                   return true;
                }
              else
               {
                   throw BadParameterName(parameterName + "NOT FOUND");

                   return false;
               }
           }
      };
    }
  }
}

#endif // DLEP_CLIENT_IMPL_H
