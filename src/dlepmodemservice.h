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


#ifndef EMANEMODEL_DLEPLAYER_MODEMSERVICE_HEADER_
#define EMANEMODEL_DLEPLAYER_MODEMSERVICE_HEADER_

#include "emane/shimlayerimpl.h"
#include "emane/utils/netutils.h"

#include "emane/controls/r2rineighbormetriccontrolmessage.h"
#include "emane/controls/r2riqueuemetriccontrolmessage.h"
#include "emane/controls/r2riselfmetriccontrolmessage.h"
#include "emane/controls/flowcontrolcontrolmessage.h"

#include "dlepclientimpl.h"

#include <map>

namespace EMANE
{
  namespace R2RI
  {
    namespace DLEP
    {
      /**
       * @class ModemService
       *
       * @brief Implements EMANE DLEP ModemService
       */

      class ModemService
      {
      public:
        ModemService(NEMId id,
                     PlatformServiceProvider *pPlatformService,
                     RadioServiceProvider * pRadioServiceProvider);

        ~ModemService();

        void handleControlMessages(const ControlMessages & controlMessages);

        bool filterDataMessages(DownstreamPacket & pkt);

        void configure(const ConfigurationUpdate & update);

        void start();

        const ConfigParameterMapType & getConfigItems() const;
        
      private:
        struct NeighborInfo {
          std::uint64_t           lastTxDataRate_;
          std::uint64_t           lastRxDataRate_;

          float                   fSINRlast_;
          float                   fRRlast_;

          DestinationMetricInfo   metrics_;

          LLDLEP::DlepMac macAddress_;

          NeighborInfo() :
           lastTxDataRate_{},
           lastRxDataRate_{},
           fSINRlast_{-256.0f},
           fRRlast_{0.0f}
            { 
              memset(&macAddress_, 0x0, sizeof(macAddress_));
            }
         };


        void handleMetricMessage_i(const Controls::R2RINeighborMetricControlMessage * pMessage);

        void handleMetricMessage_i(const Controls::R2RIQueueMetricControlMessage * pMessage);

        void handleMetricMessage_i(const Controls::R2RISelfMetricControlMessage * pMessage);

        void handleFlowControlMessage_i();

        void send_peer_update_i();

        void send_destination_update_i(const NeighborInfo & nbrInfo, bool isNewNbr);

        void send_destination_down_i(const NeighborInfo & nbrInfo);

        void load_datarate_metrics_i(LLDLEP::DataItems & dataItems, const DataRateMetricInfo & values);

        void load_destination_metrics_i(LLDLEP::DataItems & dataItems, const DestinationMetricInfo & values);

        int getRLQ_i(const std::uint16_t nbr, const float fSINRAvg, const size_t numRxFrames, const size_t numMissedFrames);

        template <typename T>
        LLDLEP::DataItem getDataItem_i(const std::string & id, const T & val)
         {
           return LLDLEP::DataItem{id, LLDLEP::DataItemValue{val}, pDlepClient_->get_protocol_config()};
         }


        LLDLEP::DlepMac getEthernetAddress_i(const NEMId id) const;

        // storage for NEM to ether mac addr mapping
        using NEMEtherAddrMap = std::map<NEMId, Utils::EtherAddr>;

        // storage for nbr info
        using NeighborInfoMap = std::map<NEMId, NeighborInfo>;
    
        NEMId id_;

        PlatformServiceProvider * pPlatformService_;
 
        RadioServiceProvider * pRadioService_;

        NeighborInfoMap currentNeighbors_; 

        NEMEtherAddrMap nemEtherAddrMap_;

        Utils::EtherAddr etherOUI_;

        Utils::EtherAddr etherBroadcastAddr_;

        std::unique_ptr<DlepClientImpl> pDlepClient_;

        ConfigParameterMapType dlepConfiguration_;

        DataRateMetricInfo selfMetrics_;

        std::uint64_t avgQueueDelayMicroseconds_;

        float fSINRMin_;

        float fSINRMax_;

        bool destinationAdvertisementEnable_;

        std::string sDiscoverymcastaddress_;
      };
    }
  }
}

#endif
