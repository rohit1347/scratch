/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 University of Arizona
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */


#include "ns3/config-store-module.h"
#include "ns3/full-module.h"
#include "ns3/wifi-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
// #include "ns3/tools-module.h"
#include "ns3/average.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"


#include "ns3/full-module.h"


#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Duplex");

void ThroughputSeconds( Ptr<FlowMonitor> monitor)
{
  double avgTp = 0;
  double tp = 0;
  Time now = Simulator::Now ();
//  int flowCount = 0;
  monitor->CheckForLostPackets ();
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      tp = i->second.rxBytes * 8.0 /(now - Seconds(1)).GetSeconds()/ 1024 / 1024;
      avgTp += tp;
//      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
//      NS_LOG_INFO( "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")");
      //      std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
      //      std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
//            NS_LOG_INFO( "  Tx Packets:   " << i->second.txPackets);
//            NS_LOG_INFO( "  Rx Packets:   " << i->second.rxPackets);
//            NS_LOG_INFO( "  Tp: " << tp  << " Mbps");
//            NS_LOG_INFO( "  Packets Lost: " << i->second.lostPackets);
    }
  NS_LOG_INFO ("throughput of "<<now.GetSeconds ()<<" is :" <<avgTp << " Mbps");
  Simulator::Schedule (Seconds (0.5), &ThroughputSeconds, monitor);
}


void
SetForwardQueue (std::map<Ptr<Node> , std::list<Ptr<Node> > > apClientMap,
    Ptr<PropagationLossModel> lossModel, double txPower)
{

	double threshold = 3;
	std::map<Ptr<Node> , std::list<Ptr<Node> > >::iterator it;
	for ( it = apClientMap.begin (); it != apClientMap.end (); it++)
	{
		Ptr<ForwardQueue> queue = CreateObject<ForwardQueue> ();
		std::list<Ptr<Node> > clients = (*it).second;
		std::list<Ptr<Node> >::iterator lit;
		std::list<Ptr<Node> >::iterator llit;
		for (lit = clients.begin (); lit != clients.end (); lit++)
		{
			Mac48Address macAddress = Mac48Address::ConvertFrom ((*lit)->GetDevice (0)->GetAddress ());
			ForwardMap  cmap (macAddress);
			for (llit = clients.begin (); llit != clients.end (); llit++)
			{
				if (llit == lit)
				{
					continue;
				}
				double r2r = CalculateSnr ((*llit), (*lit), lossModel, txPower);
				double a2r1 = CalculateSnr ((*it).first, (*lit), lossModel, txPower);
				double a2r2 = CalculateSnr ((*it).first, (*llit), lossModel, txPower);
				double sinr1 = a2r2 - r2r;
				double sinr2 = a2r1 - r2r;

				if (sinr1 > threshold && sinr2 > threshold) //std::pow (10.0, threshold))
				{
					Mac48Address add = Mac48Address::ConvertFrom ((*llit)->GetDevice (0)->GetAddress ());
					cmap.AddItem(ForwardItem (add, 1));
				}
			}
			if (cmap.Size () > 0)
			{
				queue->AddForwardMap (cmap);
			}
		}
	    Ptr<FullWifiNetDevice> device = Ptr<FullWifiNetDevice>(dynamic_cast<FullWifiNetDevice*> (
	          ns3::PeekPointer ((*it).first->GetDevice (0))));
	    Ptr<FullRegularWifiMac> mac = Ptr<FullRegularWifiMac>(dynamic_cast<FullRegularWifiMac*>(
	          ns3::PeekPointer (device->GetMac ())));
	    mac->SetForwardQueue (queue);
	}
}




void
ClockSeconds (double interval)
{
  NS_LOG_INFO ("T " << Simulator::Now ().GetSeconds ());
  Simulator::Schedule (Seconds (interval), &ClockSeconds, interval);
}

Ptr<DuplexExperiment> d = CreateObject<DuplexExperiment> ();

//void SendDataDone (std::string context, uint32_t nodeId, uint32_t iface, bool success, uint32_t bytes, DuplexMacHeader::PacketType type)
//{
//  if (success)
//    {
//      d->nodeLogList[nodeId]->sendDataSuccess++;
//      d->nodeLogList[nodeId]->bytesSent += bytes;
//      if(type == DuplexMacHeader::FULL)
//        {
//          d->nodeLogList[nodeId]->exposedBytes += bytes;
//        }
//      if(type == DuplexMacHeader::PAYLOAD)
//        {
//          d->nodeLogList[nodeId]->primaryBytes += bytes;
//        }
//      if(type ==  DuplexMacHeader::FULL_SECOND)
//        {
//          d->nodeLogList[nodeId]->secondaryBytes += bytes;
//        }
//
////      if(bytes < 1500)
////        {
////          std::cout <<"the bytes send is incorrect: " << bytes<<"\n";
////        }
//
//    }
//  else
//    d->nodeLogList[nodeId]->sendDataFail++;
//}

void SendDataDone (std::string context, uint32_t nodeId, uint32_t iface, bool success, uint32_t bytes)
{
  if (success)
    {
      d->nodeLogList[nodeId]->sendDataSuccess++;
      d->nodeLogList[nodeId]->bytesSent += bytes;

//      if(bytes < 1500)
//        {
//          std::cout <<"the bytes send is incorrect: " << bytes<<"\n";
//        }

    }
  else
    d->nodeLogList[nodeId]->sendDataFail++;
}

void FullAckTimeout (std::string context, const FullWifiMacHeader & hdr)
{
  Mac48Address src = hdr.GetAddr2 ();
  uint8_t add [6];
  src.CopyTo (add);
  d->nodeLogList[add[5]-1]->ackTimeout++;
}

void HalfAckTimeout (std::string context, const WifiMacHeader & hdr)
{
  Mac48Address src = hdr.GetAddr2 ();
  uint8_t add [6];
  src.CopyTo (add);
  d->nodeLogList[add[5]-1]->ackTimeout++;
}

void FullSendPacket (std::string context, const FullWifiMacHeader & hdr)
{
  Mac48Address src = hdr.GetAddr2 ();
  uint8_t add [6];
  src.CopyTo (add);
  d->nodeLogList[add[5]-1]->sendPacket++;
}

void HalfSendPacket(std::string context, const WifiMacHeader & hdr)
{
  Mac48Address src = hdr.GetAddr2 ();
  uint8_t add [6];
  src.CopyTo (add);
  d->nodeLogList[add[5]-1]->sendPacket++;
}

void Enqueue (std::string context, uint32_t nodeId, uint32_t iface) { d->nodeLogList[nodeId]->enqueue++; }
void SendSecondaryPacket (std::string context, uint32_t nodeId, uint32_t iface) { d->nodeLogList[nodeId]->sendSecondaryPacket++; }
void SendExposedPacket (std::string context, uint32_t nodeId, uint32_t iface) { d->nodeLogList[nodeId]->sendExposedPacket++; }
void SendSignature (std::string context, uint32_t nodeId, uint32_t iface) {d->nodeLogList[nodeId]->sendSignature++; }
void SendPayload (std::string context, uint32_t nodeId, uint32_t iface) {d->nodeLogList[nodeId]->sendPayload++; }
void SendAck (std::string context, uint32_t nodeId, uint32_t iface) {d->nodeLogList[nodeId]->sendAck++; }
//void SendPacket (std::string context, uint32_t nodeId, uint32_t iface) {d->nodeLogList[nodeId]->sendPacket++; }
void SendFill (std::string context, uint32_t nodeId, uint32_t iface) {d->nodeLogList[nodeId]->sendFill++; }
void SendData (std::string context, uint32_t nodeId, uint32_t iface) {d->nodeLogList[nodeId]->sendData++; }

void FailedToReceiveAck (std::string context, uint32_t nodeId, uint32_t iface)
{
  d->nodeLogList[nodeId]->failedAck++;
}

void ReceiveData(std::string context, uint32_t nodeId, uint32_t iface, Time delay)
{
  d->nodeLogList[nodeId]->delay += ((double)delay.GetMicroSeconds ())/1e6;
}

void EnableLog ()
{
	  LogComponentEnable("FullMacLow", LogLevel(LOG_FUNCTION | LOG_ALL | LOG_PREFIX_TIME | LOG_PREFIX_NODE) );
	  LogComponentEnable("FullDcaTxop", LogLevel(LOG_FUNCTION | LOG_ALL | LOG_PREFIX_TIME | LOG_PREFIX_NODE) );
	  LogComponentEnable("FullYansWifiPhy", LogLevel(LOG_FUNCTION | LOG_ALL | LOG_PREFIX_TIME | LOG_PREFIX_NODE) );

}

int main (int argc, char *argv[])
{

//  LogComponentEnable("FullMacLow", LogLevel(LOG_FUNCTION | LOG_ALL | LOG_PREFIX_TIME | LOG_PREFIX_NODE) );
//  LogComponentEnable("FullDcaTxop", LogLevel(LOG_FUNCTION | LOG_ALL | LOG_PREFIX_TIME | LOG_PREFIX_NODE) );
//  LogComponentEnable("YansWifiPhy", LogLevel(LOG_FUNCTION | LOG_ALL | LOG_PREFIX_TIME | LOG_PREFIX_NODE) );
  LogComponentEnable("Duplex", LogLevel(LOG_FUNCTION | LOG_ALL ) );

  //  LogComponentEnable("Ipv4AddressGenerator", LogLevel(LOG_FUNCTION | LOG_ALL | LOG_PREFIX_TIME | LOG_PREFIX_NODE) );
//  Simulator::Schedule (Seconds (0), EnableLog);
//    SeedManager::SetRun (7);
// defaults:

  d->numAps = 20;
  d->streamsPerNode = 2;
  d->numNodesPerAp = 3;
  d->dim = 800;

  d->fullDuplex = true;
  d->captureEffect = true;
  d->returnPacket = false;
  d->secondaryPacket = true;
  d->busytone = false;
  bool verbose = false;
  d->phyMode = "OfdmRate6Mbps";

  double th = 0.1;

  d->protocol = "udp";

  d->stopTime = Seconds(2);

  int nRun = 1;
  float downRatio = 0.5;
  CommandLine cmd = CreateCommandLine(d);
  cmd.AddValue("nRun", "the index of this run", nRun);
  cmd.AddValue("downRatio", "the ratio of downlink flows", downRatio );
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);

  cmd.AddValue ("phyMode", "Wifi Phy mode", d->phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", d->packetSize);
  cmd.AddValue ("startTime", "start time", d->startTime);
  cmd.AddValue ("stopTime", "stop time", d->stopTime);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("fullDuplex", "enable full duplex (true) or half duplex (false)", d->fullDuplex);
  cmd.AddValue ("captureEffect", "enable capture effect in the PHY (true) or not (false)", d->captureEffect);
  cmd.AddValue ("returnPacket", "enable returnPacket (true) or not (false)", d->returnPacket);
  cmd.AddValue ("secondaryPacket", "enable forwarding packet (true) or not (false)", d->secondaryPacket);
  cmd.AddValue ("busyTone", "enable sending busytone (true) or not (false)", d->busytone);
  cmd.AddValue ("uplinkRate", "uplink data rate", d->uplinkRate);
  cmd.AddValue ("downlinkRate", "downlink data rate", d->downlinkRate);

  cmd.Parse (argc, argv);

  d->numNodes = d->numAps*(1+d->numNodesPerAp);


  NodeContainer nodes;
  nodes.Create (d->numNodes);
  NS_LOG_INFO ("Created " << (int)nodes.GetN() << " nodes");
  d->nodeLogList.Create (d->numNodes);
  d->numStreams =(uint16_t) d->numAps*d->numNodesPerAp * d->streamsPerNode;

  std::stringstream ss;
  ss<<"runs/pos_" << (d->fullDuplex ? "duplex": "half") <<(d->secondaryPacket ? "sec":"nosec") << "_nodes_"<<(int)d->numNodesPerAp<<"_aps_"<<(int)d->numAps <<"_run_" << nRun;
  d->positionFileName = ss.str ();
  std::stringstream ss_log;
  ss_log<<"runs/log_"<<(d->fullDuplex ? "duplex": "half") <<(d->secondaryPacket ? "sec":"nosec") <<"_nodes_"<<(int)d->numNodesPerAp<<"_aps_"<<(int)d->numAps <<"_run_" << nRun;
  d->logFileName = ss_log.str ();
  std::stringstream ss_flow;
  ss_flow<<"runs/flow_"<<(d->fullDuplex ? "duplex": "half") <<(d->secondaryPacket ? "sec":"nosec") <<"_nodes_"<< (int)d->numNodesPerAp<<"_aps_"<<(int)d->numAps <<"_run_" << nRun;
  d->flowFileName = ss_flow.str ();

  // MOBILITY
  UniformVariable rand_i;
  NodeContainer apNodes;
  NodeContainer stationNodes;
  std::vector<uint16_t> apIndices;


  //  place each node into one of the two containers
  for (uint16_t i = 0; i < d->numNodes; ++i)
    {
      if (i < d->numAps)
        apNodes.Add (nodes.Get (i));
      else
        stationNodes.Add (nodes.Get (i));
    }

  NS_LOG_DEBUG ("num stations: "<< stationNodes.GetN () << " num aps: "<< apNodes.GetN ());


  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  uint16_t rowsize = ceil (sqrt (d->numAps));
  double x,y,gap;
  gap = d->dim / rowsize;
  std::vector<std::pair<double,double> > apPositions;
  // RandomVariable apOffset = NormalVariable (0,1);
  // RandomVariable stationOffset = NormalVariable (0,1);
  NormalRandomVariable apOffset;
  NormalRandomVariable stationOffset;

  if (! d->loadPositions)
    {
      NS_LOG_INFO ("Creating and writing node positions to file");
      std::ofstream fpos;
      fpos.open(d->positionFileName.c_str(), std::ofstream::trunc);
      for (uint16_t index = 0; index < d->numAps; ++index)
        {
          x = (index % rowsize +.5+ d->apSpread*apOffset.GetValue (0,1)) * gap;
          y = (floor (index / rowsize) +.5+ d->apSpread*apOffset.GetValue (0,1)) * gap;
          positionAlloc->Add (Vector (x, y, 0.0));
          apPositions.push_back (std::make_pair (x,y));
          fpos << x << std::endl << y << std::endl;
//          NS_LOG_INFO (x << ", " << y);
        }
      uint16_t stationCount = 0;
      uint16_t curAp = 0;
      for (uint16_t index = d->numAps; index < d->numNodes; ++index)
        {
          std::pair<double,double> apPos = apPositions[curAp];
          x = (apPos.first + d->stationSpread*stationOffset.GetValue ());
          y = (apPos.second + d->stationSpread*stationOffset.GetValue ());
          positionAlloc->Add (Vector (x, y, 0.0));
          fpos << x << std::endl << y << std::endl;
//          NS_LOG_INFO (x << ", " << y);

          stationCount++;
          if (stationCount == d->numNodesPerAp)
            {
              stationCount = 0;
              curAp++;
            }
        }
      fpos.close ();
    }
  else
    {
      NS_LOG_INFO ("Loading node positions from file");
      std::ifstream fpos;
      fpos.open(d->positionFileName.c_str(), std::ifstream::in);
      for (uint16_t index = 0; index < d->numNodes; ++index)
        {
          fpos >> x;
          fpos >> y;
          positionAlloc->Add (Vector (x, y, 0.0));
          //          NS_LOG_INFO (x << ", " << y);
        }
      fpos.close ();
    }
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);


  // DEVICES
  Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel> ();
  NetDeviceContainer devices;
  std::string mode;
  if (d->fullDuplex)
    {
      mode = "full";
      NS_LOG_INFO ("full duplex mode");
      if ( d->captureEffect )
        {
          NS_LOG_INFO ("captureEffect : enabled.");
        }
      else
        {
          NS_LOG_INFO ("captureEffect : disabled.");
        }
      if ( d->returnPacket )
        {
          NS_LOG_INFO ("returnPacket : enabled.");
        }
      else
        {
          NS_LOG_INFO ("returnPacket : disabled.");
        }
      if ( d->secondaryPacket )
        {
          NS_LOG_INFO ("secondaryPacket : enabled.");
        }
      else
        {
          NS_LOG_INFO ("secondaryPacket : disabled.");
        }

      // disable fragmentation for frames below 2200 bytes
      Config::SetDefault ("ns3::FullWifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
      // turn off RTS/CTS for frames below 2200 bytes
      Config::SetDefault ("ns3::FullWifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
      // Fix non-unicast data rate to be the same as that of unicast
      Config::SetDefault ("ns3::FullWifiRemoteStationManager::NonUnicastMode",
                          StringValue (d->phyMode));

      // The below set of helpers will help us to put together the wifi NICs we want
      FullWifiHelper wifi;
      if (verbose)
        {
          wifi.EnableLogComponents ();  // Turn on all Wifi logging
        }
      wifi.SetStandard (FULL_WIFI_PHY_STANDARD_80211a);

      FullYansWifiPhyHelper wifiPhy =  FullYansWifiPhyHelper::Default ();
      // This is one parameter that matters when using FixedRssLossModel
      // set it to zero; otherwise, gain will be added
      wifiPhy.Set ("RxGain", DoubleValue (0) );
      wifiPhy.Set ("EnableCaptureEffect", BooleanValue (d->captureEffect));
      // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
      wifiPhy.SetPcapDataLinkType (FullYansWifiPhyHelper::DLT_IEEE802_11_RADIO);
      // Tracing
      wifiPhy.EnablePcap ("full-wifi-simple-adhoc", devices);

      FullYansWifiChannelHelper wifiChannel;
      wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
      // The below FixedRssLossModel will cause the rss to be fixed regardless
      // of the distance between the two stations, and the transmit power
//      wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
      wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");
      wifiPhy.SetChannel (wifiChannel.Create ());

      // Add a non-QoS upper mac, and disable rate control
      FullNqosWifiMacHelper wifiMac = FullNqosWifiMacHelper::Default ();
      wifi.SetRemoteStationManager ("ns3::FullConstantRateWifiManager",
                                    "DataMode",StringValue (d->phyMode),
                                    "ControlMode",StringValue (d->phyMode));
      // Set it to adhoc mode
      wifiMac.SetType ("ns3::FullAdhocWifiMac");
      wifiMac.Set ("EnableReturnPacket", BooleanValue (d->returnPacket));
      wifiMac.Set ("EnableForward", BooleanValue (d->secondaryPacket));
      wifiMac.Set ("EnableBusyTone", BooleanValue (d->busytone));

      devices = wifi.Install (wifiPhy, wifiMac, nodes);
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::FullWifiNetDevice/Mac/AckTimeout", MakeCallback (&FullAckTimeout));
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::FullWifiNetDevice/Mac/SendPacket", MakeCallback (&FullSendPacket));
    }
  else
    {
      mode = "half";

      NS_LOG_INFO ("half duplex mode");

      // disable fragmentation for frames below 2200 bytes
      Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
      // turn off RTS/CTS for frames below 2200 bytes
      Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
      // Fix non-unicast data rate to be the same as that of unicast
      Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                          StringValue (d->phyMode));

      // The below set of helpers will help us to put together the wifi NICs we want
      WifiHelper wifi;
      if (verbose)
        {
          wifi.EnableLogComponents ();  // Turn on all Wifi logging
        }
      wifi.SetStandard (WIFI_PHY_STANDARD_80211a);

      YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
      // This is one parameter that matters when using FixedRssLossModel
      // set it to zero; otherwise, gain will be added
      wifiPhy.Set ("RxGain", DoubleValue (0) );
      // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
      wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
      // Tracing
      wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

      YansWifiChannelHelper wifiChannel;
      wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
      // The below FixedRssLossModel will cause the rss to be fixed regardless
      // of the distance between the two stations, and the transmit power
//      wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
      wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");
      wifiPhy.SetChannel (wifiChannel.Create ());

      // Add a non-QoS upper mac, and disable rate control
      NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
      wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode",StringValue (d->phyMode),
                                    "ControlMode",StringValue (d->phyMode));
      // Set it to adhoc mode
      wifiMac.SetType ("ns3::AdhocWifiMac");
      devices = wifi.Install (wifiPhy, wifiMac, nodes);
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckTimeout", MakeCallback (&HalfAckTimeout));
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/SendPacket", MakeCallback (&HalfSendPacket));
    }

  InternetStackHelper internet;
  internet.Install (nodes);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.0.0");
  //  Ipv4InterfaceContainer iface =
  ipv4.Assign (devices);



  // APPLICATIONS
  std::vector<std::pair<Ptr<Node>, double> > nodeList;
  std::pair<Ptr<Node>, Ptr<Node> > nodePair;
//  std::vector<std::pair<Ptr<Node>, Ptr<Node> > > pairList;

  std::vector<std::pair<Ptr<Node>, Ptr<Node> > >  upList;
  std::vector<std::pair<Ptr<Node>, Ptr<Node> > >  downList;

  Ptr<Node> curNode, ap;

  std::map<Ptr<Node> , std::list<Ptr<Node> > > apClientMap;
  for (uint16_t i = 0; i < apNodes.GetN (); i++)
  {
	  std::list<Ptr<Node> > clientList;
	  apClientMap.insert (std::make_pair (apNodes.Get (i), clientList));
  }

  //  std::map< std::pair<Ipv4Address, Ipv4Address>, double> flowDist;
  // for each node in full list, sort apNodes by distance to node
  for (uint16_t i = 0; i < stationNodes.GetN (); ++i)
    {
      curNode = stationNodes.Get (i);
      nodeList = GetDistanceList (apNodes, curNode);

      uint16_t k = 0;

      if(nodeList.size () > 0)
        {
        ap = nodeList[k].first;
//        NS_LOG_DEBUG ("chose ap: " << ap->GetId ());
        }
      else
        {
          continue;
        }
//        pairList.push_back(std::make_pair(curNode,ap));
//        pairList.push_back(std::make_pair(ap,curNode));
        upList.push_back(std::make_pair(curNode,ap));
        downList.push_back(std::make_pair(ap,curNode));
        apClientMap[ap].push_back (curNode);
    }

  if (d->fullDuplex)
  {
	  SetForwardQueue (apClientMap, lossModel, 1);
  }

  std::vector<std::pair<Ptr<Node>, Ptr<Node> > > flowList;
  uint16_t curNumStreams = 0;
  Ptr<Node> n1,n2;

  uint16_t upStreams = d->numStreams * (1 - downRatio);
  uint16_t downStreams = d->numStreams * downRatio;

  DataRate upRate = DataRate (d->uplinkRate);
  double cbr1 = (d->packetSize * 8)/ static_cast<double>(upRate.GetBitRate ());
//  std::cout << "interval1 :" << cbr1 <<"\n";

  DataRate downRate = DataRate (d->downlinkRate);
  double cbr2 = (d->packetSize * 8)/ static_cast<double>(downRate.GetBitRate ());
//  std::cout << "interval2 :" << cbr2 <<"\n";

  /*while (curNumStreams < upStreams and upList.size () > 0)
    {
      uint16_t index = rand_i.GetInteger (0, upList.size () - 1);
      if(d->protocol.compare("udp")==0)
        {
          CreateStream(upList[index].first, upList[index].second, d->startTime, d->stopTime, cbr1, d->packetSize);
        }
      else if(d->protocol.compare("tcp")==0)
        {
          CreateTcpStream(upList[index].first, upList[index].second, d->startTime, d->stopTime, cbr1, d->packetSize);
        }
      else
        {
          std::cout <<"the protocol name is incorrect. It should be \"udp\" or \"tcp\"\n";
          return 0;
        }
      curNumStreams++;
      flowList.push_back(upList[index]);
      upList.erase (upList.begin () + index);
    }
*/


  NS_LOG_INFO ("Created " << curNumStreams << " uplink streams.");
  curNumStreams = 0;
  while (curNumStreams < downStreams and downList.size () > 0)
    {
      uint16_t index = rand_i.GetInteger (0, downList.size () - 1);
      if(d->protocol.compare("udp")==0)
        {
          CreateStream(downList[index].first, downList[index].second, d->startTime, d->stopTime, cbr2, d->packetSize);
        }
      else if(d->protocol.compare("tcp")==0)
        {
          CreateTcpStream(downList[index].first, downList[index].second, d->startTime, d->stopTime, cbr2, d->packetSize);
        }
      else
        {
          std::cout <<"the protocol name is incorrect. It should be \"udp\" or \"tcp\"\n";
          return 0;
        }
      curNumStreams++;
      flowList.push_back(downList[index]);
      downList.erase (downList.begin () + index);
    }
  NS_LOG_INFO ("Created " << curNumStreams << " downlink streams.");



  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  Simulator::Schedule (Seconds (2), ThroughputSeconds, monitor);

  // SIMULATION
  Simulator::Stop (d->stopTime);
//  Simulator::Schedule (d->startTime, &ClockSeconds, .5);

  NS_LOG_INFO ("Run Simulation.");

  Simulator::Run ();

  NS_LOG_INFO ("Writing results to " << d->logFileName << " and legend to " << d->legendFileName);

  std::ofstream flog;
  flog.open(d->logFileName.c_str());
  flog << d->nodeLogList.Report (d->stopTime-d->startTime, false);
  flog.close ();

  flog.open(d->legendFileName.c_str());
  flog << ReportLegend ();
  flog.close ();

//  std::cout << d->nodeLogList.ReportThroughput (d->stopTime-d->startTime) <<"\n";

//  NS_LOG_INFO ("Results:");
//  NS_LOG_INFO (d->nodeLogList.Report (d->stopTime-d->startTime, true));

  std::ofstream fout;
  fout.open(d->flowFileName.c_str());

  double avgTp = 0;
  double tp = 0;
  double sum = 0;
//  int flowCount = 0;
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      tp = i->second.rxBytes * 8.0 / (d->stopTime-d->startTime).GetSeconds() / 1e6;
//        tp = i->second.rxBytes;
      avgTp += tp;
      sum += tp*tp;
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      NS_LOG_INFO( "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")");
//            std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
//            std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
            NS_LOG_INFO( "  Tx Packets:   " << i->second.txPackets);
            NS_LOG_INFO( "  Rx Packets:   " << i->second.rxPackets);
            NS_LOG_INFO( "  Tp: " << tp  << " Mbps");
            NS_LOG_INFO( "  Packets Lost: " << i->second.lostPackets);
            fout << tp << ",";
    }
  //std::cout<<"flow monitors: "
  std::cout<<avgTp<<"\n";
  fout << std::endl;
  fout.close ();

  //std::cout << "fairness: " << (avgTp*avgTp)/(sum*d->numStreams) <<"\n";



  // 11. Cleanup
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");

  return 0;
}

