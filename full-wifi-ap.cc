/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/full-module.h"
#include "ns3/athstats-helper.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/wifi-phy-state.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/trace-helper.h"
#include <fstream>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("FullWifiAp");

static bool g_verbose = true;

void
// DevTxTrace (std::string context, Ptr<const Packet> p, Ptr<OutputStreamWrapper> stream)
DevTxTrace (std::string context, Ptr<const Packet> p)
{
  if (g_verbose)
    {
      // NS_LOG_UNCOND (" TX p: " << *p );
      // *stream->GetStream () << " TX p: " << *p << std::endl;
      std::cout << " TX p: " << *p << std::endl;
    }
}
void
// DevRxTrace (std::string context, Ptr<const Packet> p, Ptr<OutputStreamWrapper> stream)
DevRxTrace (std::string context, Ptr<const Packet> p)
{
  if (g_verbose)
    {
      // NS_LOG_UNCOND (" RX p: " << *p );
      // *stream->GetStream () << " RX p: " << *p << std::endl;
      std::cout << " RX p: " << *p << std::endl;
    }
}
void
// PhyRxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble, Ptr<OutputStreamWrapper> stream)
PhyRxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
{
  if (g_verbose)
    {
      // NS_LOG_UNCOND ("PHYRXOK mode=" << mode << " snr=" << snr << " " << *packet);
      // *stream->GetStream () << "PHYRXOK mode=" << mode << " snr=" << snr << " " << *packet << std::endl;
      std::cout << "PHYRXOK mode=" << mode << " snr=" << snr << " " << *packet << std::endl;
    }
}
void
// PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr, Ptr<OutputStreamWrapper> stream)
PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{
  if (g_verbose)
    {
      // NS_LOG_UNCOND ("PHYRXERROR snr=" << snr << " " << *packet);
      // *stream->GetStream () << "PHYRXERROR snr=" << snr << " " << *packet << std::endl;
      std::cout << "PHYRXERROR snr=" << snr << " " << *packet << std::endl;
    }
}
void
// PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower, Ptr<OutputStreamWrapper> stream)
PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{
  if (g_verbose)
    {
      // NS_LOG_UNCOND ("PHYTX mode=" << mode << " " << *packet);
      // *stream->GetStream () << "PHYTX mode=" << mode << " " << *packet << std::endl;
      std::cout << "PHYTX mode=" << mode << " " << *packet << std::endl;
    }
}
#ifndef WIFI_STATE
#define WIFI_STATE

enum WifiPhyState
    {
      IDLE,
      CCA_BUSY,
      TX,
      RX,
      SWITCHING,
      SLEEP,
      OFF
    };

#endif  

void
// PhyStateTrace (std::string context, Time start, Time duration, enum WifiPhyState state, Ptr<OutputStreamWrapper> stream)
PhyStateTrace (std::string context, Time start, Time duration, enum WifiPhyState state)
{
  if (g_verbose)
    {
      // NS_LOG_UNCOND (" state=" << state << " start=" << start << " duration=" << duration);
      // *stream->GetStream () << " state=" << state << " start=" << start << " duration=" << duration << std::endl;
      std::cout << " state=" << state << " start=" << start << " duration=" << duration << std::endl;
    }
}

static void
SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

//static Vector
//GetPosition (Ptr<Node> node)
//{
//  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
//  return mobility->GetPosition ();
//}

static void 
AdvancePosition (Ptr<Node> node) 
{
  Vector pos = GetPosition (node);
  pos.x += 5.0;
  if (pos.x >= 210.0) 
    {
      return;
    }
  SetPosition (node, pos);

  if (g_verbose)
    {
      std::cout << "x="<<pos.x << std::endl;
    }
  Simulator::Schedule (Seconds (1.0), &AdvancePosition, node);
}

int main (int argc, char *argv[])
{

  LogComponentEnable("FullWifiAp", LogLevel(LOG_FUNCTION | LOG_ALL ) );
  // NS_LOG_COMPONENT_DEFINE ("FullWifiAp");

  CommandLine cmd;
  cmd.AddValue ("verbose", "Print trace information if true", g_verbose);

  cmd.Parse (argc, argv);

  Packet::EnablePrinting ();

  // enable rts cts all the time.
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));

  // WifiHelper wifi = WifiHelper::Default ();
  WifiHelper wifi = WifiHelper ();
  MobilityHelper mobility;
  NodeContainer stas;
  NodeContainer ap;
  NetDeviceContainer staDevs;
  PacketSocketHelper packetSocket;

  stas.Create (2);
  ap.Create (1);

  // give packet socket powers to nodes.
  packetSocket.Install (stas);
  packetSocket.Install (ap);

  // NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  WifiMacHelper wifiMac = WifiMacHelper();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  Ssid ssid = Ssid ("wifi-default");
  wifi.SetRemoteStationManager ("ns3::ArfWifiManager");
  // setup stas.
  wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
  staDevs = wifi.Install (wifiPhy, wifiMac, stas);
  // setup ap.
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));
  wifi.Install (wifiPhy, wifiMac, ap);

  // mobility.
  mobility.Install (stas);
  mobility.Install (ap);

  Simulator::Schedule (Seconds (1.0), &AdvancePosition, ap.Get (0));

  PacketSocketAddress socket;
  socket.SetSingleDevice (staDevs.Get (0)->GetIfIndex ());
  socket.SetPhysicalAddress (staDevs.Get (1)->GetAddress ());
  socket.SetProtocol (1);

  OnOffHelper onoff ("ns3::PacketSocketFactory", Address (socket));
  onoff.SetConstantRate (DataRate ("500kb/s"));

  ApplicationContainer apps = onoff.Install (stas.Get (0));
  apps.Start (Seconds (0.5));
  apps.Stop (Seconds (43.0));

  Simulator::Stop (Seconds (44.0));

  Config::Connect ("/NodeList/*/DeviceList/*/Mac/MacTx", MakeCallback (&DevTxTrace));
  Config::Connect ("/NodeList/*/DeviceList/*/Mac/MacRx", MakeCallback (&DevRxTrace));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk", MakeCallback (&PhyRxOkTrace));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxError", MakeCallback (&PhyRxErrorTrace));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&PhyTxTrace));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/State", MakeCallback (&PhyStateTrace));

  AthstatsHelper athstats;
  athstats.EnableAthstats ("athstats-sta", stas);
  athstats.EnableAthstats ("athstats-ap", ap);

  // PcapHelper pcapHelper;
  // Ptr<PcapFileWrapper> file = pcapHelper.CreateFile ("fd_ap.pcap", std::ios::out, PcapHelper::DLT_PPP);
  // staDevs.Get (1)->TraceConnectWithoutContext ("/NodeList/*/DeviceList/*/Mac/MacTx", MakeCallback (&DevTxTrace, file));
  // AsciiTraceHelper ascii;
  // Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("fd_ap.dat");
  // TraceConnectWithoutContext("/NodeList/*/DeviceList/*/Mac/MacTx", MakeBoundCallback (&DevTxTrace,stream));
  // TraceConnectWithoutContext("/NodeList/*/DeviceList/*/Mac/MacRx", MakeBoundCallback (&DevRxTrace,stream));
  // TraceConnectWithoutContext("/NodeList/*/DeviceList/*/Phy/State/RxOk", MakeBoundCallback (&PhyRxOkTrace,stream));
  // TraceConnectWithoutContext("/NodeList/*/DeviceList/*/Phy/State/RxError", MakeBoundCallback (&PhyRxErrorTrace,stream));
  // TraceConnectWithoutContext("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeBoundCallback (&PhyTxTrace,stream));
  // TraceConnectWithoutContext("/NodeList/*/DeviceList/*/Phy/State/State", MakeBoundCallback (&PhyStateTrace,stream));

  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
