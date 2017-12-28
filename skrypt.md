/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 AGH University of Science and Technology
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
 * Author: Lukasz Prasnal <prasnal@kt.agh.edu.pl>
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/mobility-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/aodv-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/global-route-manager.h"
#include "ns3/olsr-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/olsr-routing-protocol.h"


using namespace ns3; 

NS_LOG_COMPONENT_DEFINE ("wifi-qos-test");

class SimulationHelper 
{
public:
	SimulationHelper ();
	
	static OnOffHelper CreateOnOffHelper(InetSocketAddress socketAddress, DataRate dataRate, int packetSize, uint8_t tid, Time start, Time stop);
	static void PopulateArpCache ();
};

SimulationHelper::SimulationHelper () 
{
}

//prepare CBR traffic source
OnOffHelper
SimulationHelper::CreateOnOffHelper(InetSocketAddress socketAddress, DataRate dataRate, int packetSize, uint8_t tid, Time start, Time stop) 
{
  socketAddress.SetTos (tid << 5); //(see: https://www.tucny.com/Home/dscp-tos and http://www.revolutionwifi.net/revolutionwifi/2010/08/wireless-qos-part-3-user-priorities.html)

  OnOffHelper onOffHelper  ("ns3::UdpSocketFactory", socketAddress);
  onOffHelper.SetAttribute ("OnTime",     StringValue   ("ns3::ConstantRandomVariable[Constant=100000]"));
  onOffHelper.SetAttribute ("OffTime",    StringValue   ("ns3::ConstantRandomVariable[Constant=0]") );
  onOffHelper.SetAttribute ("DataRate",   DataRateValue (dataRate) );
  onOffHelper.SetAttribute ("PacketSize", UintegerValue (packetSize) );
  onOffHelper.SetAttribute ("Jitter",     DoubleValue (1.0)); //packets generation times modified by random value between -50% and +50% of constant time step between packets
  onOffHelper.SetAttribute ("MaxBytes",   UintegerValue (0));
  onOffHelper.SetAttribute ("StartTime",  TimeValue (start));
  onOffHelper.SetAttribute ("StopTime",   TimeValue (stop));

  return onOffHelper;
}

//fullfil the ARP cache prior to simulation run
void
SimulationHelper::PopulateArpCache () 
{
  Ptr<ArpCache> arp = CreateObject<ArpCache> ();
  arp->SetAliveTimeout (Seconds (3600 * 24 * 365) );
	
  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) 
    {	
      Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
      NS_ASSERT (ip != 0);
      ObjectVectorValue interfaces;
      ip->GetAttribute ("InterfaceList", interfaces);

      for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j++) 
        {		
          Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
          NS_ASSERT (ipIface != 0);
          Ptr<NetDevice> device = ipIface->GetDevice ();
          NS_ASSERT (device != 0);
          Mac48Address addr = Mac48Address::ConvertFrom (device->GetAddress () );
      
          for (uint32_t k = 0; k < ipIface->GetNAddresses (); k++) 
            {			
              Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();		
              if (ipAddr == Ipv4Address::GetLoopback ()) 
                continue;

              ArpCache::Entry *entry = arp->Add (ipAddr);
              Ipv4Header ipv4Hdr;
              ipv4Hdr.SetDestination (ipAddr);
              Ptr<Packet> p = Create<Packet> (100);  
              entry->MarkWaitReply (ArpCache::Ipv4PayloadHeaderPair (p, ipv4Hdr) );
              entry->MarkAlive (addr);
            }
        }
    }

    for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) 
      {
        Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT (ip != 0);
		ObjectVectorValue interfaces;
		ip->GetAttribute ("InterfaceList", interfaces);

        for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
          {
            Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
            ipIface->SetAttribute ("ArpCache", PointerValue (arp) );
          }
      }
}



/* ===== main function ===== */

int main (int argc, char *argv[])
{
  uint32_t nSTA = 5;
  uint32_t packetSize = 1470;
  float simTime = 10;
  Time appsStart = Seconds(0);
  Time appsStartBE = Seconds(5);
  float radius = 1.0;
  float calcStart = 0;
  bool oneDest = true;
  bool rtsCts = false;
  bool A_VO = true;
  bool VO = true;
  bool VI = true;
  bool A_VI = true;
  bool BE = true;
  bool BK = true;
  double Mbps = 54;
  uint32_t seed = 1;
  float IdleSlope = 0.25;
  //double MbpsBE = 1;
//  float SendSlope = 0.25;


/* ===== Command Line parameters ===== */

  CommandLine cmd;
  cmd.AddValue ("nSTA",       "Number of stations",                            nSTA);
  cmd.AddValue ("packetSize", "Packet size [B]",                               packetSize);
  cmd.AddValue ("simTime",    "simulation time [s]",                           simTime);
  cmd.AddValue ("calcStart",  "start of results analysis [s]",                 calcStart);
  cmd.AddValue ("radius",     "Radius of area [m] to randomly place stations", radius);
  cmd.AddValue ("oneDest",    "use one traffic destination?",                  oneDest);
  cmd.AddValue ("RTSCTS",     "use RTS/CTS?",                                  rtsCts);
  cmd.AddValue ("A_VO",       "run A_VO traffic?",                             A_VO);
  cmd.AddValue ("VO",         "run VO traffic?",                               VO);
  cmd.AddValue ("VI",         "run VI traffic?",                               VI);
  cmd.AddValue ("A_VI",       "run A_VI traffic?",                             A_VI);
  cmd.AddValue ("BE",         "run BE traffic?",                               BE);
  cmd.AddValue ("BK",         "run BK traffic?",                               BK);
  cmd.AddValue ("Mbps",       "traffic generated per queue [Mbps]",            Mbps);
  cmd.AddValue ("MbpsBE",     "traffic generated in BE queue [MbpsBE]",        MbpsBE);
  cmd.AddValue ("seed",       "Seed",                                          seed);
  cmd.AddValue ("IdleSlope",  "Percent of traffic for CBSA",                   IdleSlope);

  cmd.Parse (argc, argv);

  Time simulationTime = Seconds (simTime);
  ns3::RngSeedManager::SetSeed (seed);
 
  Packet::EnablePrinting ();

  NodeContainer sta;
  sta.Create (nSTA);
  


/* ======== Positioning / Mobility ======= */
  
  //UniformDiscPositionAllocator - uniform distiburion of nodes on disc area
 // Ptr<UniformDiscPositionAllocator> positionAlloc = CreateObject<UniformDiscPositionAllocator> ();
 // positionAlloc->SetX   (0.0); positionAlloc->SetY   (0.0); //set disc center
 // positionAlloc->SetRho (radius); //area radius

 // ListPositionAllocator used for uniform distiburion of nodes on the circle around central node
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  //positionAlloc->Add (Vector (0.0, 0.0, 0.0)); //1st node/AP located in the center
  for (uint32_t i = 1; i < nSTA; i++)
    positionAlloc->Add (Vector ((20.0*(i)), 0.0, 0.0));

  MobilityHelper mobility;
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  //constant speed movement configuration
  /*Ptr<ConstantVelocityMobilityModel> mob = sta[std][grp].Get (n)->GetObject<ConstantVelocityMobilityModel> ();
  mob->SetVelocity (Vector3D (movX, movY, movZ) );*/

  mobility.Install (sta);



/* ===== Propagation Model configuration ===== */

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();



/* ===== MAC and PHY configuration ===== */

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  QosWifiMacHelper mac = QosWifiMacHelper::Default (); //802.11a
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  //QosWifiMacHelper mac = HtWifiMacHelper::Default (); //802.11n
  //wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
  //QosWifiMacHelper mac = VhtWifiMacHelper::Default (); //802.11ac
  //wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);


  //PHY parameters 
  //for complete list of available parameters - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_wifi_phy.html#pri-static-attribs
  phy.Set ("RxNoiseFigure",                DoubleValue   (7.0) );
  phy.Set ("TxPowerStart",                 DoubleValue   (15.0) );
  phy.Set ("TxPowerEnd",                   DoubleValue   (15.0) );
  phy.Set ("CcaMode1Threshold",            DoubleValue   (-82.0) );
  phy.Set ("EnergyDetectionThreshold",     DoubleValue   (-88.0) );
  phy.Set ("Antennas",                     UintegerValue (1) ); //for 802.11n/ac - see http://mcsindex.com/
  phy.Set ("MaxSupportedTxSpatialStreams", UintegerValue (1) ); //for 802.11n/ac - see http://mcsindex.com/
  phy.Set ("MaxSupportedRxSpatialStreams", UintegerValue (1) ); //for 802.11n/ac - see http://mcsindex.com/
  phy.Set ("ShortGuardEnabled",            BooleanValue  (true) ); //for 802.11n/ac - see http://mcsindex.com/
  

  //WiFi Remote Station Manager parameters 

  //Constant Rate setting - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_constant_rate_wifi_manager.html#pri-attribs
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
							    "DataMode",    StringValue ("OfdmRate54Mbps"), //802.11a
							    "ControlMode", StringValue ("OfdmRate6Mbps"),  //802.11a
							    //"DataMode",    StringValue ("HtMcs7"), //802.11n - see http://mcsindex.com/
							    //"ControlMode", StringValue ("HtMcs0"), //802.11n - see http://mcsindex.com/
							    //"DataMode",    StringValue ("VhtMcs9"), //802.11ac - see http://mcsindex.com/
							    //"ControlMode", StringValue ("VhtMcs0"), //802.11ac - see http://mcsindex.com/
								"MaxSrc", UintegerValue (7), //changed - Short Retry Counter (SRC) limit!
								"MaxLrc", UintegerValue (4), //changed - Long Retry Counter (LRC) limit!
							    "RtsCtsThreshold",        UintegerValue (rtsCts ? 0 : 2500),
							    "FragmentationThreshold", UintegerValue (2500));

  //IDEAL rate manager for 802.11a - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_ideal_wifi_manager.html#pri-attribs
  /*wifi.SetRemoteStationManager ("ns3::IdealWifiManager", 
							      "RtsCtsThreshold",        UintegerValue (rtsCts ? 0 : 2500),
							      "FragmentationThreshold", UintegerValue (2500));*/

  //MINSTREL rate manager for 802.11a - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_minstrel_wifi_manager.html#pri-attribs
  /*wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager", 
							      "RtsCtsThreshold",        UintegerValue (rtsCts ? 0 : 2500),
							      "FragmentationThreshold", UintegerValue (2500));*/

  //MINSTREL rate manager for 802.11n/ac - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_minstrel_ht_wifi_manager.html#pri-attribs
  /*wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager", 
							      "RtsCtsThreshold",        UintegerValue (rtsCts ? 0 : 2500),
							      "FragmentationThreshold", UintegerValue (2500));*/

  //MAC parameters
  //for complete list of available parameters - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_adhoc_wifi_mac.html#pri-methods
  mac.SetType ("ns3::AdhocWifiMac",
               "Ssid", SsidValue (Ssid ("TEST")),
               "AltEDCASupported",   BooleanValue (true)); //NEW!

  //example - MAC config with aggregation control for 802.11ac
  /*mac.SetType ("ns3::AdhocWifiMac",
               "VO_MaxAmsduSize",    UintegerValue (11398) ),
               "VI_MaxAmsduSize",    UintegerValue (11398) ),
               "BE_MaxAmsduSize",    UintegerValue (11398) ),
               "BK_MaxAmsduSize",    UintegerValue (11398) ),
               "VO_MaxAmpduSize",    UintegerValue (1048575) ),
               "VI_MaxAmpduSize",    UintegerValue (1048575) ),
               "BE_MaxAmpduSize",    UintegerValue (1048575) ),
               "BK_MaxAmpduSize",    UintegerValue (1048575) ),
               "Ssid", SsidValue (Ssid ("TEST")),
               "AltEDCASupported",   BooleanValue (true)); //NEW! */


//phy.SetMaxPortTxRate (5400000, "ns3::WifiMacQueueController");
//phy.SetTxRateCalcMethod (0, "ns3::WifiMacQueueController");


//DataRate IS7 = DataRate (1000000 * Mbps * IdleSlope);
//DataRate IS6 = DataRate (1000000 * Mbps * IdleSlope);
uint32_t TxRate = 1000000 * 54;
DataRate MaxTxRate = DataRate (TxRate);
DataRate IS5 = DataRate (TxRate * IdleSlope);
//DataRate IS4 = DataRate (TxRate * IdleSlope);


//DataRate SS4 = DataRate (1000000 * Mbps * SendSlope);
//DataRate SS5 = DataRate (1000000 * Mbps * SendSlope);

//one queue should use either STRICT PRIORITY either CREDIT BASED SHAPED ALGORITHM
//STRICT PRIORITY
  mac.SetQueueControllerForTid (7, "ns3::StrictPriorityQueueController");
  mac.SetQueueControllerForTid (6, "ns3::StrictPriorityQueueController");
//  mac.SetQueueControllerForTid (5, "ns3::StrictPriorityQueueController");
//  mac.SetQueueControllerForTid (4, "ns3::StrictPriorityQueueController");


//CREDIT BASED SHAPING ALGORITHM 
//NOTE - idleSlope setting should consider a margin for encapsulation overhead (different for different TX modes - for 54 Mb/s and frames ~1500B it is ~12%)
//idleSlope setting were choosen for unsaturated station
//  mac.SetQueueControllerForTid (7, "ns3::CbsaQueueController",
//                                "IdleSlope", DataRateValue (DataRate (IS7) ) ); //for ~7.5 Mb/s (7,5 * 1,12 = 8,4)
//  mac.SetQueueControllerForTid (6, "ns3::CbsaQueueController",
//                               "IdleSlope", DataRateValue (DataRate (IS6) ) ); //for ~2.5 Mb/s (2,5 * 1,12 = 2,8)
  mac.SetQueueControllerForTid (5, "ns3::CbsaQueueController",
                                "MaxPortTxRate", DataRateValue (DataRate (MaxTxRate) ) );
  mac.SetQueueControllerForTid (4, "ns3::CbsaQueueController",
                                "MaxPortTxRate", DataRateValue (DataRate (MaxTxRate) ) );

  mac.SetQueueControllerForTid (5, "ns3::CbsaQueueController",
                                "TxRateCalcMethod", UintegerValue (0) );
  mac.SetQueueControllerForTid (4, "ns3::CbsaQueueController",
                               "TxRateCalcMethod", UintegerValue (0) );

  mac.SetQueueControllerForTid (5, "ns3::CbsaQueueController",
                                "IdleSlope", DataRateValue (DataRate (IS5) ) );
  mac.SetQueueControllerForTid (4, "ns3::CbsaQueueController",
                               "IdleSlope", DataRateValue (DataRate (IS5) ) );




  NetDeviceContainer staDevices = wifi.Install (phy, mac, sta);


//Configs with paths:
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (20) );
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Slot", TimeValue (MicroSeconds (9)));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Sifs", TimeValue (MicroSeconds (16)));
 
  //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/GetMsduLifetime", TimeValue (Seconds (0.01)));
 //for 802.11n/ac - see http://mcsindex.com/

//EDCA parameters:
//see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_dca_txop.html#friends
  
//MinCw:
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_EdcaTxopN/MinCw", UintegerValue (3) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_EdcaTxopN/MinCw", UintegerValue (7) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BE_EdcaTxopN/MinCw", UintegerValue (15) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BK_EdcaTxopN/MinCw", UintegerValue (15) );    

//MaxCw:
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_EdcaTxopN/MaxCw", UintegerValue (7) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_EdcaTxopN/MaxCw", UintegerValue (15) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BE_EdcaTxopN/MaxCw", UintegerValue (1023) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BK_EdcaTxopN/MaxCw", UintegerValue (1023) );

//AIFSN:

  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_EdcaTxopN/Aifsn", UintegerValue (2) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_EdcaTxopN/Aifsn", UintegerValue (2) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BE_EdcaTxopN/Aifsn", UintegerValue (3) );
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BK_EdcaTxopN/Aifsn", UintegerValue (7) );
 
//TXOP limit:

  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_EdcaTxopN/TxopLimit", TimeValue (MicroSeconds (1504) ) ); 
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_EdcaTxopN/TxopLimit", TimeValue (MicroSeconds (3008) ) ); 
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BE_EdcaTxopN/TxopLimit", TimeValue (MicroSeconds    (0) ) ); 
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BK_EdcaTxopN/TxopLimit", TimeValue (MicroSeconds    (0) ) );

//EDCA max delay (NEW):
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_EdcaTxopN/MaxDelay", TimeValue (MicroSeconds  (10240) ) ); //setting VO packet lifetime = 10*TU (TU=1024 us)
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_EdcaTxopN/MaxDelay", TimeValue (MicroSeconds (102400) ) ); //setting VI packet lifetime = 100*TU (TU=1024 us)
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BE_EdcaTxopN/MaxDelay", TimeValue (MicroSeconds (512000) ) ); //setting BE packet lifetime = 500*TU (TU=1024 us)
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BK_EdcaTxopN/MaxDelay", TimeValue (MicroSeconds (512000) ) ); //setting BK packet lifetime = 500*TU (TU=1024 us)

  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_EdcaTxopN/HiTidQueue/MaxPackets",  UintegerValue (1000)); //setting A_VO queue size 
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_EdcaTxopN/LowTidQueue/MaxPackets", UintegerValue (1000)); //setting VO queue size 
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_EdcaTxopN/HiTidQueue/MaxPackets",  UintegerValue (1000)); //setting VI queue size 
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_EdcaTxopN/LowTidQueue/MaxPackets", UintegerValue (1000)); //setting A_VI queue size 
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BE_EdcaTxopN/Queue/MaxPackets",       UintegerValue (1000)); //setting BE queue size 
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BK_EdcaTxopN/Queue/MaxPackets",       UintegerValue (1000)); //setting BK queue size 

  //int64_t streamIndex = 0;
  //wifi.AssignStreams (staDevices, streamIndex);



/* ===== Internet stack ===== */

//OlsrHelper olsr;

Ipv4StaticRoutingHelper staticRouting;

Ipv4ListRoutingHelper list;
list.Add (staticRouting, 0);
//list.Add (olsr, 10);

InternetStackHelper stack;
stack.SetRoutingHelper(list);
stack.Install (sta);

//PointToPointHelper p2p;

Ipv4AddressHelper address;
address.SetBase ("192.168.1.0", "255.255.255.0");
Ipv4InterfaceContainer staIf;
staIf = address.Assign (staDevices);

  Ptr<Ipv4> stIpv4 = sta.Get(0)->GetObject<Ipv4> ();
  Ptr<Ipv4> stIpv4_1 = sta.Get(1)->GetObject<Ipv4> ();
  Ptr<Ipv4> stIpv4_2 = sta.Get(2)->GetObject<Ipv4> ();
  Ptr<Ipv4> stIpv4_3 = sta.Get(3)->GetObject<Ipv4> ();
  

  Ptr<Ipv4StaticRouting> staRoutes = staticRouting.GetStaticRouting (stIpv4);
  Ptr<Ipv4StaticRouting> staRoutes_1 = staticRouting.GetStaticRouting(stIpv4_1);
  Ptr<Ipv4StaticRouting> staRoutes_2 = staticRouting.GetStaticRouting(stIpv4_2);
  Ptr<Ipv4StaticRouting> staRoutes_3 = staticRouting.GetStaticRouting(stIpv4_3);

  staRoutes->AddHostRouteTo(Ipv4Address ("192.168.1.4"), Ipv4Address("192.168.1.2"), 1);
  staRoutes_1->AddHostRouteTo(Ipv4Address("192.168.1.4"), Ipv4Address("192.168.1.3"), 1);
  staRoutes_2->AddHostRouteTo(Ipv4Address("192.168.1.4"), Ipv4Address("192.168.1.4"), 1);
  staRoutes_3->AddHostRouteTo(Ipv4Address("192.168.1.4"), Ipv4Address("192.168.1.4"), 0);  

//stIpv4->GetInterfaceForDevice(staDevices.Get(0)
  

/*  Ptr<Ipv4> st = sta.Get (0)->GetObject<Ipv4> ();
  Ptr<Ipv4RoutingProtocol> rp_Gw = (st->GetRoutingProtocol ());
  Ptr<Ipv4StaticRouting> StaticRouting = staticRouting.GetStaticRouting(st);
  Ptr<Ipv4ListRouting> lrp_Gw = DynamicCast<Ipv4ListRouting> (rp_Gw);

Ptr<olsr::RoutingProtocol> olsrrp_Gw;

 for (uint32_t i = 0; i < lrp_Gw->GetNRoutingProtocols ();  i++)
    {
      int16_t priority;
      Ptr<Ipv4RoutingProtocol> temp = lrp_Gw->GetRoutingProtocol (i, priority);
     if (DynamicCast<olsr::RoutingProtocol> (temp))
        {
         olsrrp_Gw = DynamicCast<olsr::RoutingProtocol> (temp);
         olsrrp_Gw->AddHostNetworkAssociation (Ipv4Address ("192.168.1.3"), Ipv4Mask ("255.255.255.0"));
        }
    }
*/
//olsrrp_Gw->AddHostNetworkAssociation (Ipv4Address ("192.168.1.4"), Ipv4Mask ("255.255.255.0"));





/* ===== Setting applications ===== */

  DataRate dataRate = DataRate (1000000 * Mbps);
  //DataRate DataRateBE = DataRate (1000000 * MbpsBE);

  uint32_t destinationSTANumber = 3; //for one common traffic destination

  Ipv4Address destination = staIf.GetAddress(destinationSTANumber);
  Ptr<Node> dest = sta.Get(0);

         // uint32_t destinationSTANumber = 2;
         // destination = staIf.GetAddress(destinationSTANumber);
         // dest = sta.Get(destinationSTANumber);

  if (oneDest)
    {
      if (A_VO) 
        {
          PacketSinkHelper sink_A_VO ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1007));
          sink_A_VO.Install (dest);
        }
      if (VO) 
        {
          PacketSinkHelper sink_VO ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1006));
          sink_VO.Install (dest);
        }
      if (VI) 
        {
          PacketSinkHelper sink_VI ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1005));
          sink_VI.Install (dest);
        }
      if (A_VI) 
        {
          PacketSinkHelper sink_A_VI ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1004));
          sink_A_VI.Install (dest);
        }
      if (BE) 
        {
          PacketSinkHelper sink_BE ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1000));
          sink_BE.Install (dest);
        }
      if (BK) 
        {
          PacketSinkHelper sink_BK ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1001));
          sink_BK.Install (dest);
        }
    }

  //for(uint32_t i = 0; i < nSTA; i++) 
    //{
      //Ptr<Node> node = sta.Get(i);

      if (!oneDest) //overwrite for different traffic destinations
        {          
        
      if (A_VO) 
        {
          OnOffHelper onOffHelper_A_VO = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1007), dataRate, packetSize, 7, appsStart, simulationTime);
          onOffHelper_A_VO.Install(dest);
        }
      if (VO) 
        {
          OnOffHelper onOffHelper_VO = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1006), dataRate, packetSize, 6, appsStart, simulationTime);
          onOffHelper_VO.Install(dest);
        }
      if (VI) 
        {
          OnOffHelper onOffHelper_VI = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1005), dataRate, packetSize, 5, appsStart, simulationTime);
          onOffHelper_VI.Install(dest);
        }
      if (A_VI) 
        {
          OnOffHelper onOffHelper_A_VI = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1004), dataRate, packetSize, 4, appsStart, simulationTime);
          onOffHelper_A_VI.Install(dest);
        }
      if (BE) 
        {
          OnOffHelper onOffHelper_BE = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1000), dataRate, packetSize, 0, appsStartBE, simulationTime);
          onOffHelper_BE.Install(dest);
        }
      if (BK) 
        {
          OnOffHelper onOffHelper_BK = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1001), dataRate, packetSize, 1, appsStart, simulationTime);
          onOffHelper_BK.Install(dest);
        }
    }



/* ===== tracing configuration and running simulation === */

  SimulationHelper::PopulateArpCache ();
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  Simulator::Stop (simulationTime);

  //phy.EnablePcap ("wireshark", 1, 0);
  //phy.EnablePcap ("wireshark", 2, 1);
 // phy.EnablePcap ("wireshark", 3, 0); // sniffing to pcap file
 // AsciiTraceHelper ascii;
  //phy.EnableAsciiAll (ascii.CreateFileStream ("out.tr"));
  //phy.EnableAscii (ascii.CreateFileStream ("out.tr"), sta.Get (1)->GetDevice (1));
  //mac.EnableAsciiAll (ascii.CreateFileStream ("out2.tr"));

  FlowMonitorHelper flowmon_helper;
  Ptr<FlowMonitor> monitor = flowmon_helper.InstallAll ();
  monitor->SetAttribute ("StartTime", TimeValue (Seconds (calcStart) ) ); //Time from which flowmonitor statistics are gathered.
  monitor->SetAttribute ("DelayBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("JitterBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("PacketSizeBinWidth", DoubleValue (20));

/*AnimationInterface anim ("animation.xml");
  anim.SetMaxPktsPerTraceFile(100000);
  anim.EnablePacketMetadata (true);
  anim.EnableIpv4RouteTracking ("routingtable-wireless.xml", Seconds(4), Seconds(5), Seconds(0.25));// AddSourceDestination(3, "192.168.1.1");  //prints the routing table from source Node 3 to dest ip 10.1.1.1
*/

  Simulator::Run ();
  Simulator::Destroy ();



/* ===== printing results ===== */

  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon_helper.GetClassifier ());
  //monitor->SerializeToXmlFile ("out.xml", true, true);

  std::string proto;
  uint64_t txBytes = 0, rxBytes = 0, txPackets = 0, rxPackets = 0, lostPackets = 0;
  double throughput;
  Time delaySum = Seconds (0), jitterSum = Seconds (0);

  std::vector<uint64_t> txBytesPerTid     = std::vector<uint64_t> (8, 0);
  std::vector<uint64_t> rxBytesPerTid     = std::vector<uint64_t> (8, 0);
  std::vector<uint64_t> txPacketsPerTid   = std::vector<uint64_t> (8, 0);
  std::vector<uint64_t> rxPacketsPerTid   = std::vector<uint64_t> (8, 0);
  std::vector<uint64_t> lostPacketsPerTid = std::vector<uint64_t> (8, 0);
  std::vector<double>   throughputPerTid  = std::vector<double>   (8, 0.0);
  std::vector<Time>     delaySumPerTid    = std::vector<Time>     (8, Seconds (0) );
  std::vector<Time>     jitterSumPerTid   = std::vector<Time>     (8, Seconds (0) );

  std::map< FlowId, FlowMonitor::FlowStats > stats = monitor->GetFlowStats();
  for (std::map< FlowId, FlowMonitor::FlowStats >::iterator flow = stats.begin (); flow != stats.end (); flow++)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (flow->first);
      switch (t.protocol)
        {
          case (6):
            proto = "TCP";
            break;
          case (17):
            proto = "UDP";
            break;
          default:
            exit (1);
        }
      std::cout << "FlowID: " << flow->first << "(" << proto << " "
                << t.sourceAddress << "/" << t.sourcePort << " --> "
                << t.destinationAddress << "/" << t.destinationPort << ")" <<
      std::endl;

      std::cout << "  Tx bytes:\t"     << flow->second.txBytes << std::endl;
      std::cout << "  Rx bytes:\t"     << flow->second.rxBytes << std::endl;
      std::cout << "  Tx packets:\t"   << flow->second.txPackets << std::endl;
      std::cout << "  Rx packets:\t"   << flow->second.rxPackets << std::endl;
      std::cout << "  Lost packets:\t" << flow->second.lostPackets << std::endl;
      if (flow->second.rxPackets > 0)
        {
          //std::cout << "  Throughput:\t"   << flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000  << " Mb/s" << std::endl;
          std::cout << "  Throughput:\t"   << flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds ()  << " Mb/s" << std::endl;
          std::cout << "  Mean delay:\t"   << (double)(flow->second.delaySum / (flow->second.rxPackets)).GetMicroSeconds () / 1000 << " ms" << std::endl;    
          if (flow->second.rxPackets > 1)
            std::cout << "  Mean jitter:\t"  << (double)(flow->second.jitterSum / (flow->second.rxPackets - 1)).GetMicroSeconds () / 1000 << " ms" << std::endl;   
          else
            std::cout << "  Mean jitter:\t---"   << std::endl;
        }
      else
        {
          std::cout << "  Throughput:\t0 Mb/s" << std::endl;
          std::cout << "  Mean delay:\t---"    << std::endl;    
          std::cout << "  Mean jitter:\t---"   << std::endl;
        }

      uint16_t tid = t.destinationPort-1000;
      txBytesPerTid[tid]     += flow->second.txBytes;
      rxBytesPerTid[tid]     += flow->second.rxBytes;
      txPacketsPerTid[tid]   += flow->second.txPackets;
      rxPacketsPerTid[tid]   += flow->second.rxPackets;
      lostPacketsPerTid[tid] += flow->second.lostPackets;
      //throughputPerTid[tid]  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000 : 0);
      throughputPerTid[tid]  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds () : 0);
      delaySumPerTid[tid]    += flow->second.delaySum;
      jitterSumPerTid[tid]   += flow->second.jitterSum;

      txBytes     += flow->second.txBytes;
      rxBytes     += flow->second.rxBytes;
      txPackets   += flow->second.txPackets;
      rxPackets   += flow->second.rxPackets;
      lostPackets += flow->second.lostPackets;
      //throughput  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000 : 0);
      throughput  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds () : 0);
      delaySum    += flow->second.delaySum;
      jitterSum   += flow->second.jitterSum;
    }
/*
  for (uint16_t tid = 0; tid < 8; tid++)
    if ((tid != 2) && (tid != 3))
      {
        std::cout << "=======================TID: " << tid << " =====================================" << std::endl;

        std::cout << "  Tx bytes:\t"     << txBytesPerTid[tid]     << std::endl;
        std::cout << "  Rx bytes:\t"     << rxBytesPerTid[tid]     << std::endl;
        std::cout << "  Tx packets:\t"   << txPacketsPerTid[tid]   << std::endl;
        std::cout << "  Rx packets:\t"   << rxPacketsPerTid[tid]   << std::endl;
        std::cout << "  Lost packets:\t" << lostPacketsPerTid[tid] << std::endl;
        std::cout << "  Throughput:\t"   << throughputPerTid[tid]  << " Mb/s" << std::endl;
        if (rxPacketsPerTid[tid] > 0)
          {
            std::cout << "  Mean delay:\t"   << (double)(delaySumPerTid[tid] / (rxPacketsPerTid[tid])).GetMicroSeconds () / 1000 << " ms" << std::endl;    
            if (rxPackets > 1)  
              std::cout << "  Mean jitter:\t"  << (double)(jitterSumPerTid[tid] / (rxPacketsPerTid[tid] - 1)).GetMicroSeconds () / 1000  << " ms" << std::endl;   
            else
              std::cout << "  Mean jitter:\t---"   << std::endl;
          }
        else
          {
            std::cout << "  Mean delay:\t---"    << std::endl;    
            std::cout << "  Mean jitter:\t---"   << std::endl;
          }
      }

  std::cout << "=======================Total: =====================================" << std::endl;

  std::cout << "  Tx bytes:\t"     << txBytes     << std::endl;
  std::cout << "  Rx bytes:\t"     << rxBytes     << std::endl;
  std::cout << "  Tx packets:\t"   << txPackets   << std::endl;
  std::cout << "  Rx packets:\t"   << rxPackets   << std::endl;
  std::cout << "  Lost packets:\t" << lostPackets << std::endl;
  std::cout << "  Throughput:\t"   << throughput  << " Mb/s" << std::endl;
  if (rxPackets > 0)
    {
      std::cout << "  Mean delay:\t"   << (double)(delaySum / (rxPackets)).GetMicroSeconds () / 1000 << " ms" << std::endl;    
      if (rxPackets > 1)  
        std::cout << "  Mean jitter:\t"  << (double)(jitterSum / (rxPackets - 1)).GetMicroSeconds () / 1000  << " ms" << std::endl;   
      else
        std::cout << "  Mean jitter:\t---"   << std::endl;
    }
  else
    {
      std::cout << "  Mean delay:\t---"    << std::endl;    
      std::cout << "  Mean jitter:\t---"   << std::endl;
    }
*/
  return 0;
}
