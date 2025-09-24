package loranetwork.simulations;

import inet.applications.udpapp.UDPBasicApp;
import inet.node.inet.Router;
import inet.node.internetcloud.InternetCloud;
import loranetwork.LoRaPhy.LoRaMedium;
import loranetwork.LoraNode.LoRaNode;
import loranetwork.LoraNode.endRescueNode;
//import loranetwork.LoraNode.endNode;
import loranetwork.LoraNode.LoRaGW;
import inet.node.inet.StandardHost;
import inet.networklayer.configurator.ipv4.IPv4NetworkConfigurator;
import inet.node.ethernet.Eth1G;

@license(LGPL);
network LoRaNetworkTest
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfEndNodes = default(1);
        //        int numberOfGateways = default(1);
        int networkSizeX = default(500);
        int networkSizeY = default(500);

        @display("bgb=355.824,356.832");
    submodules:
        //        loRaEndNodes[numberOfEndNodes]: endNode {
        //            @display("p=83.664,233.856");
        //        }
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=83.664,150.192;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=167.328,88.704");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=208.24002,27.216002");
        }
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=207.64801,88.704");
        }
        internetCloud: InternetCloud {
            @display("p=83.664,27.216002");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=24.192001,27.216002");
        }
        nsRouter: Router {
            @display("p=138.09601,27.216002");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}

network LoRaMotoNetworkTest
{
    parameters:
        int numberOfNodes = default(1);
        //        int numberOfEndNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(1400);
        int networkSizeY = default(2500);
        //@display("bgb=1400,2500;bgi=background/coquimbo-02;bgl=2");
        @display("bgb=2500,2500;bgl=2");
    submodules:
        N[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856;is=n");
        }
        //        loRaEndNodes[numberOfEndNodes]: endNode {
        //            @display("p=83.664,233.856");
        //        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=100,100;is=n");
        }
        LoRaMedium: LoRaMedium {
            @display("p=20,35;is=l;i=misc/sun");
        }
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=207.64801,88.704");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=208.24002,27.216002");
        }
    connections:
}


network LoRaMesh0
{
    parameters:
        int numberOfNodes = default(1);
        //        int numberOfEndNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(1400);
        int networkSizeY = default(2500);
        string mapPath = default("map/uni");
        int mapWidth = default(1000);
        int mapHeight = default(1000);

        //@display("bgb=1400,2500;bgi=background/coquimbo-02;bgl=2");
        //        @display("bgb=6000,4500;bgi=map/uni,s;bgg=1000,2,grey95;bgu=km");
        //		@display("bgb=1000,1000;bgi=$mapPath,s;bgg=500,2,grey95;bgu=m");
        @display("bgb=$mapWidth,$mapHeight;bgi=$mapPath,s;bgg=500,2,grey95;bgu=m");



    //        @display("bgb=2500,2500;bgl=2");
    submodules:
        //        loRaEndNodes[numberOfEndNodes]: endNode {
        //            @display("p=83.664,233.856");
        //        }
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856;is=n");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=83.664,150.192;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=200,200");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=850,-100");
        }
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=1200,-100");
        }
        internetCloud: InternetCloud {
            @display("p=300,-100");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=50,-100");
        }
        nsRouter: Router {
            @display("p=550,-100");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}


network LoRaMesh
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfEndNodes = default(1);
        int numberOfRescueNodes = default(0);
        //        int numberOfEndNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(1400);
        int networkSizeY = default(2500);
        string mapPath = default("map/uni");
        int mapWidth = default(1000);
        int mapHeight = default(1000);

        //@display("bgb=1400,2500;bgi=background/coquimbo-02;bgl=2");
        //        @display("bgb=6000,4500;bgi=map/uni,s;bgg=1000,2,grey95;bgu=km");
        //		@display("bgb=1000,1000;bgi=$mapPath,s;bgg=500,2,grey95;bgu=m");
        @display("bgb=$mapWidth,$mapHeight;bgi=$mapPath,s;bgg=500,2,grey95;bgu=m");



    //        @display("bgb=2500,2500;bgl=2");
    submodules:
        //        loRaEndNodes[numberOfEndNodes]: endNode {
        //            @display("p=83.664,233.856");
        //        }
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=134,287;is=n");
        }

        loRaEndNodes[numberOfEndNodes]: LoRaNode {
            @display("i=device/receiverdish;p=71,174;is=n");
        }

        loRaRescueNodes[numberOfRescueNodes]: endRescueNode {
            @display("i=device/ambulance;p=200,174;is=n");
        }

        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=106,98;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=251,158");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=850,-100");
        }
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=1200,-100");
        }
        internetCloud: InternetCloud {
            @display("p=300,-100");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=50,-100");
        }
        nsRouter: Router {
            @display("p=550,-100");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}

@statistic[EV](source=LoRaNode; record=count);
@statistic[EV](source=LoRaGW; record=count);
@statistic[EV](source=LoRaMedium; record=count);

