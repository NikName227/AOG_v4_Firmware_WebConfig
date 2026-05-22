void EthernetStart()
{
    SLOG("Initialising Ethernet...");

    Ethernet.begin(mac, 0);   // start with IP 0.0.0.0

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        SLOG("Ethernet hardware not found - GPS via USB only.");
        return;
    }
    if (Ethernet.linkStatus() == LinkOFF) {
        SLOG("Ethernet cable not connected - continuing anyway.");
    }

    // Build IP from EEPROM network address
    Eth_myip[0] = networkAddress.ipOne;
    Eth_myip[1] = networkAddress.ipTwo;
    Eth_myip[2] = networkAddress.ipThree;
    Eth_myip[3] = Autosteer_running ? 126 : 120;  // 126=steer+GPS  120=GPS only

    Ethernet.setLocalIP(Eth_myip);

    Eth_ipDestination[0] = Eth_myip[0];
    Eth_ipDestination[1] = Eth_myip[1];
    Eth_ipDestination[2] = Eth_myip[2];
    Eth_ipDestination[3] = 255;

    Serial.print("Module IP : "); Serial.println(Ethernet.localIP());
    Serial.print("Sending to: "); Serial.println(Eth_ipDestination);
    webLogf("Module IP: %d.%d.%d.%d",
        Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3]);

    if (Eth_udpPAOGI.begin(portMy)) {
        Serial.print("GPS UDP port     : "); Serial.println(portMy);
        webLogf("GPS UDP port: %u", portMy);
    }
    if (Eth_udpNtrip.begin(AOGNtripPort)) {
        Serial.print("NTRIP UDP port   : "); Serial.println(AOGNtripPort);
        webLogf("NTRIP UDP port: %u", AOGNtripPort);
    }
    if (Eth_udpAutoSteer.begin(AOGAutoSteerPort)) {
        Serial.print("Autosteer UDP port: "); Serial.println(AOGAutoSteerPort);
        webLogf("Autosteer UDP port: %u", AOGAutoSteerPort);
    }
}
