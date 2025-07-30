// fixé sont adresse ip a 192.168.1.1   255.255.255.0   192.168.1.1

#include "DifferentialServer.h"

DifferentialServer::DifferentialServer(uint16_t port) : server(port) {}

void DifferentialServer::begin() {
    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
    IPAddress ip(192, 168, 1, 177);
    Ethernet.begin(mac, ip);
    // Ethernet.begin(mac); // utiliser DHCP au lieu d’une IP fixe
    server.begin();
}

bool DifferentialServer::update(int &v1, int &v2, int &v3, int &v4, int &v5, int &v6, int &v7) {
    EthernetClient client = server.available();
    if (client) {
        String request = client.readStringUntil('\n');
        // Serial.println("Requête brute :");
        // Serial.println(request);

        client.flush();

        int values[7] = {0};
        int count = 0;

        int idx = request.indexOf("?");
        while (idx >= 0 && count < 7) {
            int eq = request.indexOf('=', idx);
            int andSep = request.indexOf('&', eq);
            if (eq != -1) {
                String valueStr = request.substring(eq + 1, andSep == -1 ? request.length() : andSep);
                values[count] = valueStr.toInt();
                count++;
                idx = andSep;
            } else break;
        }

        if (count == 7) {
            v1 = values[0];
            v2 = values[1];
            v3 = values[2];
            v4 = values[3];
            v5 = values[4];
            v6 = values[5];
            v7 = values[6];

            // client.println("HTTP/1.1 200 OK");
            // client.println("Content-Type: text/plain");
            // client.println("Connection: close");
            // client.println();
            client.println("Received");
            client.stop();
            return true;
        }

        client.stop();
    }
    return false;
}
