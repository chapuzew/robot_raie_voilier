#ifndef DIFFERENTIALSERVER_H
#define DIFFERENTIALSERVER_H

#include <Ethernet.h>

class DifferentialServer {
public:
    DifferentialServer(uint16_t port = 80);
    void begin();
    bool update(int &v1, int &v2, int &v3, int &v4, int &v5, int &v6, int &v7);

private:
    EthernetServer server;
};

#endif
