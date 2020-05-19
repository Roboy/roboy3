#include "wirelessLove.hpp"

const char  TestBuffer[] ="Hello World";
const int    timestampSize = 2;

uint64_t pack754(long double f, unsigned bits, unsigned expbits)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (f == 0.0) return 0; // get this special case out of the way

    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }

    // get the normalized form of f and track the exponent
    shift = 0;
    while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;

    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL<<significandbits) + 0.5f);

    // get the biased exponent
    exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

    // return the final answer
    return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

long double unpack754(uint64_t i, unsigned bits, unsigned expbits)
{
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (i == 0) return 0.0;

    // pull the significand
    result = (i&((1LL<<significandbits)-1)); // mask
    result /= (1LL<<significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1<<(expbits-1)) - 1;
    shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= (i>>(bits-1))&1? -1.0: 1.0;

    return result;
}

WirelessLove::WirelessLove(const char* SSID, const char* PASSWD, IPAddress &broadcastIP):
broadcastIP(broadcastIP){
    uint32_t timoutCounter = 0;

    // check for presence of the WiFiShield:
    if(WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi shield not present");
    }

    connected = WiFi.begin(SSID,PASSWD);
}

uint32_t WirelessLove::getLocalIP()
{
    return  WiFi.localIP();
}

void WirelessLove::printWifiStatus(void)
{
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

bool WirelessLove::connect(const char* ssid, const char* passwd, IPAddress &bIP){
  broadcastIP = bIP;
  connected = WiFi.begin(ssid,passwd);
}

bool WirelessLove::initUDPSockets(void)
{
    return UDPbroadcast.begin(broadcast_port);
}

bool WirelessLove::broadcast_send(const uint8_t * buffer, size_t size)
{
    if(0 == UDPbroadcast.beginPacket(broadcastIP, broadcast_port))
    {
        Serial.println("Can not connect to the supplied IP or PORT");
        return  false;
    }

    if(size != UDPbroadcast.write(buffer, size)){
        Serial.println("Size of the UDP Package to big! Truncated overlapping data");
    }
    UDPbroadcast.endPacket();
    return true;
}
