#include <Arduino.h>
// Downloaded from https://github.com/teebr/Influx-Arduino
#include <InfluxArduino.hpp>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "Config.h"
#include "RootCert.hpp"


#define RELAY_PIN   15
#define NTP_SERVER  "pool.ntp.org"
//Summar PST is GMT -7hrs
#define NTP_OFFSET  -7*3600
#define NTP_INTERVAL  15*1000
#define CONNECTION_RETRY 10

const char RUNNING_MEASUREMENT[] = "open_valve";

Config config;
InfluxArduino influx;
String Tags;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, NTP_OFFSET, NTP_INTERVAL);
bool misting = false;
unsigned long mistingStopTime;
unsigned long nextMistingTime;

void connectToWifi()
{
    WiFi.mode(WIFI_STA);
    while (true)
    {
        Serial.print("Connecting to Wifi network ");
        Serial.print(config.WifiSSID);
        Serial.print(" ");
        WiFi.scanNetworks();
        WiFi.begin(config.WifiSSID.c_str(), config.WifiPassword.c_str());
        for (int x = 0; x < CONNECTION_RETRY; x++)
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                Serial.println("Connected");
                return;
            }
            Serial.print(".");
            delay(500);
        }
    }
}

bool sendDatapoint(const char *measurement, const char *tags, const char *fields)
{
    // Make sure there is a connection
    if (WiFi.status() != WL_CONNECTED)
    {
        connectToWifi();
    }

    // Send the data point
    if (!influx.write(measurement, tags, fields))
    {
        Serial.print("ERROR sending ");
        Serial.print(measurement);
        Serial.print(": ");
        Serial.println(influx.getResponse());
        return false;
    }

    return true;
}

void reconfigureCheck()
{
    if (Serial.available())
    {
        char code = Serial.read();
        if (code == 'i' || code == 'I')
        {
            printConfig(config);
            return;
        }
        else if (code == 'c' || code == 'C')
        {
            // Reconfigure the sensor
            askForSettings(config);
        }
    }
}


void enableMisters()
{
    Serial.print(timeClient.getFormattedTime());
    Serial.println(" - Misters ON");
    digitalWrite(RELAY_PIN, HIGH);
    sendDatapoint(RUNNING_MEASUREMENT, Tags.c_str(), "value=1");
}

void disableMisters()
{
    Serial.print(timeClient.getFormattedTime());
    Serial.println(" - Misters OFF");
    digitalWrite(RELAY_PIN, LOW);

    // Send the last running point and then the first not running point.
    sendDatapoint(RUNNING_MEASUREMENT, Tags.c_str(), "value=1");
    sendDatapoint(RUNNING_MEASUREMENT, Tags.c_str(), "value=0");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("####### ESP32 Sensor INIT #######");

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    loadConfig(config);
    if (config.Magic != CONFIG_MAGIC)
    {
        askForSettings(config);
    }

    // wait to see if user wants to update settings
    delay(1000);
    reconfigureCheck();
    Tags = "location=" + config.Location + ",sensor=" + config.Sensor;

    connectToWifi();

    Serial.print("Getting time from NTP server...");
    timeClient.begin();
    timeClient.update();
    Serial.println("Done");

    Serial.print("Current Time: ");
    Serial.println(timeClient.getFormattedTime());
    unsigned long nowSeconds = timeClient.getEpochTime();
    nextMistingTime = nowSeconds + (config.Frequency*60) - (nowSeconds % (config.Frequency*60));
    misting = false;
    Serial.print("First misting time: ");
    Serial.println(nextMistingTime);

    Serial.print("Setting up influxdb connection...");

    influx.configure(config.InfluxDatabase.c_str(), config.InfluxHostname.c_str());
    influx.authorize(config.InfluxUser.c_str(), config.InfluxPassword.c_str());
    influx.addCertificate(ROOT_CERT);
    Serial.println("Done");

    Serial.println("Setup Complete. Entering run loop");
}

void loop()
{
    timeClient.update();

    int currentHour = timeClient.getHours();
    unsigned long nowSeconds = timeClient.getEpochTime();

    // Is the current time within the window that the misters are enabled?
    if (currentHour >= config.StartHour && currentHour < config.StopHour) {
        // Misters are enabled during this time.
        // Do they need to be turned on right now?
        if (!misting && nowSeconds >= nextMistingTime) {
            misting = true;
            mistingStopTime = nowSeconds + config.Duration;
            nextMistingTime = nowSeconds + (config.Frequency*60);
            enableMisters();
        }
    }

    // If the misters are running and the duration has been met, then turn them off
    if (misting && nowSeconds >= mistingStopTime) {
        disableMisters();
        misting = false;
        Serial.print("Next misting time: ");
        Serial.println(nextMistingTime);
    }

    if (nowSeconds % 60 == 0) {
        Serial.print("Current time: ");
        Serial.print(timeClient.getFormattedTime());
        Serial.print(" / ");
        Serial.println(nowSeconds);

        if (!misting) {
            // Sends the metric only if the misters are not currently running
            sendDatapoint(RUNNING_MEASUREMENT, Tags.c_str(), "value=0");
        }
    }
    delay(1000);
}