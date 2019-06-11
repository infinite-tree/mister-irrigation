/*
    Config.cpp

    Simple config loading and saving based (ripped off) from the examples in ArduinoJson
*/

#include "Config.h"
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

void loadConfig(Config &config) {
    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Failed");
    }
    File file = SPIFFS.open(CONFIG_FILE, "r");
    if (!file)
    {
        Serial.println("Failed to open config file");
        return;
    }
    // Allocate a temporary JsonDocument
    // Don't forget to change the capacity to match your requirements.
    // Use arduinojson.org/v6/assistant to compute the capacity.
    StaticJsonDocument<CONFIG_SIZE> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error)
        Serial.println(F("Failed to read file, using default configuration"));

    // Copy values from the JsonDocument to the Config
    config.Magic = doc["Magic"].as<String>();

    config.WifiSSID = doc["WifiSSID"].as<String>();
    config.WifiPassword = doc["WifiPassword"].as<String>();

    config.Location = doc["Location"].as<String>();
    config.Sensor = doc["Sensor"].as<String>();

    config.InfluxHostname = doc["InfluxHostname"].as<String>();
    config.InfluxDatabase = doc["InfluxDatabase"].as<String>();
    config.InfluxUser = doc["InfluxUser"].as<String>();
    config.InfluxPassword = doc["InfluxPassword"].as<String>();

    config.Frequency = doc["Frequency"];
    config.Duration = doc["Duration"];
    config.StartHour = doc["StartHour"];
    config.StopHour = doc["StopHour"];

    file.close();
}

void saveConfig(Config &config) {
    SPIFFS.begin(true);
    SPIFFS.remove(CONFIG_FILE);

    // Open file for writing
    File file = SPIFFS.open(CONFIG_FILE, "w");
    if (!file)
    {
        Serial.println(F("Failed to create file"));
        return;
    }

    // Allocate a temporary JsonDocument
    // Don't forget to change the capacity to match your requirements.
    // Use arduinojson.org/assistant to compute the capacity.
    StaticJsonDocument<CONFIG_SIZE> doc;

    // Set the values in the document
    doc["Magic"] = config.Magic;

    doc["WifiSSID"] = config.WifiSSID;
    doc["WifiPassword"] = config.WifiPassword;

    doc["Location"] = config.Location;
    doc["Sensor"] = config.Sensor;

    doc["InfluxHostname"] = config.InfluxHostname;
    doc["InfluxDatabase"] = config.InfluxDatabase;
    doc["InfluxUser"] = config.InfluxUser;
    doc["InfluxPassword"] = config.InfluxPassword;

    doc["Frequency"] = config.Frequency;
    doc["Duration"] = config.Duration;
    doc["StartHour"] = config.StartHour;
    doc["StopHour"] = config.StopHour;

    // Serialize JSON to file
    if (serializeJson(doc, file) == 0)
    {
        Serial.println(F("Failed to write to file"));
    }

    // Close the file
    file.close();
}

void printBool(bool value)
{
    if (value)
    {
        Serial.println("Yes");
    }
    else
    {
        Serial.println("No");
    }
}

void printConfig(Config &config)
{
    Serial.println("########## Device Setting ##########");

    Serial.print("Wifi SSID: ");
    Serial.println(config.WifiSSID);
    Serial.print("Wifi Password: ");
    Serial.println(config.WifiPassword);
    Serial.println();

    Serial.print("Location: ");
    Serial.println(config.Location);
    Serial.print("Sensor Name: ");
    Serial.println(config.Sensor);
    Serial.println();

    Serial.print("Influxdb Host: ");
    Serial.println(config.InfluxHostname);
    Serial.print("Influxdb Database: ");
    Serial.println(config.InfluxDatabase);
    Serial.print("Influxdb Username: ");
    Serial.println(config.InfluxUser);
    Serial.print("Influxdb Password: ");
    Serial.println(config.InfluxPassword);
    Serial.println();

    Serial.print("Frequency (in minutes): ");
    Serial.println(config.Frequency);
    Serial.print("Duration (in seconds): ");
    Serial.println(config.Duration);
    Serial.print("Start time: ");
    Serial.print(config.StartHour);
    Serial.println(":00");
    Serial.print("Stop time: ");
    Serial.print(config.StopHour);
    Serial.println(":00");

    Serial.println();
}

String readString()
{
    String value = "";
    char c;
    while (true)
    {
        if (Serial.available())
        {
            c = Serial.read();
            if (c == '\n' || c == '\r')
            {
                // clear anything else that might be waiting
                c = Serial.read();
                Serial.println();
                return value;
            }
            else
            {
                Serial.print(c);
                value += c;
            }
        }
        delay(3);
    }
}

bool readBool()
{
    char c;
    bool ret = false;
    while (true)
    {
        if (Serial.available())
        {
            c = Serial.read();
            if (c == 'y' || c == 'Y')
            {
                Serial.print(c);
                ret = true;
            }
            else if (c == 'n' || c == 'N')
            {
                Serial.print(c);
                ret = false;
            }
            else if (c == '\n' || c == '\r')
            {
                // clear anything else that might be waiting
                c = Serial.read();
                Serial.println();
                return ret;
            }
        }
        delay(3);
    }
}

int readInt()
{
    String input = readString();
    return input.toInt();

}

float readFloat()
{
    String input = readString();
    return input.toFloat();
}

void askForSettings(Config &config)
{
    Serial.println("########## Please configure the device ##########");

    Serial.print("Wifi SSID? ");
    config.WifiSSID = readString();
    Serial.print("Wifi Password? ");
    config.WifiPassword = readString();

    Serial.println();

    Serial.print("Device Location? ");
    config.Location = readString();
    Serial.print("Sensor name? ");
    config.Sensor = readString();

    Serial.println();

    Serial.print("Influxdb host? ");
    config.InfluxHostname = readString();
    Serial.print("Influxdb database? ");
    config.InfluxDatabase = readString();
    Serial.print("Influxdb username? ");
    config.InfluxUser = readString();
    Serial.print("Influxdb password? ");
    config.InfluxPassword = readString();

    Serial.println();

    Serial.print("Mister Frequency in minutes (int)? ");
    config.Frequency = readInt();
    Serial.print("Mister Duration in seconds (int)? ");
    config.Duration = readInt();
    Serial.print("Start time (hour)? ");
    config.StartHour = readInt();
    Serial.print("Stop time (hour)? ");
    config.StopHour = readInt();

    Serial.println();

    config.Magic = CONFIG_MAGIC;
    saveConfig(config);
    loadConfig(config);
    if (config.Magic != CONFIG_MAGIC)
    {
        Serial.println("ERROR: failed to save config");
    }
    else
    {
        Serial.println("Settings saved!");
    }
}
