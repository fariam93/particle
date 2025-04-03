#include "Particle.h"

#include "tracker_config.h"
#include "tracker.h"

#include "Grove_Temperature_And_Humidity_Sensor.h"
#include "TemperatureHumidityValidatorRK.h"
#include "MQTT-TLS.h"  // MQTT Library

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#ifndef SYSTEM_VERSION_v400ALPHA1
PRODUCT_ID(TRACKER_PRODUCT_ID);
#endif
PRODUCT_VERSION(TRACKER_PRODUCT_VERSION);

// MQTT Broker Details
const char* mqttServer = "broker.hivemq.com"; // Public MQTT broker
const int mqttPort = 1883; // Default MQTT port

// MQTT Client
MQTT mqttClient(mqttServer, mqttPort, callback); // MQTT client instance

// Callback function to handle incoming messages (if needed)
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE, {
    { "app.gps.nmea", LOG_LEVEL_INFO },
    { "app.gps.ubx",  LOG_LEVEL_INFO },
    { "ncp.at", LOG_LEVEL_INFO },
    { "net.ppp.client", LOG_LEVEL_INFO },
});

// Library: Grove_Temperature_And_Humidity_Sensor
DHT tempSensor(A1);

// Library: TemperatureHumidityValidatorRK
TemperatureHumidityValidator validator;

// Sample the temperature sensor every 2 seconds. This is done so the outlier values can be filtered out easily.
const unsigned long CHECK_PERIOD_MS = 2000;
unsigned long lastCheck = 0;

void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context); // Forward declaration

// Function to read temperature from the TMP36/LM35 sensor on A0.
// For TMP36: Temperature (°C) = (Vout - 0.5) * 100
// For LM35, use: Temperature (°C) = Vout * 100 (i.e., no 0.5V offset)
// Adjust the formula below if using LM35.
float readAnalogTemperature() {
    int raw = analogRead(A0);
    // Convert raw ADC reading to voltage.
    // Assuming a 12-bit ADC (0-4095) and a reference voltage of 3.3V.
    float voltage = raw * (3.3f / 4095.0f);
    float temperatureC = (voltage - 0.5f) * 100.0f;
    return temperatureC;
}

// Function to compute a combined temperature using a weighted average
// between the validated DHT sensor and the analog sensor.
float getCombinedTemperature() {
    float tempDHT = validator.getTemperatureC();
    float tempAnalog = readAnalogTemperature();
    
    // If one of the sensors provides an invalid reading, use the other.
    if (isnan(tempDHT)) return tempAnalog;
    if (isnan(tempAnalog)) return tempDHT;
    
    // Use weights based on which sensor you determine performs better.
    // For example, 70% weight for the DHT sensor and 30% for the analog sensor.
    float weightDHT = 0.7f;
    float weightAnalog = 0.3f;
    
    return (weightDHT * tempDHT) + (weightAnalog * tempAnalog);
}


void setup()
{
    Tracker::instance().init();
    
    // Callback to add key press information to the location publish
    Tracker::instance().location.regLocGenCallback(myLocationGenerationCallback);

    // Initialize temperature sensor
    tempSensor.begin();

    Particle.connect();
    waitUntil(Particle.connected);

    // Connect to MQTT Broker
    if (mqttClient.connect("ParticleTracker")) {
        Serial.println("Connected to MQTT Broker!");
    } else {
        Serial.println("Failed to connect to MQTT Broker!");
    }

}

void loop()
{
    Tracker::instance().loop();

    if (millis() - lastCheck >= CHECK_PERIOD_MS) {
        lastCheck = millis();

        validator.addSample(tempSensor.getTempCelcius(), tempSensor.getHumidity());

        // Log.info("tempC=%f tempF=%f humidity=%f", validator.getTemperatureC(), validator.getTemperatureF(), validator.getHumidity());

        float tempC = validator.getTemperatureC();
        float tempAnalog = readAnalogTemperature();
        float combinedTemp = getCombinedTemperature();
        float hum = validator.getHumidity();

        // Create MQTT JSON payload
        char payload[150];
        snprintf(payload, sizeof(payload), 
            "{\"temp_dht\": %.2f, \"temp_analog\": %.2f, \"temp_combined\": %.2f, \"humidity\": %.1f}", 
            tempC, tempAnalog, combinedTemp, hum);

        // Publish data to MQTT topic
        mqttClient.publish("tracker/temperature", payload);
        Serial.println("Data Published: " + String(payload));

        // Maintain MQTT connection
        mqttClient.loop();
    }
}


void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{
    float tempC = validator.getTemperatureC();
    if (!isnan(tempC)) {
        writer.name("temp").value(tempC, 2);
    }

    // Add analog sensor temperature.
    float tempAnalog = readAnalogTemperature();
    if (!isnan(tempAnalog)) {
        writer.name("temp_analog").value(tempAnalog, 2);
    }
    
    // Add combined weighted temperature.
    float combinedTemp = getCombinedTemperature();
    if (!isnan(combinedTemp)) {
        writer.name("temp_combined").value(combinedTemp, 2);
    }

    float hum = validator.getHumidity();
    if (!isnan(hum)) {
        writer.name("hum").value(hum, 1);
    }

}
