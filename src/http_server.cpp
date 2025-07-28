#include "http_server.h"

WebServer HTTPServer::server(HTTP_SERVER_PORT);
bool HTTPServer::serverRunning = false;

void HTTPServer::init() {
    if (!WiFi.isConnected()) {
        Serial.println("DEBUG::http_server.cpp Cannot start HTTP server - WiFi not connected");
        return;
    }
    
    // Setup routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/command", HTTP_POST, handleCommand);
    server.onNotFound(handleNotFound);
    
    // Start server
    server.begin();
    serverRunning = true;
    
    Serial.println("DEBUG::http_server.cpp HTTP server started on port " + String(HTTP_SERVER_PORT));
    Serial.println("DEBUG::http_server.cpp Server IP: " + WiFi.localIP().toString());
    Serial.println("DEBUG::http_server.cpp Available endpoints:");
    Serial.println("DEBUG::http_server.cpp   GET  / - Device info");
    Serial.println("DEBUG::http_server.cpp   GET  /status - System status");
    Serial.println("DEBUG::http_server.cpp   POST /command - Execute commands");
}

void HTTPServer::handleClient() {
    if (serverRunning) {
        server.handleClient();
    }
}

bool HTTPServer::isRunning() {
    return serverRunning;
}

String HTTPServer::getServerIP() {
    return WiFi.localIP().toString();
}

void HTTPServer::handleRoot() {
    setCORSHeaders();
    
    DynamicJsonDocument doc(512);
    doc["device"] = "Human Detection Sensor";
    doc["version"] = "1.0.0";
    doc["device_id"] = WiFi.macAddress();
    doc["ip_address"] = WiFi.localIP().toString();
    doc["uptime_ms"] = millis();
    doc["endpoints"] = JsonArray();
    doc["endpoints"].add("/status - GET system status");
    doc["endpoints"].add("/command - POST commands (arm, disarm, status, test_leds, reboot)");
    
    String response;
    serializeJson(doc, response);
    
    server.send(200, "application/json", response);
    Serial.println("DEBUG::http_server.cpp Root endpoint accessed from " + server.client().remoteIP().toString());
}

void HTTPServer::handleStatus() {
    setCORSHeaders();
    
    // Check for API key in query parameter
    String apiKey = server.arg("api_key");
    if (apiKey != API_KEY) {
        sendJsonResponse(401, false, "Invalid API key");
        return;
    }
    
    CommandResponse response = CommandHandler::handleStatusCommand();
    sendJsonResponse(response.httpCode, response.success, response.message, response.data);
    
    Serial.println("DEBUG::http_server.cpp Status endpoint accessed from " + server.client().remoteIP().toString());
}

void HTTPServer::handleCommand() {
    setCORSHeaders();
    
    if (!server.hasArg("plain")) {
        sendJsonResponse(400, false, "No JSON body provided");
        return;
    }
    
    String jsonBody = server.arg("plain");
    Serial.println("DEBUG::http_server.cpp Command received from " + server.client().remoteIP().toString());
    Serial.println("DEBUG::http_server.cpp JSON: " + jsonBody);
    
    CommandResponse response = CommandHandler::processCommand(jsonBody);
    sendJsonResponse(response.httpCode, response.success, response.message, response.data);
}

void HTTPServer::handleNotFound() {
    setCORSHeaders();
    
    String message = "Endpoint not found\n\n";
    message += "Available endpoints:\n";
    message += "GET  / - Device info\n";
    message += "GET  /status?api_key=YOUR_KEY - System status\n";
    message += "POST /command - Execute commands\n";
    
    server.send(404, "text/plain", message);
    Serial.println("DEBUG::http_server.cpp 404 - Unknown endpoint: " + server.uri());
}

void HTTPServer::sendJsonResponse(int code, bool success, const String& message, const String& data) {
    DynamicJsonDocument doc(1024);
    
    doc["success"] = success;
    doc["message"] = message;
    doc["timestamp"] = millis();
    
    if (data.length() > 0) {
        DynamicJsonDocument dataDoc(512);
        DeserializationError error = deserializeJson(dataDoc, data);
        if (!error) {
            doc["data"] = dataDoc;
        } else {
            doc["data"] = data;
        }
    }
    
    String response;
    serializeJson(doc, response);
    
    server.send(code, "application/json", response);
}

void HTTPServer::setCORSHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
} 