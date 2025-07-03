#include <SPI.h>
#include <MFRC522.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "PatientRFIDMappings.h"
#include <esp_now.h>
#include <WiFi.h>
#include <map>
#include <vector>
#include <algorithm>
#include "SharedQueue.h"
#include <Wire.h>
#include "RTClib.h"
#define IR_SENSOR_PIN 16  // GPIO 0
#include <ESP32Servo.h>
#define SERVO_PIN 26
Servo myServo;
bool isOpen = false;
bool arbiter = true;
int status;
#define RST_PIN  5
#define SS_PIN   4
#define GREEN_LED_PIN 15
#define RED_LED_PIN   2
//#define BLUE_LED_PIN   
#define BUZZER   27
MFRC522 mfrc522(SS_PIN, RST_PIN);
Preferences prefs;
int k;
const int DOCTOR_QUEUE_POSITION = 10;  // Position for doctor's immediate queue
const int MAX_QUEUE_SIZE = 600;        // Maximum queue size
const int STATUS_WAITING = 0;
const int STATUS_CALLED = 1;
const int STATUS_COMPLETED = 2;
bool waitingForCard = false;
SharedQueue sharedQueue("rfid-patients");
DateTime parseDateTime(String timestamp);
void processCardUnified(String uid);
void printAllQueues();
void clearAllQueues();

bool isMaster = false;          // True if this node is the current master
int myIndex = 0;               // This node's index in arrivalMACs
int currentMasterIndex = 0;     // Index of the currently known master
//const int numDoctorNodes = sizeof(doctorMACs) / sizeof(doctorMACs[0]);

RTC_DS3231 rtc;
bool isArrivalNode = false, isDoctorNode = false;
std::map<String, unsigned long> recentUIDs;
const unsigned long UID_CACHE_TTL_MS = 2000;
const uint8_t arrivalMACs[][6] = {
    
    {0x78, 0x1C, 0x3C, 0x2D, 0xA2, 0xA4},
    {0x00, 0x4B, 0x12, 0x97, 0x2E, 0xA4},  //78:1C:3C:2D:A2:A4
    {0x5C, 0x01, 0x3B, 0x97, 0x54, 0xB4}, //00:4B:12:97:2E:A4
    {0x78, 0x1C, 0x3C, 0xE6, 0x6C, 0xB8}, //78:1C:3C:E6:6C:B8
    {0x78, 0x1C, 0x3C, 0xE3, 0xAB, 0x30}, //78:1C:3C:E3:AB:30
    {0x5C, 0x01, 0x3B, 0x98, 0xDB, 0x04}, //5C:01:3B:98:DB:04
    {0x78, 0x42, 0x1C, 0x6C, 0xE4, 0x9C} //78:42:1C:6C:E4:9C
};
const int numArrivalNodes = sizeof(arrivalMACs) / sizeof(arrivalMACs[0]);

const uint8_t doctorMACs[][6] = {
    {0x78, 0x42, 0x1C, 0x6C, 0xA8, 0x3C},
    {0x5C, 0x01, 0x3B, 0x98, 0x3C, 0xEC},
    {0x5C, 0x01, 0x3B, 0x98, 0xE8, 0x2C},//5C:01:3B:98:E8:2C
    {0x78, 0x1C, 0x3C, 0xE5, 0x50, 0x0C}
};

const int numDoctorNodes = sizeof(doctorMACs) / sizeof(doctorMACs[0]);  // 👈 Must be AFTER doctorMACs is declared
// Function declarations
void handleDoctorRemoval(const SourceInfo& source, const QueueItem& item, const String& uidStr);
void handleClearQueue(const SourceInfo& source);
void handleMasterChange(const SourceInfo& source);
void updateQueueStatus(const String& uid, bool incrementStatus);
void sendPatientToDoctor(const SourceInfo& source, const QueueEntry& entry);
void broadcastStatusUpdate(const QueueEntry& entry);
void notifyNoPatients(int doctorNodeID);
void createMixedQueue();
void broadcastAllQueues();


void broadcastToArrivalNodes(const QueueItem &item) {
String myMAC = WiFi.macAddress();
if (myMAC == String(item.sourceMAC)) {
    Serial.println("🔁 Received own broadcast. Skipping rebroadcast.");
    return;
    for (int i = 0; i < numArrivalNodes; i++) {
        esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
    }
    }
}


int getPermanentNumber(String uid) {
    prefs.begin("rfidMap", true); int pid=-1;
    if (prefs.isKey(uid.c_str())) pid = prefs.getUInt(uid.c_str(),-1);
    prefs.end(); return pid;
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent 🟢" : "Failed 🔴");
}

void printAllQueues() {
    Serial.println("\n📋 Current Queue Status:");
    Serial.println("------------------------------------------------");
    Serial.println("| UID            | Number | Status  | Timestamp          |");
    Serial.println("------------------------------------------------");

    std::vector<QueueEntry> entries = sharedQueue.getAll();
    
    for (const auto& entry : entries) {
        // Format status description
        String statusDesc;
        switch(entry.status) {
            case 0: statusDesc = "Waiting   "; break;
            case 1: statusDesc = "Called    "; break;
            case 2: statusDesc = "Completed "; break;
            case 3: statusDesc = "Delete "; break;
            default: statusDesc = "Unknown   "; break;
        }

        // Print formatted entry
        Serial.printf("| %-14.14s | %-6d | %s | %-18.18s |\n", 
                     entry.uid, 
                     entry.number, 
                     statusDesc.c_str(),
                     entry.timestamp);
    }

    Serial.println("------------------------------------------------");
    Serial.printf("Total patients in queue: %d\n", entries.size());
    Serial.println();
}

void clearAllQueues() {
    sharedQueue.clear(); 
    Serial.println("🔄 All queues cleared.");
}
  DateTime parseDateTime(String timestamp) {
    int yr, mo, dy, hr, mn, sc;
    sscanf(timestamp.c_str(), "%d-%d-%d %d:%d:%d", &yr, &mo, &dy, &hr, &mn, &sc);
    return DateTime(yr, mo, dy, hr, mn, sc);
}

void processCardUnified(String uid, bool isLocalScan = true) {
    DateTime now = rtc.now();
    char timeBuffer[25];
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    String timeStr = String(timeBuffer);

    int pid = getPermanentNumber(uid);
    if (pid == -1) pid = 0;

    QueueItem item;
    strncpy(item.uid, uid.c_str(), sizeof(item.uid));
    strncpy(item.timestamp, timeStr.c_str(), sizeof(item.timestamp));
    item.number = pid;
    item.addToQueue = true;
    item.removeFromQueue = false;

    // Check existing patient status
    QueueEntry existing;
    if (sharedQueue.getByUID(uid, existing)) {
        if (existing.status == 1) {
            // Patient was called before, now re-registering
            existing.status = 2;
            sharedQueue.updateEntry(existing);
            item.status = 2;
            Serial.println("🔄 Re-registered returning patient (status 2).");
            if (isLocalScan) blinkLED(GREEN_LED_PIN);
        } else if (existing.status == 0) {
            // Patient already waiting
            Serial.println("⚠️ UID already in queue (status 0). Ignoring scan.");
            if (isLocalScan) blinkLED(RED_LED_PIN);
            return;
        } else if (existing.status == 2) {
            // Completed patient registering again
            existing.status = 0; // Reset to waiting
            sharedQueue.updateEntry(existing);
            item.status = 0;
            Serial.println("🔁 Completed patient re-registered (status reset to 0).");
            if (isLocalScan) blinkLED(GREEN_LED_PIN);
        }
    } else {
        // New patient
        item.status = 0;
        sharedQueue.add(uid, timeStr, pid, status);
        Serial.println("✅ New UID added to sharedQueue (status 0).");
        if (isLocalScan) blinkLED(GREEN_LED_PIN);
    }

    // Broadcast/Send the update
    if (isMaster) {
        for (int i = 0; i < numArrivalNodes; i++) {
            if (i != myIndex) {
                esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
            }
        }
        Serial.println("📤 Broadcasted patient update to arrival nodes.");
    } else {
        esp_now_send(arrivalMACs[currentMasterIndex], (uint8_t*)&item, sizeof(item));
        Serial.println("📤 Sent patient info to master only.");
    }

    printAllQueues();
}


void notifyDoctorsOfNewMaster() {
    if (myIndex < 0 || myIndex >= numArrivalNodes) {
        Serial.printf("❌ Cannot notify doctors: invalid myIndex = %d\n", myIndex);
        return;
    }

    QueueItem announce = {};
    announce.number = 0; // Optional, set to 0 or omit if not needed
    announce.addToQueue = false;
    announce.removeFromQueue = false;
    strcpy(announce.uid, "MASTER_CHANGE");
    strcpy(announce.type, "MASTER");

    uint8_t myMAC[6];
    WiFi.macAddress(myMAC);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             myMAC[0], myMAC[1], myMAC[2], myMAC[3], myMAC[4], myMAC[5]);
    strncpy(announce.sourceMAC, macStr, sizeof(announce.sourceMAC));

    for (int i = 0; i < numDoctorNodes; ++i) {
        announce.node = i + 1;
        esp_now_send(doctorMACs[i], (uint8_t*)&announce, sizeof(announce));
    }

    Serial.printf("📢 New master announced (MAC: %s) to all doctors.\n", announce.sourceMAC);
}


void sendQueueToAllArrivalNodes(SharedQueue& queue) {
    std::vector<QueueEntry> entries = queue.getAll();
    String myMAC = WiFi.macAddress();

    for (const auto& entry : entries) {
        QueueItem item;
        strncpy(item.uid, entry.uid, sizeof(item.uid));
        strncpy(item.timestamp, entry.timestamp, sizeof(item.timestamp));
        item.number = entry.number;
        item.node = 0; // Not tied to a specific doctor node
        item.status = entry.status;
        item.addToQueue = true;
        item.removeFromQueue = false;

        // Identify this node’s MAC
        snprintf(item.sourceMAC, sizeof(item.sourceMAC), "%s", myMAC.c_str());

        broadcastToArrivalNodes(item);
    }

    Serial.println("📡 Broadcasted full queue with status to all arrival nodes.");
}



void broadcastAllQueues() {
  const std::vector<QueueEntry>& all = sharedQueue.getAll();  // Get full queue
  QueueItem item;

  for (const QueueEntry& entry : all) {
    // Fill QueueItem
    strncpy(item.uid, entry.uid, sizeof(item.uid) - 1);
    item.uid[sizeof(item.uid) - 1] = '\0';

    strncpy(item.timestamp, entry.timestamp, sizeof(item.timestamp) - 1);
    item.timestamp[sizeof(item.timestamp) - 1] = '\0';

    item.number = entry.number;
    item.status = entry.status;

    item.addToQueue = true;
    item.removeFromQueue = false;

    // Optional: use "main" or "" depending on your protocol
    strncpy(item.queueID, "main", sizeof(item.queueID) - 1);
    item.queueID[sizeof(item.queueID) - 1] = '\0';

    // Broadcast to all arrival nodes
    for (int i = 0; i < numArrivalNodes; ++i) {
      esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
    }
  }

  Serial.printf("📡 Broadcasted %d queue entries with status to all arrival nodes.\n", all.size());
}



void cleanupRecentUIDs() {
    unsigned long now = millis();
    for (auto it = recentUIDs.begin(); it != recentUIDs.end(); ) {
        if (now - it->second > UID_CACHE_TTL_MS)
            it = recentUIDs.erase(it);
        else
            ++it;
    }
}


void sendTimeToAllArrivalNodes() {
    DateTime now = rtc.now();
    char timeBuffer[25];
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

    QueueItem timeSync;
    strcpy(timeSync.uid, "RTC_SYNC");
    strncpy(timeSync.timestamp, timeBuffer, sizeof(timeSync.timestamp));
    timeSync.addToQueue = false;
    timeSync.removeFromQueue = false;

    for (int i = 0; i < numArrivalNodes; i++) {
        if (i == myIndex) continue;  // Don't send to self
        esp_now_send(arrivalMACs[i], (uint8_t*)&timeSync, sizeof(timeSync));
    }

    Serial.printf("🕒 Master broadcasted RTC time to arrival nodes: %s\n", timeBuffer);
}



void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
    QueueItem item;
    memcpy(&item, incomingData, sizeof(item));
    String uidStr(item.uid);
    // memcpy(&item, incomingData, sizeof(item));
    const uint8_t* mac = recvInfo->src_addr;
    
 char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("📩 Received from: "); Serial.println(macStr);

    isArrivalNode = isDoctorNode = false;
    for (int i = 0; i < numArrivalNodes; i++)
        if (memcmp(mac, arrivalMACs[i], 6) == 0) { isArrivalNode = true; break; }
    for (int i = 0; i < 4; i++)
        if (memcmp(mac, doctorMACs[i], 6) == 0) { isDoctorNode = true;
     break; }

        if (isArrivalNode) { Serial.println("🔄 Handling Arrival Node message...");}
                  
 if (isArrivalNode) {
 
    SourceInfo source = identifySource(recvInfo->src_addr);
    handleArrivalMessage(source, item, uidStr);
    } 
    else if (isDoctorNode) {
     
    SourceInfo source = identifySource1(recvInfo->src_addr);
        handleDoctorMessage(source, item, uidStr);
        Serial.println("🔄 Handling Doctor's Node message...");
   }



}

// Helper structures and functions


SourceInfo identifySource(const uint8_t* mac) {
    SourceInfo info;
    char macBuffer[18];
    snprintf(macBuffer, sizeof(macBuffer), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    info.macStr = String(macBuffer);

    // Check arrival nodes
    for (int i = 0; i < numArrivalNodes; ++i) {
        if (memcmp(mac, arrivalMACs[i], 6) == 0) { 
            info.fromArrival = true; 
            info.srcIndex = i; 
            break; 
        }
    }

   
    return info;
}

SourceInfo identifySource1(const uint8_t* mac) {
    SourceInfo info;
    char macBuffer[18];
    snprintf(macBuffer, sizeof(macBuffer), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    info.macStr = String(macBuffer);

    // Check doctor nodes
    for (int i = 0; i < numDoctorNodes; ++i) {
        if (memcmp(mac, doctorMACs[i], 6) == 0) { 
            info.fromDoctor = true; 
            info.doctorNodeID = i+1; 
            break; 
        }
    }

    return info;
}

void handleArrivalMessage(const SourceInfo& source, const QueueItem& item, const String& uidStr) {
    Serial.printf("📩 Arrival message from node %d (%s)\n", source.srcIndex, source.macStr.c_str());
    cleanupRecentUIDs();

    // Handle RTC sync
    if (strcmp(item.uid, "RTC_SYNC") == 0) {
        handleRTCSync(source, item);
        return;
    }

    // Handle add to queue
    if (item.addToQueue && !item.removeFromQueue) {
        handleAddToQueue(source, item, uidStr);
    } 
    // Handle remove from queue
    else if (!item.addToQueue && item.removeFromQueue) {
        handleRemoveFromQueue(source, item, uidStr);
    }
}

void handleDoctorMessage(const SourceInfo& source, const QueueItem& item, const String& uidStr) {
    Serial.printf("📩 Doctor message from Node %d (%s)\n", source.doctorNodeID, source.macStr.c_str());
     Serial.print("📩 this is the doctors ID ");
      Serial.println(source.doctorNodeID);
    if (strcmp(item.uid, "REQ_NEXT") == 0) {
        handleDoctorRequest(source);
    } 
    else if (item.removeFromQueue) {
        handleDoctorRemoval(source, item, uidStr);
    } 
    else if (strcmp(item.uid, "CLEAR_QUEUE") == 0) {
        handleClearQueue(source);
    }
    
    printAllQueues();
}

// Individual handler implementations would go here
void handleRTCSync(const SourceInfo& source, const QueueItem& item) {
    static unsigned long lastRTCUpdate = 0;
    static bool rtcSetOnce = false;

    if (!isMaster && source.srcIndex == currentMasterIndex) {
        unsigned long nowMillis = millis();
        if (!rtcSetOnce || nowMillis - lastRTCUpdate >= 7200000UL) {
            DateTime newTime = parseDateTime(String(item.timestamp));
            rtc.adjust(newTime);
            lastRTCUpdate = nowMillis;
            rtcSetOnce = true;
            Serial.printf("🕒 RTC updated from master (%d) to: %s\n", currentMasterIndex, item.timestamp);
        } else {
            Serial.println("⏳ Ignoring RTC sync: less than 2 hours since last update.");
        }
    } else {
        Serial.printf("⚠️ Ignoring RTC_SYNC: not from current master (%d), came from %d\n", 
                     currentMasterIndex, source.srcIndex);
    }
}

void handleAddToQueue(const SourceInfo& source, const QueueItem& item, const String& uidStr) {
    if (recentUIDs.count("add:" + uidStr)) {
        Serial.println("⏳ Duplicate add event, ignoring.");
        return;
    }
    recentUIDs["add:" + uidStr] = millis();

    // Check for master change
    if (!isMaster && source.srcIndex != currentMasterIndex) {
        handleMasterChange(source);
    }

    // Process the add event
    processCardUnified(uidStr, false);
}

void handleRemoveFromQueue(const SourceInfo& source, const QueueItem& item, const String& uidStr) {
    // 1. Check for duplicate removal requests
    if (recentUIDs.count("rem:" + uidStr)) {
        Serial.println("⏳ Duplicate remove event, ignoring.");
        return;
    }
    recentUIDs["rem:" + uidStr] = millis();
   
    // 2. Master node verification and update if needed
    if (!isMaster && source.srcIndex != currentMasterIndex && source.srcIndex != myIndex) {
        Serial.printf("🔁 Master updated to node %d (for removal event).\n", source.srcIndex);
        currentMasterIndex = source.srcIndex;
    }

    // 3. Get the current queue entry or add new if not exists
    QueueEntry entry;
    if (!sharedQueue.getByUID(uidStr, entry)) {
        // Patient not in queue - add new with status 0
        DateTime now = rtc.now();
        char timeBuffer[25];
        snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
                 now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
        
        int pid = getPermanentNumber(uidStr);
        if (pid == -1) pid = 0;
         status= entry.status;
        sharedQueue.add(uidStr, String(timeBuffer), pid, status);
        Serial.printf("✅ Added new patient %s with status 0\n", uidStr.c_str());
        return;
    }

    // 4. Status-based processing
    switch(entry.status) {
        case 0: // Waiting - send to doctor (insert at position 10)
         
                Serial.printf("🔼 Patient %s inserted at position 10 for doctor\n", uidStr.c_str());
           
            break;
            
        case 1: // Called - increment to completed (2)
            entry.status = 2;
            sharedQueue.updateEntry(entry);
            Serial.printf("✅ Patient %s status updated to Completed (2)\n", uidStr.c_str());
            break;
            
        case 2: // Completed - remove from queue
            sharedQueue.removeByUID(uidStr);
            Serial.printf("🗑️ Patient %s removed from queue\n", uidStr.c_str());
            break;
            
        default:
            Serial.println("⚠️ Invalid patient status");
            return;
    }

    // 5. Broadcast the update if master
    if (isMaster) {
        QueueItem update;
        strncpy(update.uid, entry.uid, sizeof(update.uid));
        update.number = entry.number;
        update.status = entry.status;
        update.addToQueue = (entry.status != 2);
        update.removeFromQueue = (entry.status == 2);
        
        for (int i = 0; i < numArrivalNodes; i++) {
            if (i != myIndex) {
                esp_now_send(arrivalMACs[i], (uint8_t*)&update, sizeof(update));
            }
        }
        Serial.println("📢 Broadcasted queue update to all nodes");
    }
}

void handleDoctorRequest(const SourceInfo& source) {
    Serial.println("👨‍⚕️ Doctor requests next patient");
    
    if (!isMaster) {
        Serial.println("⚠️ Current master unresponsive. Becoming master to serve doctor.");
        isMaster = true;
        currentMasterIndex = myIndex;
    }

    // Static variables to maintain batch state
    static int status2Count = 0;
    static int status0Count = 0;
    const int MAX_STATUS_2 = 5;
    const int MAX_STATUS_0 = 3;

    // Check available patients
    bool hasStatus0 = false;
    bool hasStatus2 = false;
    std::vector<QueueEntry> allPatients = sharedQueue.getAll();
    
    for (const auto& patient : allPatients) {
        if (patient.status == 0) hasStatus0 = true;
        else if (patient.status == 2) hasStatus2 = true;
        if (hasStatus0 && hasStatus2) break;
    }

    // Find patient to process
    QueueEntry patientToSend;
    bool foundPatient = false;

    // Only mix if both statuses available
    if (hasStatus0 && hasStatus2) {
        if (status2Count < MAX_STATUS_2) {
            // Find oldest status 2 patient
            for (const auto& patient : allPatients) {
                if (patient.status == 2) {
                    patientToSend = patient;
                    foundPatient = true;
                    status2Count++;
                    break;
                }
            }
        }
        else if (status0Count < MAX_STATUS_0) {
            // Find oldest status 0 patient
            for (const auto& patient : allPatients) {
                if (patient.status == 0) {
                    patientToSend = patient;
                    foundPatient = true;
                    status0Count++;
                    if (status0Count == 1) status2Count = 0; // Reset counter
                    break;
                }
            }
        }
    }
    else {
        // Send whatever is available
        if (hasStatus2) {
            // Send oldest status 2
            for (const auto& patient : allPatients) {
                if (patient.status == 2) {
                    patientToSend = patient;
                    foundPatient = true;
                    break;
                }
            }
        }
        else if (hasStatus0) {
            // Send oldest status 0
            for (const auto& patient : allPatients) {
                if (patient.status == 0) {
                    patientToSend = patient;
                    foundPatient = true;
                    break;
                }
            }
        }
    }

    // Reset counters when both quotas are filled
    if (status2Count >= MAX_STATUS_2 && status0Count >= MAX_STATUS_0) {
        status2Count = 0;
        status0Count = 0;
    }

    if (!foundPatient) {
        notifyNoPatients(source.doctorNodeID);
        return;
    }

    // Process the selected patient
    sharedQueue.removeByUID(patientToSend.uid);
    
    if (patientToSend.status == 0) {
        // Update status to called (1) and requeue
        patientToSend.status = 1;
        sharedQueue.add(patientToSend.uid, patientToSend.timestamp, 
                       patientToSend.number, patientToSend.status);
        Serial.printf("✅ Called patient %s (0→1), requeued\n", patientToSend.uid);
    } else {
        Serial.printf("✔️ Completed patient %s removed\n", patientToSend.uid);
    }

    // Send to doctor
    sendPatientToDoctor(source, patientToSend);

    // Broadcast updates if master
    if (isMaster) {
        QueueItem update;
        strncpy(update.uid, patientToSend.uid, sizeof(update.uid));
        strncpy(update.timestamp, patientToSend.timestamp, sizeof(update.timestamp));
        update.number = patientToSend.number;
        update.status = patientToSend.status;
        update.addToQueue = (patientToSend.status != 2);
        update.removeFromQueue = (patientToSend.status == 2);
        
        for (int i = 0; i < numArrivalNodes; i++) {
            if (i != myIndex) {
                esp_now_send(arrivalMACs[i], (uint8_t*)&update, sizeof(update));
            }
        }
        Serial.println("📢 Broadcasted update");
    }

    printAllQueues();
}



bool hasEligiblePatients() {
    std::vector<QueueEntry> allEntries = sharedQueue.getAll();
    for (const auto& entry : allEntries) {
        if (entry.status == 0 || entry.status == 2) {
            return true;
        }
    }
    return false;
}



void sendPatientToDoctor(const SourceInfo& source, const QueueEntry& entry) {
    QueueItem item;
    strncpy(item.uid, entry.uid, sizeof(item.uid));
    strncpy(item.timestamp, entry.timestamp, sizeof(item.timestamp));
    item.number = entry.number;
    item.status = entry.status;
    item.addToQueue = false;
    item.removeFromQueue = false;
     item.node= source.doctorNodeID;
    esp_err_t result = esp_now_send(doctorMACs[source.doctorNodeID-1], (uint8_t*)&item, sizeof(item));
    if (result != ESP_OK) {
        Serial.println("Error sending patient to doctor");
    }
}

void broadcastStatusUpdate(const QueueEntry& entry) {
    QueueItem item;
    strncpy(item.uid, entry.uid, sizeof(item.uid));
    strncpy(item.timestamp, entry.timestamp, sizeof(item.timestamp));
    item.number = entry.number;
    item.status = entry.status;
    item.addToQueue = true;
    item.removeFromQueue = false;
    
    for (int i = 0; i < numArrivalNodes; i++) {
        if (i != myIndex) {
            esp_now_send(arrivalMACs[i], (uint8_t*)&item, sizeof(item));
        }
    }
}

void notifyNoPatients(int doctorNodeID) {
    QueueItem item;
    strncpy(item.uid, "NO_PATIENT", sizeof(item.uid));
    item.number = 0;
    item.status = 0;
    item.addToQueue = false;
    item.removeFromQueue = false;
    
    esp_now_send(doctorMACs[doctorNodeID-1], (uint8_t*)&item, sizeof(item));
}




// void handleDoctorRemoval(const SourceInfo& source, const QueueItem& item, const String& uidStr) {
//     if (!sharedQueue.exists(uidStr)) {
//         Serial.printf("Patient %s not found in queue\n", uidStr.c_str());
//         return;
//     }

//     QueueEntry entry;
//     if (!sharedQueue.getByUID(uidStr, entry)) {
//         Serial.printf("Failed to get queue entry for %s\n", uidStr.c_str());
//         return;
//     }

//     // Only process if this is a removal request from doctor
//     if (item.removeFromQueue && entry.status == 2) {
//         // Remove patient from queue
//         sharedQueue.removeByUID(uidStr);  // No return value check since it's void
//         Serial.printf("Doctor %d removed completed patient: %s\n", 
//                     source.doctorNodeID, uidStr.c_str());
        
//         // Broadcast removal if master
//         if (isMaster) {
//             QueueItem update;
//             strncpy(update.uid, entry.uid, sizeof(update.uid));
//             update.number = entry.number;
//             update.status = entry.status;
//             update.addToQueue = false;
//             update.removeFromQueue = true;
            
//             // Broadcast to all nodes
//             for (int i = 0; i < numArrivalNodes; i++) {
//                 if (i != myIndex) {
//                     esp_now_send(arrivalMACs[i], (uint8_t*)&update, sizeof(update));
//                 }
//             }
//             Serial.println("📢 Broadcasted removal to all nodes");
//         }
//     } 
//     else if (entry.status == 0) {
//         // Handle status update to 1 (if needed)
//         entry.status = 1;
//         sharedQueue.updateEntry(entry);  // No return value check since it's likely void
//         Serial.printf("Doctor %d updated patient status to Called (1): %s\n", 
//                     source.doctorNodeID, uidStr.c_str());

//         if (isMaster) {
//             QueueItem update;
//             strncpy(update.uid, entry.uid, sizeof(update.uid));
//             strncpy(update.timestamp, entry.timestamp, sizeof(update.timestamp));
//             update.number = entry.number;
//             update.status = entry.status;
//             update.addToQueue = true;
//             update.removeFromQueue = false;
            
//             for (int i = 0; i < numArrivalNodes; i++) {
//                 if (i != myIndex) {
//                     esp_now_send(arrivalMACs[i], (uint8_t*)&update, sizeof(update));
//                 }
//             }
//             Serial.println("📢 Broadcasted status update to arrival nodes");
//         }
//     }

//     printAllQueues();
// }


void handleDoctorRemoval(const SourceInfo& source, const QueueItem& item, const String& uidStr) {
    if (!sharedQueue.exists(uidStr)) {
        Serial.printf("⚠️ Patient %s not found in queue\n", uidStr.c_str());
        return;
    }

    QueueEntry entry;
    if (!sharedQueue.getByUID(uidStr, entry)) {
        Serial.printf("❌ Failed to get queue entry for %s\n", uidStr.c_str());
        return;
    }

    // Handle removal request for completed patients (status 2)
    if (item.removeFromQueue && entry.status == 2) {
        sharedQueue.removeByUID(uidStr);
        Serial.printf("✅ Doctor %d removed COMPLETED patient: %s\n", 
                    source.doctorNodeID, uidStr.c_str());
        
        if (isMaster) {
            QueueItem update;
            strncpy(update.uid, entry.uid, sizeof(update.uid));
            update.number = entry.number;
            update.status = entry.status;
            update.addToQueue = false;
            update.removeFromQueue = true;
            
            // Broadcast removal to all nodes
            for (int i = 0; i < numArrivalNodes; i++) {
                if (i != myIndex) {
                    esp_now_send(arrivalMACs[i], (uint8_t*)&update, sizeof(update));
                }
            }
            Serial.println("📢 Broadcasted removal notification");
        }
        printAllQueues();
        return;
    }

    // Handle status progression (only forward)
    if (entry.status == 0) {
        // Only allow status increase (0 → 1)
        entry.status = 1;
        sharedQueue.updateEntry(entry);
        Serial.printf("🔄 Doctor %d updated patient status to CALLED (1): %s\n", 
                    source.doctorNodeID, uidStr.c_str());

        if (isMaster) {
            QueueItem update;
            strncpy(update.uid, entry.uid, sizeof(update.uid));
            strncpy(update.timestamp, entry.timestamp, sizeof(update.timestamp));
            update.number = entry.number;
            update.status = entry.status;
            update.addToQueue = true;
            update.removeFromQueue = false;
            
            for (int i = 0; i < numArrivalNodes; i++) {
                if (i != myIndex) {
                    esp_now_send(arrivalMACs[i], (uint8_t*)&update, sizeof(update));
                }
            }
            Serial.println("📢 Broadcasted status update");
        }
    }
    else if (entry.status == 1) {
        // Status 1 patients remain unchanged unless explicitly completed
        Serial.printf("ℹ️ Patient %s is already CALLED (status 1)\n", uidStr.c_str());
    }
    else if (entry.status == 2) {
        // Status 2 patients should only be removed (handled above)
        Serial.printf("ℹ️ Patient %s is COMPLETED (status 2)\n", uidStr.c_str());
    }

    printAllQueues();
}
void handleClearQueue(const SourceInfo& source) {
    sharedQueue.clear();
    Serial.printf("Doctor %d cleared the queue\n", source.doctorNodeID);
}

void handleMasterChange(const SourceInfo& source) {
    if (!isMaster && source.srcIndex > currentMasterIndex) {
        isMaster = true;
        currentMasterIndex = myIndex;
        Serial.println("This node is now master");
    }
}

void updateQueueStatus(const String& uid, bool incrementStatus) {
    QueueEntry entry;
    if (sharedQueue.getByUID(uid, entry)) {
        if (incrementStatus) {
            entry.status = min(entry.status + 1, 2); // Cap status at 2
        }
        sharedQueue.updateEntry(entry);
    }
}

void createMixedQueue() {
    // Implementation depends on your queue mixing logic
    // Example:
   // sharedQueue.clear();
    // Add logic to combine queues if needed
    Serial.println("Created mixed queue");
}




 void setup() {


    Serial.begin(115200);
    prefs.begin("rfidMap", false);
    loadRFIDMappings(prefs);  
     pinMode(IR_SENSOR_PIN, INPUT);
  myServo.attach(SERVO_PIN, 500, 2500);  // Correct for ESP32 + SG90
  myServo.write(0);  // Start at 0 degrees
    //clearAllQueues();
   
myIndex = 0;
for (; myIndex < numArrivalNodes; ++myIndex) {
    if (memcmp(WiFi.macAddress().c_str(), arrivalMACs[myIndex], 6) == 0) break;
}
bool isMaster = (myIndex == 0);
size_t currentMasterIndex = 0;

String myMAC = WiFi.macAddress();
Serial.print("This node MAC: "); Serial.println(myMAC);

for (int i = 0; i < numArrivalNodes; i++) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             arrivalMACs[i][0], arrivalMACs[i][1], arrivalMACs[i][2],
             arrivalMACs[i][3], arrivalMACs[i][4], arrivalMACs[i][5]);
    if (myMAC.equalsIgnoreCase(String(macStr))) {
        myIndex = i;
        break;
    }
}

isMaster = (myIndex == 0);  // Node 0 is default master
currentMasterIndex = 0;




    SPI.begin(); WiFi.mode(WIFI_STA);
WiFi.setTxPower(WIFI_POWER_19_5dBm); 
 esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);   
  Serial.print("WiFi MAC: "); Serial.println(WiFi.macAddress());
    mfrc522.PCD_Init();
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
   //  pinMode(BLUE_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, LOW );
    digitalWrite(RED_LED_PIN, LOW);
    //digitalWrite(BLUE_LED_PIN, HIGH);
    if (!rtc.begin()) {
        Serial.println("❌ Couldn't find RTC module! Check wiring.");
        while (1);
    }

    if (rtc.lostPower()) {
        Serial.println("⚠️ RTC lost power, setting to compile time.");
        rtc.adjust(DateTime(__DATE__, __TIME__));
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW Init Failed");
        return;
    }

    for (int i = 0; i < numArrivalNodes; i++) {
        esp_now_peer_info_t p = {};
        memcpy(p.peer_addr, arrivalMACs[i], 6);
        p.channel = 1;
        esp_now_add_peer(&p);
    }

    for (int i = 0; i < 4; i++) {
        esp_now_peer_info_t p = {};
        memcpy(p.peer_addr, doctorMACs[i], 6);
        p.channel = 1;
        esp_now_add_peer(&p);
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
 if (isMaster) {
        delay(100);  // small delay to ensure peers are ready (optional)
        sendTimeToAllArrivalNodes();  // master sends current RTC time to all others
    }
    sharedQueue.load();
  
// if(arbiter){
// createMixedQueue();broadcastAllQueues(); //Only arbiter should contain the following  
// }
     printAllQueues();
}




void loop() {
  if (!waitingForCard) {
    // Check for object using IR sensor
    int sensorValue = digitalRead(IR_SENSOR_PIN);
    delay(50);
    if (sensorValue == LOW) {
      Serial.println("🚫 Object detected by IR sensor.");
      Serial.println("🔄 Dispensing card...");

     // myServo.write(60);  // Open or dispense
     // delay(1000);        // Let the card fall
     // myServo.write(0);   // Close again
      delay(500);         // Optional debounce delay

      waitingForCard = true;  // Now expect card scan
    }
  } 
  else {
    // Wait for RFID card scan
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      String uid = getUIDString(mfrc522.uid.uidByte, mfrc522.uid.size);
      k = 1;

      processCardUnified(uid, true);  // Applies for both arbiter and arrival

      mfrc522.PICC_HaltA(); 
      mfrc522.PCD_StopCrypto1(); 
      delay(1200);

      if (arbiter) {
        createMixedQueue();       // Only arbiter does this
        broadcastAllQueues();     // Only arbiter does this
      }

      k = 0;
      printAllQueues();
      waitingForCard = false;  // Go back to IR monitoring
    }
  }

  // Master/arbiter syncs time every 2 hours
  static unsigned long lastSyncTime = 0;
  unsigned long nowMillis = millis();

  if (isMaster && nowMillis - lastSyncTime >= 7200000UL) {  // 2 hours = 7200000 ms
    sendTimeToAllArrivalNodes();
    lastSyncTime = nowMillis;
  }
}


String getUIDString(byte *buf, byte size) {
    String uid=""; for (byte i=0;i<size;i++)
    {if(buf[i]<0x10)uid+="0"; uid+=String(buf[i],HEX);} uid.toUpperCase(); return uid;
}

void blinkLED(int pin) {
    //digitalWrite(BLUE_LED_PIN , LOW);
    digitalWrite(pin, HIGH);// digitalWrite(BUZZER,HIGH ); 
    delay(500); 
    //digitalWrite(BLUE_LED_PIN , HIGH);
    digitalWrite(pin, LOW); //digitalWrite(BUZZER, LOW );
}


