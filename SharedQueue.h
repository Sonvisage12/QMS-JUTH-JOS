
#ifndef SHAREDQUEUE_H
#define SHAREDQUEUE_H

#include <Arduino.h>
#include <vector>
#include <Preferences.h>
#include <RTClib.h>

struct QueueEntry {
  String uid;
  String timestamp;
  int number;
  int status; 
};

struct QueueItem {
  char uid[20];
  char timestamp[25];
  int number;
  bool removeFromQueue;
};

class SharedQueue {
public:
  SharedQueue(const String& ns);
  void load();
  void save();
  void print();
  void add(const String& uid, const String& timestamp, int number);
  void addIfNew(const String& uid, const String& timestamp, int number);
  void removeByUID(const String& uid);
  bool exists(const String& uid);
  int getOrAssignPermanentNumber(const String& uid, const DateTime& now);
  std::vector<QueueEntry>& getQueue();

private:
  std::vector<QueueEntry> queue;
  Preferences prefs;
  String namespaceStr;
  int counter;

  void sortQueue();
};

#endif
