#pragma once

#include <Arduino.h>

void favoriotSetup();
void favoriotLoop();
void favoriotAction(const char* action);
void favoriotTaggingResult(uint32_t id, const char* status,
                           int tag = -1, const char* color = nullptr);
void favoriotSortCompleted();
void favoriotSortCancelled();
