#pragma once

#include <Arduino.h>

void favoriotSetup();
void favoriotLoop();
void favoriotAction(const char* action);
void favoriotSortStarted(int tag, const char* color);
void favoriotSortCompleted();
void favoriotSortCancelled();
