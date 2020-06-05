#include "orion_packet.h"

enum MessageType { ReadSettings = 1, ExecuteCommand = 2 };

struct _ReadSettingsCommand
{
    OrionDataHeader header = {1, ReadSettings};

    uint16_t p;
    uint16_t i;
    uint16_t d;
}

typedef struct _ReadSettingsCommand ReadSettingsCommand;

struct _ReadSettingsResult
{
    OrionDataHeader header = {1, ReadSettings};

    uint16_t p;
    uint16_t i;
    uint16_t d;
}

typedef struct _ReadSettingsResult ReadSettingsResult;




