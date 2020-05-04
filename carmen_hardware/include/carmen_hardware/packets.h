

struct _Header
{
   uint8_t version;
   uint8_t messageType;
   uint8_t errorCode;

   // Error ?
}

typedef struct _Header Header;

struct _ReadSettingsCommand
{
    Header header;

    uint16_t p;
    uint16_t i;
    uint16_t d;
}

typedef struct _ReadSettingsCommand ReadSettingsCommand;

struct _ReadSettingsResult
{
    Header header;

    uint16_t p;
    uint16_t i;
    uint16_t d;
}

typedef struct _ReadSettingsResult ReadSettingsResult;




