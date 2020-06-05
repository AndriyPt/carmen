
struct _OrionRequestHeader
{
   uint32_t packetId;
};

typedef _OrionRequestHeader OrionRequestHeader;

struct _OrionResponseHeader
{
   uint32_t packetId;
   uint8_t errorCode;
};

typedef _OrionResponseHeader OrionResponseHeader;

struct _OrionResponse
{
    OrionResponseHeader header;
    uint8_t body[0];
}

typedef _OrionResponse OrionResponse;

struct _OrionDataHeader
{
   uint8_t version;
   uint8_t messageType;
};

typedef _OrionDataHeader OrionDataHeader;
