/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef __PROTOCOLMESSAGE
#define __PROTOCOLMESSAGE

#include <QObject>
#include <QDebug>
#include <QByteArray>

const char STX = 0xff;
const char ETX = 0xfe;
const char DEL = 0xfd;

const char STX_STUFF = 0x02;
const char ETX_STUFF = 0x01;
const char DEL_STUFF = 0x00;

enum MessageParsingState
{
    WAITING_FOR_STX,
    WAITING_FOR_ETX
};

enum MessageCheck
{
    NO_CHECK,
    USE_CHECKSUM,
    USE_CRC
};

class ProtocolMessage
{
public:
    ProtocolMessage(MessageCheck check = NO_CHECK);

    bool checkMessage();
    bool composeMessage(QByteArray& data, QByteArray& data2);
    void createMessage(QByteArray& data);

    QByteArray& content() { return m_messageBuffer; }
    int length() { return m_messageBuffer.length(); }

private:
    bool calculateChecksum(QByteArray& dest, QByteArray& src);
    bool checkChecksum(QByteArray& src);

    bool calculateCRC(QByteArray& dest, QByteArray& src);
    bool checkCRC(QByteArray& src);

    bool addStuffing(QByteArray& dest, QByteArray& src);
    bool removeStuffing(QByteArray& dest, QByteArray& src);

private:
    MessageParsingState     m_parsingState;
    QByteArray              m_messageBuffer;

    MessageCheck            m_messageCheck;
};

QByteArray composeMessage(int selection, bool latencyMeasurement, int step);

#endif  // __PROTOCOLMESSAGE
