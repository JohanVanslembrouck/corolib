/**
 * @file protocolmessage.h
 * @brief
 * Contains the definition of a ProtocolMessage that can be sent over a serial line connection.
 * At the wire level, the byte stream of a ProtocolMessage starts with a STX symbol
 * and ends with an ETX symbol. The ETX symbol is optionally preceded by a CRC or checksum.
 * The DEL (delimiter) is used to delimit (indicate) occurrences of STX, ETX or itself
 * in the byte stream.
 *
 * This class does not use coroutine functionality.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef __PROTOCOLMESSAGE
#define __PROTOCOLMESSAGE

#include <QObject>
#include <QDebug>
#include <QByteArray>

const char STX = static_cast<char>(0xff);		// STX = Start of Text
const char ETX = static_cast<char>(0xfe);		// ETX = End of Text
const char DEL = static_cast<char>(0xfd);		// DEL = Delimiter

const char STX_STUFF = static_cast<char>(0x02);
const char ETX_STUFF = static_cast<char>(0x01);
const char DEL_STUFF = static_cast<char>(0x00);

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

/**
 * @class ProtocolMessage
 * @brief
 *
 */
class ProtocolMessage
{
public:
    ProtocolMessage(MessageCheck check = NO_CHECK);

    bool checkMessage();
    bool composeMessage(QByteArray& data, QByteArray& data2);
    bool composeMessage(const char* data, int length, int& index);
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

QByteArray composeMessage(int selection, int step, int repetition = 1);

#endif  // __PROTOCOLMESSAGE
