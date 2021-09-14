/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include <QtEndian>

#include "protocolmessage.h"
#include "crc.h"

/**
 * Section to determine the endianness of the computer - begin
 *
 *
 *
 */

enum EndianNess
{
    UNDETERMINED,
    LITTLEENDIAN,
    BIGENDIAN
};

static EndianNess endianness = UNDETERMINED;

class DetermineEndianness
{
public:
    DetermineEndianness()
    {
        qint32 i = 0X11223344;
        qint32 j = qToBigEndian(i);
        if (i != j)
            endianness = LITTLEENDIAN;
        else
            endianness = BIGENDIAN;
    }
};

DetermineEndianness determineEndianness;

/**
 * Section to determine the endianness of the computer - end
 *
 */


union CRCConvert
{
    qint32 crc1;
    char crc2[4];
};

/**
 * @brief ProtocolMessage::ProtocolMessage
 * @param check
 */
ProtocolMessage::ProtocolMessage(MessageCheck check)
    : m_parsingState(WAITING_FOR_STX)
    , m_messageCheck(check)
{

}

/**
 * @brief ProtocolMessage::checkMessage
 * @return
 */
bool ProtocolMessage::checkMessage()
{
    // Preliminary checks
    // ------------------
    if (m_messageBuffer[0] != STX)
    {
        qWarning() << Q_FUNC_INFO << "message does not start with STX";
        return false;
    }

    int length = m_messageBuffer.length();
    if (m_messageBuffer[length - 1] != ETX)
    {
        qWarning() << Q_FUNC_INFO << "message does not start with ETX";
        return false;
    }

    int stx_counter = 0;
    for (int i = 1; i < length - 1; i++)
    {
        if (m_messageBuffer[i] == STX)
            stx_counter++;
    }
    if (stx_counter)
    {
        qWarning() << Q_FUNC_INFO << "message contains" << stx_counter << "STX symbols";
        for (int i = 0; i < m_messageBuffer.length(); i++)
            printf("%02X ", (unsigned char) m_messageBuffer[i]);
        printf("\n");
        return false;
    }

    // Remove stuffing
    // ---------------
    QByteArray data2;
    if (!removeStuffing(data2, m_messageBuffer))
    {
        qWarning() << Q_FUNC_INFO << "remove stuffing failed";
        return false;
    }

    // Checksum or CRC check
    // ---------------------
    if (m_messageCheck == USE_CHECKSUM)
    {
        if (!checkChecksum(data2))
        {
            qWarning() << Q_FUNC_INFO << "Check checksum failed";
            return false;
        }
    }
    else if (m_messageCheck == USE_CRC)
    {
        if (!checkCRC(data2))
        {
            qWarning() << Q_FUNC_INFO << "Check CRC failed";
            return false;
        }
    }
    return true;
}

/**
 * @brief ProtocolMessage::createMessage
 * @param data
 */
void ProtocolMessage::createMessage(QByteArray& data)
{
    QByteArray data1;
    if (m_messageCheck == USE_CHECKSUM)
    {
        if (!calculateChecksum(data1, data))
        {
            qWarning() << "Checksum calculation failed";
            return;
        }
    }
    else if (m_messageCheck == USE_CRC)
    {
        if (!calculateCRC(data1, data))
        {
            qWarning() << "CRC calculation failed";
            return;
        }
    }
    else
    {
        data1 = data;
    }

    QByteArray data2;
    if (!addStuffing(data2, data1))
    {
        qWarning() << "add stuffing failed";
        return;
    }

    m_messageBuffer = data2;
}

/**
 * @brief ProtocolMessage::composeMessage
 * @param data
 * @param data2
 * @return
 */
bool ProtocolMessage::composeMessage(QByteArray& data, QByteArray& data2)
{
    int idx = -1;
    bool messageComplete = false;

    for (char b : data)
    {
        idx++;
        if (m_parsingState == WAITING_FOR_STX)
        {
            if (b == STX)
            {
                m_messageBuffer.clear();
                m_messageBuffer.append(STX);
                m_parsingState = WAITING_FOR_ETX;
            }
            else
            {
                //qWarning() << Q_FUNC_INFO << "received bytes while waiting for STX";
            }
        }
        else
        {
            m_messageBuffer.append(b);
            if (b == STX)
            {
                //qWarning() << Q_FUNC_INFO << "received STX while waiting for ETX";
            }
            if (b == ETX)
            {
                m_parsingState = WAITING_FOR_STX;
                messageComplete = true;
                break;
            }
        }
    }

    if (messageComplete)
    {
        // If there are any bytes in data after the ETX,
        // copy them to data2.
        idx++;
#if 0
        if (data[idx] != STX)
        {
            // 0 received instead
            qWarning() << Q_FUNC_INFO << "first byte after ETX != STX" << (unsigned char) data[idx];
        }
#endif
        for (; idx < data.length(); idx++)
        {
            data2.append(data[idx]);
        }
        if (data2.length() > 0)
        {
            qInfo() << "data2 = " << data2;
        }
    }

    return messageComplete;
}

/**
 * @brief ProtocolMessage::calculateChecksum
 * @param dest
 * @param src
 * @return
 */
bool ProtocolMessage::calculateChecksum(QByteArray& dest, QByteArray& src)
{
    if (src[0] != STX || src[src.length() - 1] != ETX)
    {
        return false;
    }

    dest.append(STX);
    char checksum = 0;
    for (int i = 1; i < src.length() - 1; i++)
    {
        char c = src[i];
        dest.append(c);
        checksum += c;
    }
    dest.append(checksum);
    dest.append(ETX);

    return true;
}

/**
 * @brief ProtocolMessage::checkChecksum
 * @param src
 * @return
 */
bool ProtocolMessage::checkChecksum(QByteArray& src)
{
    if (src[0] != STX || src[src.length() - 1] != ETX)
    {
        return false;
    }

    qint16 checksum = 0;
    for (int i = 1; i < src.length() - 2; i++)
    {
        checksum += src[i];
    }

    int idx = src.length() - 2;
    if (idx > 0)
    {
        char transmittedChecksum = src[idx];
        if (checksum != transmittedChecksum)
        {
            qWarning() << Q_FUNC_INFO << "Calculated checksum " << checksum
                       << " differs from transmitted checksum " << transmittedChecksum;
            return false;
        }
    }
    else
        return false;

    return true;
}

/**
 * @brief ProtocolMessage::calculateCRC
 *
 * The transmitter of the packet starts with the CRC seed 0xffffffff - the
 * transmitter calculates the CRC on each byte starting after the packet's
 * initial 0xff, up to and including the last character before the terminating
 * 0xfe -the CRC is calculated on bytes before byte stuffing of 0xfd, 0xfe,
 * and 0xff is performed - after the CRC has been calculated, the remainder
 * is bitwise inverted - the remainder is then appended to the end of the
 * packet, before the 0xfe, with the least significant byte first.
 *
 * @param dest
 * @param src
 * @return
 */
bool ProtocolMessage::calculateCRC(QByteArray& dest, QByteArray& src)
{
    if (src[0] != STX || src[src.length() - 1] != ETX)
    {
        return false;
    }

    for (int i = 0; i < src.length() - 1; i++)
    {
        dest.append(src[i]);
    }

    char* data = src.data();
    unsigned long buffersize = src.length() - 2;
    unsigned long crc = CalculateCRC32((unsigned char*)(data+1), buffersize);

    // Determine how to write to memory:
    // "the remainder is then appended to the end of the packet, before the 0xfe, with the least significant byte first"

    CRCConvert CRCConvert_;
    CRCConvert_.crc1 = crc;

    if (endianness == LITTLEENDIAN)
    {
        for (int i = 0; i < 4; i++)
        {
            dest.append(CRCConvert_.crc2[i]);
        }
    }
    else
    {
        for (int i = 3; i >=0 ; i--)
        {
            dest.append(CRCConvert_.crc2[i]);
        }
    }

    dest.append(ETX);

    return true;
}

/**
 * @brief ProtocolMessage::checkCRC
 *
 * The receiver starts with a seed of 0xffffffff and calculates the CRC over
 * each byte after the 0xff and before the 0xfe, including the CRC remainder
 * that the transmitter appended before the 0xfe - the CRC is calculated on
 * bytes after byte stuffing of 0xfd, 0xfe, and 0xff is reversed -the remainder
 * is compared with 0xdebb20e3; if they match, then the CRC has been
 * verified.
 *
 * @param src
 * @return
 */
bool ProtocolMessage::checkCRC(QByteArray& src)
{
    if (src[0] != STX || src[src.length() - 1] != ETX)
    {
        return false;
    }

    char* data = src.data();
    unsigned long buffersize = src.length() - 2;
    unsigned long crc = CalculateCRC32((unsigned char*)(data+1), buffersize);
    if (crc != 0x2144df1c)
    {
        qWarning() << Q_FUNC_INFO << "Calculated CRC " << crc << " differs from expected CRC " << 0x2144df1c;
        for (int i = 0; i < src.length(); i++)
            printf("%02X ", (unsigned char) data[i]);
        printf("\n");
        return false;
    }
    return true;
}

/**
 * @brief ProtocolMessage::addStuffing
 * @param dest
 * @param src
 * @return
 */
bool ProtocolMessage::addStuffing(QByteArray& dest, QByteArray& src)
{
    if (src[0] != STX || src[src.length() - 1] != ETX)
    {
        return false;
    }

    dest.append(STX);
    for (int i = 1; i < src.length() - 1; i++)
    {
        char c = src[i];
        if (c == STX)
        {
            dest.append(DEL);
            dest.append(STX_STUFF);
        }
        else if (c == ETX)
        {
            dest.append(DEL);
            dest.append(ETX_STUFF);
        }
        else if (c == DEL)
        {
            dest.append(DEL);
            dest.append(DEL_STUFF);
        }
        else
        {
            dest.append(c);
        }
    }
    dest.append(ETX);

    return true;
}

/**
 * @brief ProtocolMessage::removeStuffing
 * @param dest
 * @param src
 * @return
 */
bool ProtocolMessage::removeStuffing(QByteArray& dest, QByteArray& src)
{
    if (src[0] != STX || src[src.length() - 1] != ETX)
    {
        return false;
    }

    bool delEncountered = false;
    dest.append(STX);
    for (int i = 1; i < src.length() - 1; i++)
    {
        char c = src[i];
        if (c == DEL)
        {
            delEncountered = true;
        }
        else if (delEncountered)
        {
            delEncountered = false;
            if (c == STX_STUFF)
            {
                dest.append(STX);
            }
            else if (c == ETX_STUFF)
            {
                dest.append(ETX);
            }
            else if (c == DEL_STUFF)
            {
                dest.append(DEL);
            }
            else
            {
                qWarning() << "Encountered invalid byte " << c << " after " << DEL;
                return false;
            }
        }
        else
        {
            dest.append(c);
        }
    }
    dest.append(ETX);

    return true;
}


/**
 * @brief composeMessage
 * @param latencyMeasurement
 * @param selection
 * @return
 */
QByteArray composeMessage(int selection, int step)
{
    //qDebug() << Q_FUNC_INFO << selection << latencyMeasurement << step;

    QByteArray data;
    data.append(STX);

    for (int i = 0x10; i < 0x14; i++)
    {
        data.append(static_cast<unsigned char>(i));
    }

    for (int i = 0; i < step * selection; i++)
    {
        data.append(static_cast<unsigned char>(0x20 + i));
    }

    data.append(ETX);

    return data;
}
