#include <assert.h>
#include <array>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <endian.h>
#include <sbe_chipOp_handler.hpp>
#include <file.hpp>
#include <iostream>
namespace openpower
{
namespace sbe
{
namespace internal
{

constexpr uint16_t MAGIC_CODE = 0xC0DE;
constexpr auto SBE_OPERATION_SUCCESSFUL = 0;
constexpr auto LENGTH_OF_DISTANCE_HEADER_IN_WORDS = 0x1;
constexpr auto LENGTH_OF_RESP_HEADER_IN_WORDS = 0x2;
constexpr auto DISTANCE_TO_RESP_CODE = 0x1;
constexpr auto MAX_FFDC_LEN_IN_WORDS = 5120;
constexpr auto WORD_SIZE = 4;
constexpr auto MAGIC_CODE_BITS = 16;
std::vector<sbe_word_t> writeToFifo(const char* devPath,
                                    const sbe_word_t* cmdBuffer,
                                    size_t cmdBufLen,
                                    size_t respBufLen)
{
    std::vector<sbe_word_t> response;
    std::ostringstream errMsg;
    ssize_t len;
    void *pos;

    //Open the device and obtain the file descriptor associated with it.
    FileDescriptor fileFd(devPath, (O_RDWR | O_NONBLOCK));

    printf("Polling to write\n");
    //Wait for FIFO device and perform write operation
    struct pollfd poll_fd = {};
    poll_fd.fd = fileFd();
    poll_fd.events = POLLOUT | POLLERR | POLLHUP | POLLNVAL | POLLRDHUP;

    int rc = 0;
    if ((rc = poll(&poll_fd, 1, -1)) < 0)
    {
        //TODO:use elog infrastructure
        errMsg << "Waiting for FIFO device:" << devPath << "to write failed"
               << "rc=" << rc << "errno=" << errno;
        throw std::runtime_error(errMsg.str().c_str());
    }
    if (poll_fd.revents & POLLERR)
    {
        //TODO:use elog infrastructure
        errMsg << "POLLERR while waiting for writeable FIFO,errno:" << errno;
        throw std::runtime_error(errMsg.str().c_str());
    }
    printf("POLL to write completed, Will attempt to write\n");
    auto bytesToWrite = (cmdBufLen * WORD_SIZE);
    //Perform the write operation
    len = write(fileFd(), cmdBuffer, bytesToWrite);
    if (len < 0)
    {
        //TODO:use elog infrastructure
        errMsg << "Failed to write to FIFO device:" << devPath << " Length "
               "returned= " << len << " errno=" << errno;
        throw std::runtime_error(errMsg.str().c_str());
    }
    printf("Write complete!! will POLL for READ\n");
    //Wait for FIFO device and perform read operation
    poll_fd.fd = fileFd();
    poll_fd.events = POLLIN | POLLERR;
    if ((rc = poll(&poll_fd, 1, -1) < 0))
    {
        //TODO:use elog infrastructure
        errMsg << "Waiting for FIFO device:" << devPath << "to read failed"
               << " rc=" << rc << " and errno=" << errno;
        throw std::runtime_error(errMsg.str().c_str());
    }
    if (poll_fd.revents & POLLERR)
    {
        //TODO:use elog infrastructure
        errMsg << "POLLERR while waiting for readable FIFO,errno:" << errno;
        throw std::runtime_error(errMsg.str().c_str());
    }
    printf("Poll for read completed\n");
    //Create a temporary buffer
    std::vector<sbe_word_t> buffer(respBufLen);
printf("Will attempt to read\n");

    len = 0;
    size_t readSize;
    size_t bytesRead = 0;
    do {
        if (len > 0)
        {
            bytesRead += len;
            // Manually handle the vector expansion as we're writing to the
            // data store in read(), and we need data() to be big enough
            // already.
            buffer.resize(2 * buffer.size());
        }
        pos = &(buffer.data()[bytesRead]);
        readSize = buffer.capacity() * sizeof(sbe_word_t) - bytesRead;
        errno = 0;

    } while ((len = read(fileFd(), pos, readSize)) > 0);

    if (errno != EAGAIN)
    {
        //TODO:use elog infrastructure
        errMsg << "Failed to read the FIFO device:" << devPath << "bytes read ="
            << len << " errno=" << errno;
        throw std::runtime_error(errMsg.str().c_str());
    }

    buffer.resize(bytesRead / sizeof(sbe_word_t));

    //Extract the valid number of words read.
    for (size_t i = 0; i < buffer.size(); i++)
    {
        response.push_back(be32toh(buffer[i]));
    }

    //Closing of the file descriptor will be handled when the FileDescriptor
    //object will go out of scope.
    return response;
}

static const char* primaryStatusDesc[] =
{
    [0x0000] = "Operation successful",
    [0x0001] = "Invalid or unsupported command",
    [0x0002] = "Invalid data passed",
    [0x0003] = "Sequence Error",
    [0x0004] = "SBE Internal Error",
};

static const char* secondaryStatusDesc[] =
{
    [0x0000] = "Operation successful",
    [0x0001] = "Command Class not supported",
    [0x0002] = "Command not supported",
    [0x0003] = "Invalid address passed",
    [0x0004] = "Invalid Target Type passed",
    [0x0005] = "Invalid Chiplet Id passed",
    [0x0006] = "Target not present",
    [0x0007] = "Target is not functional",
    [0x0008] = "Command not allowed in current state",
    [0x0009] = "Functionality not Supported",
    [0x000A] = "Generic failure in execution",
    [0x000B] = "Blacklisted registers or memory accessed",
    [0x000C] = "SBE Operating System Failure",
    [0x000D] = "SBE FIFO Access Failure",
    [0x000E] = "Insufficient data passed as part of command",
    [0x000F] = "Excess data passed as part of command",
    [0x0010] = "Hardware timeout",
    [0x0011] = "PCB-PIB Error",
    [0x0012] = "SBE FIFO Parity Error",
};

static const char* getPrimaryStatusDesc(const uint16_t primary)
{
    if (primary < (sizeof(primaryStatusDesc) / sizeof(*primaryStatusDesc)))
    {
        return primaryStatusDesc[primary];
    }

    assert(primary == 0x00FE);

    return "Unknown Error";
}

static const char* getSecondaryStatusDesc(const uint16_t secondary)
{
    if (secondary < (sizeof(secondaryStatusDesc) / sizeof(*secondaryStatusDesc)))
    {
        return secondaryStatusDesc[secondary];
    }

    return "Undefined error code";
}

void parseResponse(std::vector<sbe_word_t>& sbeDataBuf)
{
    //Number of 32-bit words obtained from the SBE
    size_t lengthObtained = sbeDataBuf.size();

    //Fetch the SBE header and SBE chiop primary and secondary status
    //Last value in the buffer will have the offset for the SBE header
    size_t distanceToStatusHeader = sbeDataBuf[sbeDataBuf.size() - 1];

    if (lengthObtained < distanceToStatusHeader)
    {
        //TODO:use elog infrastructure
        std::ostringstream errMsg;
        errMsg << "Distance to SBE status header value " <<
               distanceToStatusHeader << " is greater then total lenght of "
               "response buffer " << lengthObtained;
        throw std::runtime_error(errMsg.str().c_str());
    }

    //Fetch the response header contents
    auto iter = sbeDataBuf.begin();
    std::advance(iter, (lengthObtained - distanceToStatusHeader));

    //First header word will have 2 bytes of MAGIC CODE followed by
    //Command class and command type
    //|  MAGIC BYTES:0xCODE | COMMAND-CLASS | COMMAND-TYPE|
    sbe_word_t l_magicCode = (*iter >> MAGIC_CODE_BITS);

    //Fetch the primary and secondary response code
    std::advance(iter, DISTANCE_TO_RESP_CODE);
    sbe_word_t l_status = *iter;
    const uint16_t primary = (l_status >> 16) & 0xffff;
    const uint16_t secondary = l_status & 0xffff;

    //Validate the magic code obtained in the response
    if (l_magicCode != MAGIC_CODE)
    {
        //TODO:use elog infrastructure
        std::ostringstream errMsg;
        errMsg << "Invalid MAGIC keyword in the response header (" <<
               l_magicCode << "),expected keyword " << MAGIC_CODE;
        throw std::runtime_error(errMsg.str().c_str());
    }

    //Validate the Primary and Secondary response value
    if (l_status != SBE_OPERATION_SUCCESSFUL)
    {
        //Extract the SBE FFDC and throw it to the caller
        size_t ffdcLen = (distanceToStatusHeader -
                          LENGTH_OF_RESP_HEADER_IN_WORDS -
                          LENGTH_OF_DISTANCE_HEADER_IN_WORDS);
        if (ffdcLen)
        {
            std::vector<sbe_word_t> ffdcData(ffdcLen);
            //Fetch the offset of FFDC data
            auto ffdcOffset = (lengthObtained - distanceToStatusHeader) +
                              LENGTH_OF_RESP_HEADER_IN_WORDS;
            std::copy_n((sbeDataBuf.begin() + ffdcOffset), ffdcLen,
                        ffdcData.begin());
        }

        //TODO:use elog infrastructure to return the SBE and Hardware procedure
        //FFDC container back to the caller.
        std::ostringstream errMsg;
        errMsg << "Chip operation failed with SBE response code "
               << std::hex << primary << ":" << std::hex << secondary
               << ": " << getPrimaryStatusDesc(primary) << ", "
               << getSecondaryStatusDesc(secondary)
               << ". Length of FFDC data of obtained: " << ffdcLen;
        throw std::runtime_error(errMsg.str().c_str());
    }

    //In case of success, remove the response header content and send only the
    //data.Response header will be towards the end of the buffer.
    auto respLen = (lengthObtained - distanceToStatusHeader);
    iter = sbeDataBuf.begin();
    std::advance(iter,respLen);
    sbeDataBuf.erase(iter, sbeDataBuf.end());
}

}
}
}
