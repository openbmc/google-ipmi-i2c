#pragma once

#include <ipmid/api.h>

#include <cstdint>
#include <ipmid/oemrouter.hpp>

using std::uint8_t;

namespace oem
{
namespace i2c
{
/*
 * Request header
 */
constexpr size_t requestHeaderBus = 0;
constexpr size_t requestHeaderFlags = 1;
constexpr size_t requestHeaderLen = 2;

typedef uint8_t BusId;
typedef uint8_t ReqFlags;

constexpr ReqFlags requestFlagsUsePec = (1 << 7);

/*
 * Request step.
 */
constexpr size_t stepHeaderDevAndDir = 0;
constexpr size_t stepHeaderFlags = 1;
constexpr size_t stepHeaderParm = 2;
constexpr size_t stepHeaderLen = 3;

typedef uint8_t DevAddr;
typedef uint8_t StepFlags;
constexpr StepFlags stepFlagsRecvLen = (1 << 7);
constexpr StepFlags stepFlagsNoStart = (1 << 6);

// So far 2 steps suffics, so 4 should be safe.
constexpr size_t maxSteps = 4;

// Currently we specify 32 byte payload limit;
// but for block read with PEC that entails 34 total bytes.
constexpr size_t largestReply = 34;

} // namespace i2c

/**
 * I2c is a global i2c-via-ipmi manager and IPMI handler.
 */
class I2c
{
  public:
    /**
     * Allows specification of the mechanism to register OEM IPMI handler.
     *
     * @param[in] oemRouter - A pointer to a router instance.
     */
    void registerWith(Router* oemRouter);

    /**
     * The i2c-via-ipmi commands go through this method.
     *
     * @param[in] cmd - the IPMI command.
     * @param[in] reqBuf - the IPMI command buffer.
     * @param[in,out] replyBuf - the IPMI response buffer.
     * @param[in,out] dataLen - pointer to request length, set to reply length.
     * @return IPMI return code.
     */
    ipmi_ret_t transfer(ipmi_cmd_t cmd, const uint8_t* reqBuf,
                        uint8_t* replyBuf, size_t* dataLen);
};

} // namespace oem
