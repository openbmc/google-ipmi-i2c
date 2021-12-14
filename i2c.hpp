#pragma once

#include <ipmid/api.h>

#include <cstdint>
#include <ipmid/api-types.hpp>
#include <ipmid/message.hpp>
#include <span>
#include <vector>

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

using Resp = ipmi::RspType<std::vector<uint8_t>>;

/**
 * I2c is a global i2c-via-ipmi manager and IPMI handler.
 */
class I2c
{
  public:
    /**
     * Register OEM IPMI handler.
     */
    void registerOemRouter();

    /**
     * The i2c-via-ipmi commands go through this method.
     *
     * @param[in] ctx - IPMI Request Context.
     * @param[in] data - Request Data.
     * @return IPMI response.
     */
    Resp transfer(ipmi::Context::ptr ctx, std::span<const uint8_t> data);
};

} // namespace oem
