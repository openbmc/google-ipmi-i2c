# I2C via IPMI Proxy Adapter

Authors: peterh@google.com

Last Updated: 2017-05-16

## Objective

Provide a mechanism so that the host can manipulate I2C and SMBus devices on the
associated BMC.

## Background

Recent server systems come with a secondary processing system attached for the
purpose of monitoring and control, generally referred to as a BMC. There is a
large effort to develop an open source framework for writing applications and
control systems that will run on the BMC, known as OpenBMC. Within Google the
effort has been internalized (while also providing upstream pushes) as gBMC. The
primary goal of OpenBMC is to support remote and local system management through
several interfaces, IPMI being the one of interest here.

Historically, a host tool provides a kind of adaptation layer, enabling many
system-level functions for Google Servers. Important functions presume direct
I2C access, but with a BMC, the server is wired such that only the BMC has
direct access and control over many motherboard components, including
substantially all I2C adapters. To resolve this inconsistency, OpenBMC will be
extended to expose the needed I2C adapters for use by host tools.

IPMI allows for OEM commands to provide custom information flow or system
control with a BMC. And OpenBmc now has its own IANA OEM Enterprise Number,
which may be used when upstreaming such controls for general use.

## Overview

The BMC IPMI daemon supports the addition of command handlers. So we will add an
OEM Extension command for I2C transfer which will perform an I2C transfer and
return the results.

With this piece, in theory a host tool's I2C I/O calls could be converted to the
equivalent IPMI calls. However, in some cases, the host tool would be advised to
instantiate an I2C device and use a higher level API. As a simple example, we
should probably use the associated Linux driver, rather than raw I2C I/O, when
reprogramming an at24c64d.

So we propose to create a proxy I2C adapter for each I2C bus we need to reach on
the BMD. This avoids the need to convert raw host I2C, and enables regular Linux
I2C device drivers at the host to connect to actual devices on the BMC.

## Detailed Design

### IPMID OEM Command Extension Routing

*   Register wildcard command handler for any OEM Group Extension request (netfn
    0x2e, cmd 0xff)
*   Provide for registration of OEN, command pairs
*   Route received requests to registered handler

### IPMID Messages

#### Request

```
NetFn    0x2e
OemGroup 49871
Cmd      2 (openBmcI2cOemCmd)
```

##### Request Data

byte(s) | type     | ID       | Description
------- | -------- | -------- | ----------------------------------------
0       | byte     | bus      | i2c adapater number at BMC
1       | ReqFlags | reqFlags | Flags for request
        |          |          | bit 7 - PEC flag for M_RECV_LEN transfer
        |          |          | bits 6:0 - zero -reserved for future use
2+      | Step     | -        | One per struct i2c_msg in transfer.

##### Each Request Step

| byte(s) | type      | ID        | Description                                |
| ------- | --------- | --------- | ------------------------------------------ |
| 0       | byte      | devAndDir | bit 0: 1 if step is read, 0 if write       |
|         |           |           | bits 7:1 i2c device address                |
| 1       | StepFlags | stepFlags | Flags fro each step                        |
|         |           |           | bit 7: carries M_RECV_LEN bit - read steps |
:         :           :           : only                                       :
|         |           |           | bit 6: reserved for M_NOSTART              |
| 2       | byte      | parm      | Transfer length (except M_RECV_LEN)        |
| 3-p+2   | byte      | wr_data   | Write steps only: parm bytes of data to    |
:         :           :           : write.                                     :

#### Response

```
NetFn    0x2e
OemGroup 49871
Cmd      2 (openBmcI2cOemCmd)
cc       Completion code
Data     Only if successful: all bytes read, in order received.
```

#### Message Example

`i2c-1 0x50 FRU EEPROM`

```
Content, trailing zero bytes omitted.
# i2cdump -f -y 1 0x50 b
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef
00: 01 00 00 01 00 00 00 fe 01 0b 19 83 6a 99 c6 51    ?..?...?????j??Q
10: 75 61 6e 74 61 d7 4d 65 6d 6f 72 79 20 52 69 73    uanta?Memory Ris
20: 65 72 20 44 44 52 34 20 42 6f 61 72 64 cf 51 54    er DDR4 Board?QT
30: 46 34 4b 31 31 35 30 37 30 30 32 33 38 cb 33 37    F4K1150700238?37
40: 53 34 4c 52 42 30 30 32 30 c9 46 52 55 20 76 30    S4LRB0020?FRU v0
50: 2e 30 31 c3 41 33 47 01 04 c1 00 00 00 00 00 99    .01?A3G???.....?

Read `Quanta' from FRU at bus=1/addr=0x50/offset=15 at BMC.
# ipmitool -I dbus raw 0x2e 2 0x79 0x2b 0x00 1 0 0xa0 0 1 15 0xa1 0 6
79 2b 00 51 75 61 6e 74 61

Read `Quanta' from FRU at bus=1/addr=0x50/offset=15 at host
# ipmitool raw 0x2e 2 0x79 0x2b 0x00 1 0 0xa0 0 1 15 0xa1 0 6
79 2b 00 51 75 61 6e 74 61

Key to bytes
netfn 0x2e for OEM extension
cmd 2 for I2C via IPMI
oem iana number (0x79 0x2b 0x00 for OpenBmc; Google OEN also works)
req hdr (bmc bus, request flags)
write step bytes
 * devAndDir 0xa0 => device addr 0x50, dir = write
 * flags 0
 * parm 1 => send 1 data byte (data bytes follow step header)
 * data byte 15 selects register 15
 * read step bytes (devAndDir, flags, parm=6 => read 6 bytes)
```

Hopefully you can see how to modify these commands in simple ways:

*   Use a different BMC bus or device address.
*   Change the data byte to select a different starting register, other than 15.
*   Change the read step length to read more/fewer bytes, but note max is 32.

More advanced ideas

*   Skip the read step & add more data bytes to write to selected address.
*   Zero length read / write step do quick write 1 / 0, respectively

### IPMID OpenBMC I2C Command Extension

*   Register handler for OpenBmc I2c Oem Extension requests
*   Handle I2C transfer request
    *   Decode message information
    *   Convert request steps to struct i2c_msg array entries
        *   Adjust M_RCV_LEN steps for IOCTL
    *   Perform I2C_RDWR ioctl on associated I2C bus
    *   Return nonzero completion code upon errors; or
    *   Append all bytes read into one reply upon success

### Prodkernel i2c-via-ipmi Proxy Platform Driver

#### Platform i2c-via-ipmi

Platform device attributes at /sys/devices/platform/i2c-via-ipmi/${name}

*   name - identifies proxy (BMC's i2c bus number)

#### Proxy Operation

##### I2C Adapter Instance

*   Host tool performs I/O on this interface
*   Implements master_xfer method

##### IPMI User Instance

*   Supports IPMI messaging
*   Implements ipmi_recv_hndl callback

##### I2C master_xfer logic

*   Convert struct i2c_msg array to IPMI request
*   Send request & wait for completion

##### IPMI ipmi_recv_hndl logic

*   Check everything possible to ensure this is a reply to our request
*   Copy received bytes to user buffer(s)
*   Declare completion

