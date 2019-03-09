/*
 * I2C Bus driver proxy for BMC I2C bus access.
 *
 * For hosts incorporating a BMC, I2C resources are typically connected to
 * the BMC, not the host. This module creates host proxies for the BMC's
 * virtual I2C adapters.
 *
 * At heart, each proxy is a virtual i2c-adapter, and so supports:
 * . raw I2C I/O - as used by i2c-tools, for example
 * . dynamic i2c device creation
 *
 * These proxies and the IPMI extensions underpinning them directly support
 * the I2C_RDWR interface; Linux' SMBUS emulation layer supports SMBUS calls
 * on top of that.
 *
 * Each proxy device connects to exactly one i2c adapter at the BMC,
 * but you can create many proxies if you have many adapters to reach.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": "  fmt

#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/ipmi.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <stdarg.h>

/* TODO(peterh): move to include/uapi/linux/ipmi_msgdefs.h */
#define IPMI_NETFN_OEM_GROUP_REQUEST  0x2e
#define IPMI_NETFN_OEM_GROUP_RESPONSE 0x2f

/* Specific IANA OEM Numbers for this functionality. */
#define IPMI_OEM_GOOG 11129
#define IPMI_OEM_OPENBMC 49871

/* Specific OEM Message for this functionality. */
#define OPENBMC_I2C_OEM_IPMI_CMD 2

/* OEM IPMI Request header consists of 3-byte IANA OEN */
#define OEM_IPMI_REQ_HDR_OEN 0
#define OEM_IPMI_REQ_HDR_LEN 3

/* OEM IPMI Reply header consists of CC and 3 byte IANA OEN */
#define OEM_IPMI_REPLY_HDR_CC 0
#define OEM_IPMI_REPLY_HDR_OEN 1
#define OEM_IPMI_REPLY_HDR_LEN 4

/* I2C OEM IPMI Request header */
#define I2C_OEM_IPMI_REQ_BUS 0
#define I2C_OEM_IPMI_REQ_FLAGS 1
#define I2C_OEM_IPMI_REQ_FLAG_PEC (1 << 7)
#define I2C_OEM_IPMI_REQ_HDR_LEN 2
/* Followed by one or more steps. */

/* I2C OEM IPMI Request step - optional repeated element. */
#define I2C_OEM_IPMI_STEP_DEV_AND_DIR 0
#define I2C_OEM_IPMI_STEP_IS_READ (1 << 0)
#define I2C_OEM_IPMI_STEP_DEV(dandd) ((dandd) >> 1)
#define I2C_OEM_IPMI_STEP_FLAGS 1
#define I2C_OEM_IPMI_STEP_FLAG_RECV_LEN (1 << 7)
#define I2C_OEM_IPMI_STEP_PARM 2
#define I2C_OEM_IPMI_STEP_HDR_LEN 3
/* Followed by parm bytes of wr data if ! IS_READ */

/*
 * I2C OEM IPMI Reply consists of OEM IPMI reply header,
 * followed by every byte read, in requested order.
 */

#define IPMI_MAX_REQ_LEN 60
#define IPMI_MAX_STEP_LEN 34

/* via-ipmi as little-endian hex-ascii */
#define I2C_VIA_IPMI_BUS_MAGIC 0x696d70692d616976

/**
 * struct i2c_via_ipmi_bus - per proxy state.

 * @magic:	Pattern usable to verify void pointer conversion validity.
 * @node:	Embedded list pointers for this structure.
 * @pdev:	Parent i2c-via-ipmi platform device.
 * @remote_bus_id:	Which BMC i2c adapter number to proxy.
 * @adap:	Embedded i2c adapter.
 * @lock:	Spinlock protecting proxy from ioctl vs. reply races.
 * @ipmi_seq:	Nonce to use as msgId for IPMI request/reply; message
 *		handler rejects mismatches as stale replies.
 * @cmd_complete: Completion struct for this proxy.
 * @cmd_rc:	Completion code from IPMI reply.
 * @msgs:	Pointer to array of messages as given to i2c xfer handler.
 * @msgs_count:`Number of msgs[] to process as given to i2c xfer handler.
 * @if_num:	IPMI interface number to use for BMC messaging.
 * @user:	IPMI user for routing requests and replies.
 * @req_addr:	IPMI address to which request is addressed.
 *		(Always local BMC in this version.)
 * @req:	IPMI request structure.
 * @req_data:	Reserved for IPMI request data bytes.
 *
 * This structure holds the state of each BMC i2c proxy adapter during its
 * lifetime.
 *
 * The adap property allows this proxy to act as an i2c adapter, while
 * IPMI user and related addressing information tie the proxy to the
 * IPMI messaging service, and thence the BMC.
 *
 * The msgs and req related properties convery i2c xfer arguments to the
 * reply handler, while the completion properties flow back from the reply
 * handler to the i2c xfer operation that launched the request.
 */
struct i2c_via_ipmi_bus {
/* private: internal use only */
	u64 magic;
	struct list_head node;
	struct platform_device *pdev;
	unsigned remote_bus_id;
	struct i2c_adapter adap;
	spinlock_t lock;
	long ipmi_seq;
	struct completion cmd_complete;
	int cmd_rc;
	struct i2c_msg *msgs;
	size_t msgs_count;
	unsigned if_num;
	ipmi_user_t user;
	struct ipmi_addr req_addr;
	struct kernel_ipmi_msg req;
	unsigned char req_data[256];
};

#define ADAP_NAME_MAX sizeof(((struct i2c_via_ipmi_bus *)0)->adap.name)

static LIST_HEAD(proxies);

static char *proxy_invalid_reason(struct i2c_via_ipmi_bus *bus)
{
	if (!bus)
		return kasprintf(GFP_KERNEL, "is nullptr");
	if (bus->magic != I2C_VIA_IPMI_BUS_MAGIC)
		return kasprintf(GFP_KERNEL, "has bad magic %#llx", bus->magic);
	return NULL;
}

static void i2c_via_ipmi_reply_handler(struct ipmi_recv_msg *reply,
				       void *user_msg_data)
{
	const unsigned char *buf, *oen, *next_byte, *buf_end;
	struct i2c_via_ipmi_bus *bus;
	unsigned char recv_len;
	struct i2c_msg *msg;
	struct device *dev;
	char *reason;
	u32 oem_code;
	int cmd_rc;
	size_t i;
	/*
	 * If message isn't a reply to our ipmi_request_settime,
	 * then user_msg_data is unlikely to point to our i2c_via_ipmi_bus,
	 * so confirm everything we can.
	 */
	bus = user_msg_data;
	reason = proxy_invalid_reason(bus);
	if (reason) {
		pr_err("%s: user_msg_data %s", __func__, reason);
		kfree(reason);
		goto not_our_reply;
	}
	dev = &bus->adap.dev;
	if (!(reply->user &&
	    reply->user_msg_data == user_msg_data &&
	    reply->msg.netfn == IPMI_NETFN_OEM_GROUP_RESPONSE &&
	    reply->msg.cmd == OPENBMC_I2C_OEM_IPMI_CMD &&
	    /* Check response length for error state or valid OEM packet */
	    ((reply->msg.data_len > 0 &&
	       (reply->msg.data[OEM_IPMI_REPLY_HDR_CC] != IPMI_CC_NO_ERROR)) ||
	     (reply->msg.data_len >= OEM_IPMI_REPLY_HDR_LEN)))) {
		dev_info(dev, "%s: apparent non-reply?", __func__);
		goto not_our_reply;
	}
	if (!spin_trylock(&bus->lock)) {
		dev_info(dev, "%s: drop seq=%ld, mutex busy.",
			 __func__, reply->msgid);
		goto not_our_reply;
	}
	if (reply->msgid != bus->ipmi_seq) {
		spin_unlock(&bus->lock);
		dev_info(dev, "%s: drop seq=%ld, want seq=%ld.",
			 __func__, reply->msgid, bus->ipmi_seq);
		goto not_our_reply;
	}
	buf = reply->msg.data;
	buf_end = buf + reply->msg.data_len;

	/*
	 * Extract CC and IANA OEN from OEM IPMI Reply header.
	 */
	cmd_rc = buf[OEM_IPMI_REPLY_HDR_CC];
	/*
	 * If we get an error return, accept the reply and return the error
	 * up the I2C stack.
	 */
	if (cmd_rc != IPMI_CC_NO_ERROR)
		goto accept_reply;

	oen = &buf[OEM_IPMI_REPLY_HDR_OEN];
	oem_code = ((((u32)oen[2] << 8) | oen[1]) << 8) | oen[0];
	next_byte = buf + OEM_IPMI_REPLY_HDR_LEN;
	if (oem_code != IPMI_OEM_OPENBMC) {
		spin_unlock(&bus->lock);
		dev_info(dev, "%s: wrong OEM Enterprise Number %u",
			__func__, oem_code);
		goto not_our_reply;
	}
	/*
	 * Note: having confirmed it's a reply to our request,
	 * any return past here should complete that request.
	 */
	/*
	 * BMC I/O seems to have worked - that's all for writes; if there
	 * are read steps, the rest of the reply caries the bytes read.
	 * Check the whole reply before trusting any of it - if it is
	 * the wrong size for this request, call it an I/O error.
	 */
	for (i = 0; i < bus->msgs_count; ++i) {
		msg = &bus->msgs[i];
		if (!(msg->flags & I2C_M_RD))
			continue;
		if (msg->flags & I2C_M_RECV_LEN) {
			if (next_byte >= buf_end) {
				bus->cmd_rc = -EPROTO;
				spin_unlock(&bus->lock);
				dev_err(dev, "%s[%zu]: response omits RECV_LEN",
					__func__, i);
				goto reject_reply;
			}
			/*
			 * Limit net payload to SMBUS maximum.
			 * To avoid overrun, buffer should be able to hold
			 * union i2c_smbus_data for this kind of step.
			 */
			recv_len = *next_byte;
			if (recv_len > I2C_SMBUS_BLOCK_MAX) {
				bus->cmd_rc = -EPROTO;
				spin_unlock(&bus->lock);
				dev_err(dev, "%s[%zu]: recv_len=%u, max is %d",
					__func__, i, recv_len,
					I2C_SMBUS_BLOCK_MAX);
				goto reject_reply;
			}
			msg->len = recv_len +
					(msg->flags & I2C_CLIENT_PEC ? 2 : 1);
		}
		next_byte += msg->len;
		if (next_byte > buf_end) {
			bus->cmd_rc = -EPROTO;
			spin_unlock(&bus->lock);
			dev_err(dev, "%s[%zu]: response too small",
				__func__, i);
			goto reject_reply;
		}
	}
	if (next_byte < buf_end) {
		bus->cmd_rc = -EPROTO;
		spin_unlock(&bus->lock);
		dev_err(dev, "%s: response len=%d, expected=%zd",
			__func__, reply->msg.data_len, buf_end - next_byte);
		goto reject_reply;
	}
	/*
	 * Everything checks out, copy results back to caller.
	 */
	next_byte = buf + 4;
	for (i = 0; i < bus->msgs_count; ++i) {
		msg = &bus->msgs[i];
		if (msg->flags & I2C_M_RD) {
			memcpy(msg->buf, next_byte, msg->len);
			next_byte += msg->len;
		}
	}

accept_reply:
	bus->cmd_rc = cmd_rc;
	spin_unlock(&bus->lock);

reject_reply:
	complete(&bus->cmd_complete);

not_our_reply:
	reply->done(reply);
	return;
}

static struct ipmi_user_hndl i2c_via_ipmi_ipmi_handlers = {
	.ipmi_recv_hndl = i2c_via_ipmi_reply_handler,
};

static u32 i2c_via_ipmi_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static int i2c_via_ipmi_xfer(struct i2c_adapter *adap,
			     struct i2c_msg *msgs, int num)
{
	unsigned char *next_byte, *oen, *msg_hdr, *step_hdr;
	struct i2c_via_ipmi_bus *bus = adap->algo_data;
	struct device *dev = &adap->dev;
	struct kernel_ipmi_msg *req;
	const struct i2c_msg *msg;
	char *reason;
	int cmd_rc;
	size_t i;
	int rc;

	reason = proxy_invalid_reason(bus);
	if (reason) {
		dev_err(dev, "%s: adap.algo_data %s", __func__, reason);
		kfree(reason);
		return -EFAULT;
	}
	/*
	 * Setup kernel message structure
	 */
	req = &bus->req;
	next_byte = bus->req_data;
	req->data = next_byte;
	/*
	 * Route message to specific OEM command handler.
	 */
	req->netfn = IPMI_NETFN_OEM_GROUP_REQUEST;
	req->cmd = OPENBMC_I2C_OEM_IPMI_CMD;
	/*
	 * IANA OEN is the only content of OEM IPMI Request header.
	 */
	oen = &next_byte[OEM_IPMI_REQ_HDR_OEN];
	oen[0] = (unsigned char)(IPMI_OEM_OPENBMC & 255);
	oen[1] = (unsigned char)((IPMI_OEM_OPENBMC >> 8) & 255);
	oen[2] = (unsigned char)((IPMI_OEM_OPENBMC >> 16) & 255);
	next_byte += OEM_IPMI_REQ_HDR_LEN;
	/*
	 * I2C OEM Message header: bus id & overall flag byte.
	 */
	msg_hdr = next_byte;
	msg_hdr[I2C_OEM_IPMI_REQ_BUS] = bus->remote_bus_id;
	msg_hdr[I2C_OEM_IPMI_REQ_FLAGS] = 0;
	next_byte += I2C_OEM_IPMI_REQ_HDR_LEN;
	/*
	 * Foreach i2c message, append a step.
	 */
	for (i = 0; i < num; ++i) {
		msg = &msgs[i];
		if (msg->len > IPMI_MAX_STEP_LEN) {
			dev_err(dev, "%s[%zu](%#x): len=%d, max=%d",
				__func__, i, msg->addr,
				msg->len, IPMI_MAX_STEP_LEN);
			return -EMSGSIZE;
		}
		/*
		 * Step header - if write step, payload will follow.
		 */
		step_hdr = next_byte;
		next_byte += I2C_OEM_IPMI_STEP_HDR_LEN;
		if (next_byte - msg_hdr > IPMI_MAX_REQ_LEN) {
			dev_err(dev, "%s: request too big.", __func__);
			return -EMSGSIZE;
		}
		step_hdr[I2C_OEM_IPMI_STEP_DEV_AND_DIR] = msg->addr << 1;
		step_hdr[I2C_OEM_IPMI_STEP_FLAGS] = 0;
		step_hdr[I2C_OEM_IPMI_STEP_PARM] = msg->len;
		if (!(msg->flags & I2C_M_RD)) {
			if (msg->flags & I2C_M_RECV_LEN) {
				dev_err(dev, "%s[%zu](%#x): RECV_LEN for WR?",
					__func__, i, msg->addr);
				return -EINVAL;
			}
			if (next_byte + msg->len - msg_hdr > IPMI_MAX_REQ_LEN) {
				dev_err(dev, "%s[%zu](%#x): request too big.",
					__func__, i, msg->addr);
				return -EMSGSIZE;
			}
			if (msg->len) {
				memcpy(next_byte, msg->buf, msg->len);
				next_byte += msg->len;
			}
			dev_dbg(dev, "%s[%zu](%#x): write %d bytes.",
				__func__, i, msg->addr, msg->len);
			continue;
		}
		step_hdr[0] |= 1;  /* Set rd bit */
		if (msg->flags & I2C_M_RECV_LEN) {
			step_hdr[I2C_OEM_IPMI_STEP_FLAGS] |=
				I2C_OEM_IPMI_STEP_FLAG_RECV_LEN;
			if (msg->len == 2)
				msg_hdr[I2C_OEM_IPMI_REQ_FLAGS] |=
					I2C_OEM_IPMI_REQ_FLAG_PEC;
			dev_dbg(dev, "%s[%zu](%#x): read count+%d bytes.",
				__func__, i, msg->addr, msg->len);
			continue;
		}
		dev_dbg(dev, "%s[%zu](%#x): read %d bytes.",
			__func__, i, msg->addr, msg->len);
	}
	req->data_len = next_byte - bus->req_data;
	dev_dbg(dev, "%s: Sending %d step, %d byte request.",
		__func__, num, req->data_len);
	/*
	 * Completion routines need key xfer parameters.
	 */
	bus->msgs = msgs;
	bus->msgs_count = num;

	/*
	 * IPMI handler modifies the same proxy object to return results.
	 * Concurrent proxy access is protected by two properties:
	 * bus->ipmi_seq, a unique sequence number, and
	 * bus->lock, a regular spinlock.
	 * The ipmi_request_settime() call carries ipmi_seq as its msgId;
	 * the IPMI handler checks for exact match to ensure it is a reply
	 * to THIS request. Upon match, it copies answers back & completes
	 * the request. Upon mismatch, proxy state is not modified in any
	 * way, and only lock, impi_seq, and dev are referenced.
	 * This side, the ioctl handler, advances ipmi_seq immediately after
	 * completion or timeout, so any later response will be dropped.
	 * Upon timeout, the cmd_cc state seeded here will remain.
	 * The spinlock is held here while advancing ipmi_seq, and in the
	 * IPMI resonse handler while relying upon the match.
	 */
	bus->cmd_rc = -ETIMEDOUT;
	reinit_completion(&bus->cmd_complete);
	rc = ipmi_request_settime(bus->user, &bus->req_addr, bus->ipmi_seq,
				  req, bus, 0, -1, 0);
	if (rc < 0) {
		dev_err(dev, "%s: ipmi_request_settime returned %d.",
			__func__, rc);
		return rc;
	}
	wait_for_completion_killable_timeout(&bus->cmd_complete,
					     bus->adap.timeout);
	spin_lock(&bus->lock);
	bus->ipmi_seq += 1;
	cmd_rc = bus->cmd_rc;
	spin_unlock(&bus->lock);
	if (!cmd_rc) {
		dev_dbg(dev, "%s: returning num=%d.", __func__, num);
		return num;
	}
	if (cmd_rc == -ETIMEDOUT) {
		dev_err(dev, "%s: returning -ETIMEDOUT.", __func__);
		return cmd_rc;
	}
	if (cmd_rc == -EPROTO) {
		dev_err(dev, "%s: returning -EPROTO.", __func__);
		return -EPROTO;
	}
	if (cmd_rc > 0) {
		dev_dbg(dev, "%s: cmd_rc=%d, so returning -EPROTO.",
			__func__, cmd_rc);
		return -EPROTO;
	}
	dev_dbg(dev, "%s: returning cmd_rc=%d.", __func__, cmd_rc);
	return cmd_rc;
}

static const struct i2c_algorithm i2c_via_ipmi_algo = {
	.functionality	= i2c_via_ipmi_functionality,
	.master_xfer	= i2c_via_ipmi_xfer,
};

static int i2c_via_ipmi_create(struct platform_device *pdev,
			       const char *name,
			       unsigned bmc_bus_id,
			       struct i2c_via_ipmi_bus **new_bus)
{
	struct device *parent = &pdev->dev;
	struct i2c_via_ipmi_bus *bus = NULL;
	int rc;

	bus = devm_kzalloc(parent, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;
	bus->magic = I2C_VIA_IPMI_BUS_MAGIC;
	INIT_LIST_HEAD(&bus->node);
	bus->pdev = pdev;
	bus->remote_bus_id = bmc_bus_id;
	spin_lock_init(&bus->lock);
	bus->ipmi_seq = 0x40000000L;
	init_completion(&bus->cmd_complete);
	bus->req_addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	bus->req_addr.channel = IPMI_BMC_CHANNEL;
	dev_dbg(parent, "%s: name=%s, bmc_bus_id=%u",
		__func__, name, bmc_bus_id);
	rc = ipmi_create_user(bus->if_num, &i2c_via_ipmi_ipmi_handlers,
			      bus, &bus->user);
	if (rc) {
		dev_err(parent, "%s: ipmi_create_user(%d, ...) returned %d.",
			__func__, bus->if_num, rc);
		goto free_instance;
	}

	bus->adap.owner = THIS_MODULE;
	strncpy(bus->adap.name, name, sizeof(bus->adap.name));
	bus->adap.retries = 0;
	/* The BMC's i2c timeout is (and likely will remain) 5s, therefore
	 * this host timeout should be set longer to avoid it timing out
	 * ahead of the BMC's chance.
	 */
	bus->adap.timeout = 10 * HZ;
	bus->adap.algo = &i2c_via_ipmi_algo;
	bus->adap.algo_data = bus;
	if (pdev)
		bus->adap.dev.parent = &pdev->dev;
	rc = i2c_add_adapter(&bus->adap);
	if (rc < 0) {
		dev_err(parent, "%s: i2c_add_adapter(%d, ...) returned %d.",
			__func__, bmc_bus_id, rc);
		goto free_instance;
	}

	list_add_tail(&bus->node, &proxies);
	if (new_bus)
		*new_bus = bus;
	return 0;

free_instance:
	devm_kfree(parent, bus);
	return rc;
}

static int i2c_via_ipmi_destroy(struct i2c_via_ipmi_bus *bus)
{
	struct device *parent = &bus->pdev->dev;

	dev_dbg(parent, "%s: name=%s, bmc_bus_id=%u",
		__func__, bus->adap.name, bus->remote_bus_id);
	i2c_del_adapter(&bus->adap);
	list_del(&bus->node);
	devm_kfree(parent, bus);
	return 0;
}

static int i2c_via_ipmi_probe(struct platform_device *pdev)
{
	char name[ADAP_NAME_MAX];
	unsigned bus_id;
	int rc;

	/*
	 * Create requested population of proxies.
	 */
	for (bus_id = 0; bus_id < CONFIG_I2C_VIA_IPMI_AUTOPROXIES; ++bus_id) {
		snprintf(name, sizeof(name), "bmc%u", bus_id);
		rc = i2c_via_ipmi_create(pdev, name, bus_id, NULL);
		if (rc)
			return rc;
	}
	return 0;
}

static int i2c_via_ipmi_remove(struct platform_device *pdev)
{
	struct i2c_via_ipmi_bus *bus, *safe;
	int pass = 0;

	/*
	 * Destroy all proxies in reverse order.
	 */
	list_for_each_entry_safe_reverse(bus, safe, &proxies, node) {
		i2c_via_ipmi_destroy(bus);
		++pass;
	}
	dev_dbg(&pdev->dev, "%s: removed %d proxies.", __func__, pass);
	return 0;
}

static void i2c_via_ipmi_release(struct device *__unused)
{
	/*
	 * This function releases the resources associated with a device.  Since
	 * our device struct is statically allocated, we don't need to do
	 * anything here.
	 */
}

static struct platform_driver i2c_via_ipmi_driver = {
	.driver = {
		.name		= "i2c-via-ipmi",
	},
	.probe	= i2c_via_ipmi_probe,
	.remove	= i2c_via_ipmi_remove,
};

static struct platform_device i2c_via_ipmi_pdev = {
	.name		= "i2c-via-ipmi",
	.id		= PLATFORM_DEVID_NONE,
	.dev.release	= i2c_via_ipmi_release,
};

int i2c_via_ipmi_init(void)
{
	int rc;

	pr_debug("%s: entering.", __func__);
	rc = platform_driver_register(&i2c_via_ipmi_driver);
	if (rc) {
		pr_err("couldn't register i2c-via-ipmi platform driver\n");
		return rc;
	}
	rc = platform_device_register(&i2c_via_ipmi_pdev);
	if (rc) {
		pr_err("couldn't register i2c-via-ipmi platform device\n");
		goto unregister_pdrv;
	}
	pr_debug("%s: returning 0.", __func__);
	return 0;

unregister_pdrv:
	platform_driver_unregister(&i2c_via_ipmi_driver);
	return rc;
}

void i2c_via_ipmi_exit(void)
{
	pr_debug("%s: entering.", __func__);
	platform_device_unregister(&i2c_via_ipmi_pdev);
	platform_driver_unregister(&i2c_via_ipmi_driver);
	pr_debug("%s: returning.", __func__);
}

subsys_initcall(i2c_via_ipmi_init);
module_exit(i2c_via_ipmi_exit);

MODULE_AUTHOR("Peter Hanson <peterh@google.com>");
MODULE_DESCRIPTION("I2C Bus driver proxy for BMC I2C bus access.");
MODULE_LICENSE("GPL v2");
