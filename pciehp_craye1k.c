// Copyright 2022 Lawrence Livermore National Security, LLC
//
// SPDX-License-Identifier: GPL-2.0
/*
 * Cray ClusterStor E1000 hotplug slot LED driver
 *
 * This driver provides custom set_attention_status/get_attention_status
 * callbacks to control the NVMe slot LEDs.  Unlike normal hotplug slots,
 * the Cray E1000 slot LEDs are set with IPMI calls.
 *
 * This driver is based off of ibmpex.c
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ipmi.h>
#include <linux/ipmi_smi.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/dmi.h>
#include <linux/pci.h>
#include <linux/pci_hotplug.h>
#include "pciehp.h"

/* Cray E1000 commands */
#define CRAYE1K_CMD_NETFN       0x3c
#define CRAYE1K_CMD_PRIMARY     0x33
#define CRAYE1K_CMD_FAULT_LED   0x39
#define CRAYE1K_CMD_LOCATE_LED  0x22

/* Subcommands */
#define CRAYE1K_GET_LED		0x0
#define CRAYE1K_SET_LED		0x1
#define CRAYE1K_SET_PRIMARY	0x1

static struct craye1k {
	struct device *dev;   /* BMC device */
	struct mutex lock;
	struct completion read_complete;
	struct ipmi_addr address;
	ipmi_user_t user;
	int iface;

	long tx_msg_id;
	struct kernel_ipmi_msg tx_msg;
	unsigned char tx_msg_data[IPMI_MAX_MSG_LENGTH];

	unsigned char rx_msg_data[IPMI_MAX_MSG_LENGTH];
	unsigned long rx_msg_len;
	unsigned char rx_result;	/* IPMI completion code */

	/*
	 * Record the original set_attention_status()/get_attention_status()
	 * callbacks for the 24 ports so that we can restore them when we
	 * remove the module.
	 */
	const struct hotplug_slot_ops *orig_ops[24];
	struct hotplug_slot_ops new_ops[24];
} *craye1k = NULL;

/*
 * craye1k_msg_handler() - IPMI message response handler
 */
static void craye1k_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data)
{
	if (msg->msgid != craye1k->tx_msg_id) {
		dev_warn(craye1k->dev,
			"Received msgid %d doesn't match expected msgid %d\n",
			(int)msg->msgid,
			(int)craye1k->tx_msg_id);
		craye1k->rx_result = IPMI_UNKNOWN_ERR_COMPLETION_CODE;
		goto out;
	}

	/* Set rx_result to the IPMI completion code */
	if (msg->msg.data_len > 0)
		craye1k->rx_result = msg->msg.data[0];
	else
		craye1k->rx_result = IPMI_UNKNOWN_ERR_COMPLETION_CODE;

	if (msg->msg.data_len > 1) {
		/* Exclude completion code from data bytes */
		craye1k->rx_msg_len = msg->msg.data_len - 1;
		memcpy(craye1k->rx_msg_data, msg->msg.data + 1,
			craye1k->rx_msg_len);
	} else
		craye1k->rx_msg_len = 0;

out:
	ipmi_free_recv_msg(msg);
	complete(&craye1k->read_complete);
}

/*
 * craye1k_send_message() - Send the message already setup in 'craye1k'
 *
 * Context: craye1k->lock is already held.
 * Return: 0 on success, non-zero on error.
 */
static int craye1k_send_message(void)
{
	int rc;

	rc = ipmi_validate_addr(&craye1k->address, sizeof(craye1k->address));
	if (rc) {
		dev_err(craye1k->dev, "validate_addr() = %d\n", rc);
		return rc;
	}

	craye1k->tx_msg_id++;

	rc = ipmi_request_settime(craye1k->user, &craye1k->address,
		craye1k->tx_msg_id, &craye1k->tx_msg, craye1k, 0, -1, 0);
	if (rc) {
		dev_err(craye1k->dev, "ipmi_request_settime() = %d\n", rc);
		return rc;
	}

	return 0;
}

/*
 * craye1k_do_message() - Send the message in 'craye1k' and wait for a response
 *
 * Return: 0 on success, non-zero on error.
 */
static int craye1k_do_message(void)
{
	int rc;

	if (mutex_lock_interruptible(&craye1k->lock) != 0)
		return -1;

	rc = craye1k_send_message();
	if (rc == 0) {
		wait_for_completion_killable_timeout(&craye1k->read_complete,
			msecs_to_jiffies(500));
	}

	if (craye1k->rx_result == IPMI_UNKNOWN_ERR_COMPLETION_CODE)
		rc = -1;

	mutex_unlock(&craye1k->lock);

	return rc;
}

/*
 * __craye1k_do_command() - Do an IPMI command
 *
 * Send a command with optional data bytes, and read back response bytes.
 *
 * Returns: 0 on success, non-zero on error.
 */
static int __craye1k_do_command(u8 netfn, u8 cmd, u8 *send_data,
	u8 send_data_len, u8 *recv_data, u8 recv_data_len)
{
	int rc;

	craye1k->tx_msg.netfn = netfn;
	craye1k->tx_msg.cmd = cmd;

	if (send_data) {
		memcpy(&craye1k->tx_msg_data[0], send_data, send_data_len);
		craye1k->tx_msg.data_len = send_data_len;
	} else {
		craye1k->tx_msg_data[0] = 0;
		craye1k->tx_msg.data_len = 0;
	}

	rc = craye1k_do_message();
	memcpy(recv_data, craye1k->rx_msg_data, recv_data_len);

	return rc;
}

/*
 * craye1k_do_command_and_netfn() - Do IPMI command and return 1st data byte
 *
 * Do an IPMI command with the given netfn, cmd, and optional send payload
 * bytes.
 *
 * Returns: the last byte from the response or 0 if response had no response
 * data bytes, else -1 on error.
 */
static int craye1k_do_command_and_netfn(u8 netfn, u8 cmd, u8 *send_data,
	u8 send_data_len)
{
	int rc;

	if (!craye1k) {
		/* sanity check - this should never happen */
		dev_err(craye1k->dev, "BMC disappeared");
		return -1;
	}

	rc = __craye1k_do_command(netfn, cmd, send_data, send_data_len, NULL,
		0);

	if (rc != 0) {
		dev_err(craye1k->dev, "Error attempting command (0x%x, 0x%x)\n",
			netfn, cmd);
		return -1;
	}

	if (craye1k->tx_msg.data_len == 0)
		return 0;

	/* Return last received byte value */
	return craye1k->rx_msg_data[craye1k->rx_msg_len - 1];
}

/*
 * craye1k_do_command() - Do a Cray E1000 specific IPMI command.
 * @cmd: Cray E1000 specific command
 * @send_data:  Data to send after the command
 * @send_data_len: Data length
 *
 * Returns: the last byte from the response or 0 if response had no response
 * data bytes, else -1 on error.
 */
static int craye1k_do_command(u8 cmd, u8 *send_data, u8 send_data_len)
{
	return craye1k_do_command_and_netfn(CRAYE1K_CMD_NETFN, cmd, send_data,
		send_data_len);
}

/*
 * __craye1k_set_primary() - Tell the BMC we want to be the primary server
 *
 * An E1000 board has two physical servers on it.  In order to set a slot
 * NVMe LED, this server needs to first tell the BMC that it's the primary
 * server.
 *
 * Returns: 0 on success, 1 otherwise.
 */

static int __craye1k_set_primary(void)
{
	u8 bytes[2] = {CRAYE1K_SET_PRIMARY, 1};	/* set primary to 1 */
	int rc;

	rc = craye1k_do_command(CRAYE1K_CMD_PRIMARY, bytes, 2);
	if (rc == -1)
		return 1;

	return 0;
}

/*
 * craye1k_is_primary() - Are we the primary server?
 *
 * Returns: 1 if we are the primary server, 0 otherwise.
 */
static int craye1k_is_primary(void)
{
	u8 byte = 0;
	int rc;

	/* Response byte is 0x1 on success */
	rc = craye1k_do_command(CRAYE1K_CMD_PRIMARY, &byte, 1);
	if (rc == 0x1)
		return 1;   /* success */

	return 0;   /* We are not the primary server node */
}

/*
 * craye1k_set_primary() - Attempt to set ourselves as the primary server
 *
 * Returns: 0 on success, 1 otherwise.
 */
static int craye1k_set_primary(void)
{
	int tries = 10;

	if (craye1k_is_primary())
		return 0;

	if (__craye1k_set_primary() != 0)
		return 1;	/* error */

	/*
	 * It can take 2 to 3 seconds after setting primary for the controller
	 * to report that it is the primary.
	 */
	while (tries--) {
		msleep(500);
		if (craye1k_is_primary())
			break;
	}

	if (tries == 0)
		return 1;	/* never reported that it's primary */

	/*
	 * It has been observed that there is a 500ms-1s period after setting
	 * primary where the "set LED" commands will not work correctly.  More
	 * specifically, you can set an LED, and the expected LED value will be
	 * correctly returned on a query, but then 500ms later, the LED will
	 * report an incorrect value, and the LED will not have been set.
	 *
	 * To get around this, just explicitly wait 1.5 seconds after setting
	 * primary before you can do commands.
	 */
	msleep(1500);

	return 0;
}

/*
 * craye1k_get_slot_led() - Get slot LED value
 * @slot: Slot number (1-24)
 * @is_locate_led: 0 = get fault LED value, 1 = get locate LED value
 *
 * Returns: slot value on success, -1 on failure.
 */
static int craye1k_get_slot_led(unsigned char slot, unsigned char is_locate_led)
{
	u8 bytes[2];
	u8 cmd;

	bytes[0] = CRAYE1K_GET_LED;
	bytes[1] = slot;

	cmd = is_locate_led ? CRAYE1K_CMD_LOCATE_LED : CRAYE1K_CMD_FAULT_LED;

	return craye1k_do_command(cmd, bytes, 2);
}

/*
 * craye1k_set_slot_led() - Attempt to set the locate/fault LED to a value
 * @slot: Slot number (1-24)
 * @is_locate_led: 0 = use fault LED, 1 = use locate LED
 * @value: Value to set (0 or 1)
 *
 * NOTE: The LED may not be set after calling this function!  The E1000
 * controller seems to be a little flaky and doesn't always set the LED. This
 * is accounted for in craye1k_set_attention_status() since it checks the
 * LED value after setting it to ensure it's correct.  If it's not correct, it
 * will retry setting the LED up to two more times.
 *
 * Returns: 0 on success, 1 on failure.
 */
static int craye1k_set_slot_led(unsigned char slot, unsigned char is_locate_led,
	unsigned char value)
{
	int rc;
	u8 bytes[3];


	bytes[0] = CRAYE1K_SET_LED;
	bytes[1] = slot;
	bytes[2] = value;

	if (is_locate_led)
		rc = craye1k_do_command(CRAYE1K_CMD_LOCATE_LED, bytes, 3);
	else
		rc = craye1k_do_command(CRAYE1K_CMD_FAULT_LED, bytes, 3);

	if (rc == -1) {
		/* Error setting LED - let higher level retries deal with it */
		return 1;
	}

	return 0;
}

static struct ipmi_user_hndl craye1k_msg_handlers = {
	.ipmi_recv_hndl = craye1k_msg_handler
};

int craye1k_get_attention_status(struct hotplug_slot *hotplug_slot,
	u8 *status)
{
	unsigned char slot;
	int locate, fault;

	slot = PSN(to_ctrl(hotplug_slot));
	if (!(slot >= 1 && slot <= 24))
		return -EINVAL;

	if (craye1k_set_primary() != 0)
		return -EIO;

	locate = craye1k_get_slot_led(slot, 1);
	if (locate == -1)
		return -EINVAL;

	fault = craye1k_get_slot_led(slot, 0);
	if (fault == -1)
		return -EINVAL;

	*status = locate << 1 | fault;
	return 0;
}

int craye1k_set_attention_status(struct hotplug_slot *hotplug_slot, u8 status)
{
	unsigned char slot;
	int tries = 3;
	int rc;
	u8 new_status;

	slot = PSN(to_ctrl(hotplug_slot));
	if (!(slot >= 1 && slot <= 24))
		return -EINVAL;

	/*
	 * The retry logic is here since the controller doesn't always set the
	 * LED on the first try.
	 */
	while (tries--) {
		/*
		 * The node must first set itself to be the primary node before
		 * setting the slot LEDs (each board has two nodes, or
		 * "servers" as they're called  by the manufacturer).  This can
		 * lead to contention if both nodes are trying to set the LEDs
		 * at the same time.
		 */
		rc = craye1k_set_primary();
		if (rc != 0) {
			/* Could not set as primary node.  Just retry again. */
			continue;
		}

		/* locate */
		if (craye1k_set_slot_led(slot, 1, (status & 0x2) >> 1) != 0)
			continue;	/* fail, retry */

		/* fault */
		if (craye1k_set_slot_led(slot, 0, status & 0x1) != 0)
			continue;	/* fail, retry */

		rc = craye1k_get_attention_status(hotplug_slot, &new_status);
		if ((rc == 0) && (new_status == status))
			break;	/* success */

		/*
		 * At this point we weren't successful in setting the LED and
		 * need to try again.
		 *
		 * Do a random 0-999ms back-off to reduce contention with other
		 * server node in the unlikely case that both server nodes are
		 * trying to set a LED at the same time.
		 */
		msleep(get_random_int() % 1000);
	}

	if (tries == 0)
		return -EIO;

	return 0;
}

/*
 * Returns the hotplug controller for a given pci_dev (if any).
 */
struct controller *craye1k_pci_dev_to_ctrl(struct pci_dev *dev)
{
	struct device *device;
	struct pcie_device *edev;
	struct controller *ctrl;

	device = pcie_port_find_device(dev, PCIE_PORT_SERVICE_HP);
	if (!device)
		return NULL;

	edev = to_pcie_device(device);
	if (!edev)
		return NULL;

	ctrl = get_service_data(edev);
	if (!ctrl)
		return NULL;

	return ctrl;
}

/*
 * Update the hotplug 'attention' callbacks to point to craye1k's callbacks.
 */
void craye1k_setup_attention_callbacks(void)
{
	struct pci_dev *dev = NULL;
	const struct hotplug_slot_ops *ops;
	struct hotplug_slot_ops *new_ops;
	struct controller *ctrl;
	unsigned char slot;

	/*
	 * Iterate though all the PCI devices looking for the ones controlled
	 * by the pciehp driver.
	 */
	for_each_pci_dev(dev) {
		ctrl = craye1k_pci_dev_to_ctrl(dev);
		if (!ctrl)	/* not controlled by pciehp */
			continue;

		slot = PSN(ctrl);
		if (!(slot >= 1 && slot <= 24))
			continue;

		/*
		 * Save old hotplug ops callbacks for restoration when
		 * we unload the driver.
		 */
		ops = ctrl->hotplug_slot.ops;
		craye1k->orig_ops[slot - 1] = ops;

		/*
		 * 'ops' is const, so we can't just go in and change
		 * ctrl->hotplug_slot.ops.[get|set]_attention_status to
		 * point to our callbacks.  Instead we make a copy of ops,
		 * update our callbacks in it, and point ctrl->hotplug_slot.ops
		 * to our new 'ops'.
		 */
		new_ops = &craye1k->new_ops[slot - 1];
		memcpy(new_ops, ops, sizeof(*ops));
		new_ops->set_attention_status = craye1k_set_attention_status;
		new_ops->get_attention_status = craye1k_get_attention_status;

		/*
		 * Re-generate sysfs entry for our ops.  In this case, it will
		 * add our 'attention' sysfs entry for slots 1-24.
		 */
		pci_hp_del(&ctrl->hotplug_slot);
		ctrl->hotplug_slot.ops = new_ops;
		pci_hp_add(&ctrl->hotplug_slot);
	}
}

void craye1k_restore_attention_callbacks(void)
{
	struct pci_dev *dev = NULL;
	unsigned char slot;
	struct controller *ctrl;

	for_each_pci_dev(dev) {
		ctrl = craye1k_pci_dev_to_ctrl(dev);
		if (!ctrl)
			continue;

		slot = PSN(ctrl);
		if (!(slot >= 1 && slot <= 24))
			continue;

		pci_hp_del(&ctrl->hotplug_slot);
		ctrl->hotplug_slot.ops = craye1k->orig_ops[slot - 1];
		pci_hp_add(&ctrl->hotplug_slot);

	}
}

static void craye1k_new_smi(int iface, struct device *dev)
{
	int rc;

	if (craye1k) {
		/* This shouldn't happen */
		dev_err(dev, "Already initialized");
		return;
	}

	craye1k = kzalloc(sizeof(*craye1k), GFP_KERNEL);
	if (!craye1k)
		return;

	craye1k->address.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	craye1k->address.channel = IPMI_BMC_CHANNEL;
	craye1k->iface = iface;
	craye1k->dev = dev;
	craye1k->tx_msg.data = craye1k->tx_msg_data;

	init_completion(&craye1k->read_complete);
	mutex_init(&craye1k->lock);

	rc = ipmi_create_user(craye1k->iface, &craye1k_msg_handlers,
			craye1k, &craye1k->user);
	if (rc < 0) {
		dev_err(dev, "Unable to register user with IPMI iface %d\n",
				craye1k->iface);
		kfree(craye1k);
		craye1k = NULL;
	} else {
		craye1k_setup_attention_callbacks();
	}
	dev_info(dev, "Cray ClusterStor E1000 slot LEDs registered");
}

static void craye1k_cleanup(void)
{
	if (craye1k) {
		craye1k_restore_attention_callbacks();
		ipmi_destroy_user(craye1k->user);
		kfree(craye1k);
		craye1k = NULL;
	}
}

static void craye1k_smi_gone(int iface)
{
	if (craye1k->iface != iface)
		return;

	craye1k_cleanup();
}

static struct ipmi_smi_watcher smi_watcher = {
	.owner    = THIS_MODULE,
	.new_smi  = craye1k_new_smi,
	.smi_gone = craye1k_smi_gone
};

static bool is_craye1k_board(void)
{
	return dmi_match(DMI_PRODUCT_NAME, "VSSEP1EC");
}

int craye1k_init(void)
{
	int rc = 0;

	if (is_craye1k_board())
		rc = ipmi_smi_watcher_register(&smi_watcher);

	return rc;
}

void craye1k_exit(void)
{
	if (is_craye1k_board()) {
		ipmi_smi_watcher_unregister(&smi_watcher);
		craye1k_cleanup();
	}
}

MODULE_SOFTDEP("pre: pciehp");

module_init(craye1k_init);
module_exit(craye1k_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tony Hutter <hutter2@llnl.gov>");
MODULE_DESCRIPTION("Cray E1000 NVMe Slot LED driver");
