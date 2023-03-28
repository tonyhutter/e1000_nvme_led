# Cray ClusterStor E1000 NVMe slot LED driver

## Getting Started

This is a driver for the ClusterStor E1000 NVMe slot LEDs.  It allows you to
write to the /sys/bus/slot/[1-24]/attention sysfs files to set/clear the fault
and locate LEDs for the NVMe slots:

```
# Set fault LED on slot 1
echo 1 > /sys/bus/slot/1/attention

# Set locate LED on slot 1
echo 2 > /sys/bus/slot/1/attention

# Set fault and locate LEDs on slot 1
echo 3 > /sys/bus/slot/1/attention

# Clear LED on slot 1
echo 0 > /sys/bus/slot/1/attention
```

### Installing

You can build the driver in two ways:

1. Patch it into the kernel and build it:

```
cd <kernel_source_dir>
patch -p1 < Driver-for-Cray-ClusterStor-E1000-NVMe-slot-LEDs.patch
make drivers && make bzImage
```

2. Build it as an external module.  This requires copying over two headers from
the Linux kernel source:

```
cp <kernel_source_dir>/drivers/pci/pcie/portdrv.h .
cp <kernel_source_dir>/drivers/pci/hotplug/pciehp.h .

# rename path "../pcie/portdev.h" to "portdrv.h" (some kernels need this)
sed -i 's/\.\.\/pcie\/portdrv\.h/portdrv.h/g' pciehp.h
make
```

## License
Licensed under the GPLv2 (see LICENSE.md).

LLNL-CODE-843156
