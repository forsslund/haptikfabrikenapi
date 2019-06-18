/*******************************************************
 HIDAPI - Multi-Platform library for
 communication with HID devices.

 Alan Ott
 Signal 11 Software

 8/22/2009
 Linux Version - 6/2/2009

 Copyright 2009, All Rights Reserved.

 At the discretion of the user of this library,
 this software may be licensed under the terms of the
 GNU General Public License v3, a BSD-Style license, or the
 original HIDAPI license as outlined in the LICENSE.txt,
 LICENSE-gpl3.txt, LICENSE-bsd.txt, and LICENSE-orig.txt
 files located at the root of the source distribution.
 These files may also be found in the public source
 code repository located at:
        http://github.com/signal11/hidapi .
********************************************************/
#if defined(LINUX)
/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <locale.h>
#include <errno.h>

/* Unix */
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <fcntl.h>
#include <poll.h>

/* Linux */
#include <linux/hidraw.h>
#include <linux/version.h>
#include <linux/input.h>
#include <libudev.h>

#include "hidapi.h"

/* Definitions from linux/hidraw.h. Since these are new, some distros
   may not have header files which contain them. */
#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#endif
#ifndef HIDIOCGFEATURE
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif


/* USB HID device property names */
const char *device_string_names[] = {
	"manufacturer",
	"product",
	"serial",
};

/* Symbolic names for the properties above */
enum device_string_id {
	DEVICE_STRING_MANUFACTURER,
	DEVICE_STRING_PRODUCT,
	DEVICE_STRING_SERIAL,

	DEVICE_STRING_COUNT,
};

struct hid_device_ {
	int device_handle;
	int blocking;
	int uses_numbered_reports;
};


static __u32 kernel_version = 0;

static __u32 detect_kernel_version(void)
{
	struct utsname name;
	int major, minor, release;
	int ret;

	uname(&name);
	ret = sscanf(name.release, "%d.%d.%d", &major, &minor, &release);
	if (ret == 3) {
		return KERNEL_VERSION(major, minor, release);
	}

	ret = sscanf(name.release, "%d.%d", &major, &minor);
	if (ret == 2) {
		return KERNEL_VERSION(major, minor, 0);
	}

	printf("Couldn't determine kernel version from version string \"%s\"\n", name.release);
	return 0;
}

static hid_device *new_hid_device(void)
{
	hid_device *dev = calloc(1, sizeof(hid_device));
	dev->device_handle = -1;
	dev->blocking = 1;
	dev->uses_numbered_reports = 0;

	return dev;
}


/* The caller must free the returned string with free(). */
static wchar_t *utf8_to_wchar_t(const char *utf8)
{
	wchar_t *ret = NULL;

	if (utf8) {
		size_t wlen = mbstowcs(NULL, utf8, 0);
		if ((size_t) -1 == wlen) {
			return wcsdup(L"");
		}
		ret = calloc(wlen+1, sizeof(wchar_t));
		mbstowcs(ret, utf8, wlen+1);
		ret[wlen] = 0x0000;
	}

	return ret;
}

/* Get an attribute value from a udev_device and return it as a whar_t
   string. The returned string must be freed with free() when done.*/
static wchar_t *copy_udev_string(struct udev_device *dev, const char *udev_name)
{
	return utf8_to_wchar_t(udev_device_get_sysattr_value(dev, udev_name));
}

/* uses_numbered_reports() returns 1 if report_descriptor describes a device
   which contains numbered reports. */
static int uses_numbered_reports(__u8 *report_descriptor, __u32 size) {
	unsigned int i = 0;
	int size_code;
	int data_len, key_size;

	while (i < size) {
		int key = report_descriptor[i];

		/* Check for the Report ID key */
		if (key == 0x85/*Report ID*/) {
			/* This device has a Report ID, which means it uses
			   numbered reports. */
			return 1;
		}

		//printf("key: %02hhx\n", key);

		if ((key & 0xf0) == 0xf0) {
			/* This is a Long Item. The next byte contains the
			   length of the data section (value) for this key.
			   See the HID specification, version 1.11, section
			   6.2.2.3, titled "Long Items." */
			if (i+1 < size)
				data_len = report_descriptor[i+1];
			else
				data_len = 0; /* malformed report */
			key_size = 3;
		}
		else {
			/* This is a Short Item. The bottom two bits of the
			   key contain the size code for the data section
			   (value) for this key.  Refer to the HID
			   specification, version 1.11, section 6.2.2.2,
			   titled "Short Items." */
			size_code = key & 0x3;
			switch (size_code) {
			case 0:
			case 1:
			case 2:
				data_len = size_code;
				break;
			case 3:
				data_len = 4;
				break;
			default:
				/* Can't ever happen since size_code is & 0x3 */
				data_len = 0;
				break;
			};
			key_size = 1;
		}

		/* Skip over this key and it's associated data */
		i += data_len + key_size;
	}

	/* Didn't find a Report ID key. Device doesn't use numbered reports. */
	return 0;
}

/*
 * The caller is responsible for free()ing the (newly-allocated) character
 * strings pointed to by serial_number_utf8 and product_name_utf8 after use.
 */
static int
parse_uevent_info(const char *uevent, int *bus_type,
	unsigned short *vendor_id, unsigned short *product_id,
	char **serial_number_utf8, char **product_name_utf8)
{
	char *tmp = strdup(uevent);
	char *saveptr = NULL;
	char *line;
	char *key;
	char *value;

	int found_id = 0;
	int found_serial = 0;
	int found_name = 0;

	line = strtok_r(tmp, "\n", &saveptr);
	while (line != NULL) {
		/* line: "KEY=value" */
		key = line;
		value = strchr(line, '=');
		if (!value) {
			goto next_line;
		}
		*value = '\0';
		value++;

		if (strcmp(key, "HID_ID") == 0) {
			/**
			 *        type vendor   product
			 * HID_ID=0003:000005AC:00008242
			 **/
			int ret = sscanf(value, "%x:%hx:%hx", bus_type, vendor_id, product_id);
			if (ret == 3) {
				found_id = 1;
			}
		} else if (strcmp(key, "HID_NAME") == 0) {
			/* The caller has to free the product name */
			*product_name_utf8 = strdup(value);
			found_name = 1;
		} else if (strcmp(key, "HID_UNIQ") == 0) {
			/* The caller has to free the serial number */
			*serial_number_utf8 = strdup(value);
			found_serial = 1;
		}

next_line:
		line = strtok_r(NULL, "\n", &saveptr);
	}

	free(tmp);
	return (found_id && found_name && found_serial);
}


static int get_device_string(hid_device *dev, enum device_string_id key, wchar_t *string, size_t maxlen)
{
	struct udev *udev;
	struct udev_device *udev_dev, *parent, *hid_dev;
	struct stat s;
	int ret = -1;
        char *serial_number_utf8 = NULL;
        char *product_name_utf8 = NULL;

	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		return -1;
	}

	/* Get the dev_t (major/minor numbers) from the file handle. */
	fstat(dev->device_handle, &s);
	/* Open a udev device from the dev_t. 'c' means character device. */
	udev_dev = udev_device_new_from_devnum(udev, 'c', s.st_rdev);
	if (udev_dev) {
		hid_dev = udev_device_get_parent_with_subsystem_devtype(
			udev_dev,
			"hid",
			NULL);
		if (hid_dev) {
			unsigned short dev_vid;
			unsigned short dev_pid;
			int bus_type;
			size_t retm;

			ret = parse_uevent_info(
			           udev_device_get_sysattr_value(hid_dev, "uevent"),
			           &bus_type,
			           &dev_vid,
			           &dev_pid,
			           &serial_number_utf8,
			           &product_name_utf8);

			if (bus_type == BUS_BLUETOOTH) {
				switch (key) {
					case DEVICE_STRING_MANUFACTURER:
						wcsncpy(string, L"", maxlen);
						ret = 0;
						break;
					case DEVICE_STRING_PRODUCT:
						retm = mbstowcs(string, product_name_utf8, maxlen);
						ret = (retm == (size_t)-1)? -1: 0;
						break;
					case DEVICE_STRING_SERIAL:
						retm = mbstowcs(string, serial_number_utf8, maxlen);
						ret = (retm == (size_t)-1)? -1: 0;
						break;
					case DEVICE_STRING_COUNT:
					default:
						ret = -1;
						break;
				}
			}
			else {
				/* This is a USB device. Find its parent USB Device node. */
				parent = udev_device_get_parent_with_subsystem_devtype(
					   udev_dev,
					   "usb",
					   "usb_device");
				if (parent) {
					const char *str;
					const char *key_str = NULL;

					if (key >= 0 && key < DEVICE_STRING_COUNT) {
						key_str = device_string_names[key];
					} else {
						ret = -1;
						goto end;
					}

					str = udev_device_get_sysattr_value(parent, key_str);
					if (str) {
						/* Convert the string from UTF-8 to wchar_t */
						retm = mbstowcs(string, str, maxlen);
						ret = (retm == (size_t)-1)? -1: 0;
						goto end;
					}
				}
			}
		}
	}

end:
        free(serial_number_utf8);
        free(product_name_utf8);

	udev_device_unref(udev_dev);
	/* parent and hid_dev don't need to be (and can't be) unref'd.
	   I'm not sure why, but they'll throw double-free() errors. */
	udev_unref(udev);

	return ret;
}

int HID_API_EXPORT hid_init(void)
{
	const char *locale;

	/* Set the locale if it's not set. */
	locale = setlocale(LC_CTYPE, NULL);
	if (!locale)
		setlocale(LC_CTYPE, "");

	kernel_version = detect_kernel_version();

	return 0;
}

int HID_API_EXPORT hid_exit(void)
{
	/* Nothing to do for this in the Linux/hidraw implementation. */
	return 0;
}


struct hid_device_info  HID_API_EXPORT *hid_enumerate(unsigned short vendor_id, unsigned short product_id)
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;

	struct hid_device_info *root = NULL; /* return object */
	struct hid_device_info *cur_dev = NULL;
	struct hid_device_info *prev_dev = NULL; /* previous device */

	hid_init();

	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		return NULL;
	}

	/* Create a list of the devices in the 'hidraw' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "hidraw");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);
	/* For each item, see if it matches the vid/pid, and if so
	   create a udev_device record for it */
	udev_list_entry_foreach(dev_list_entry, devices) {
		const char *sysfs_path;
		const char *dev_path;
		const char *str;
		struct udev_device *raw_dev; /* The device's hidraw udev node. */
		struct udev_device *hid_dev; /* The device's HID udev node. */
		struct udev_device *usb_dev; /* The device's USB udev node. */
		struct udev_device *intf_dev; /* The device's interface (in the USB sense). */
		unsigned short dev_vid;
		unsigned short dev_pid;
		char *serial_number_utf8 = NULL;
		char *product_name_utf8 = NULL;
		int bus_type;
		int result;

		/* Get the filename of the /sys entry for the device
		   and create a udev_device object (dev) representing it */
		sysfs_path = udev_list_entry_get_name(dev_list_entry);
		raw_dev = udev_device_new_from_syspath(udev, sysfs_path);
		dev_path = udev_device_get_devnode(raw_dev);

		hid_dev = udev_device_get_parent_with_subsystem_devtype(
			raw_dev,
			"hid",
			NULL);

		if (!hid_dev) {
			/* Unable to find parent hid device. */
			goto next;
		}

		result = parse_uevent_info(
			udev_device_get_sysattr_value(hid_dev, "uevent"),
			&bus_type,
			&dev_vid,
			&dev_pid,
			&serial_number_utf8,
			&product_name_utf8);

		if (!result) {
			/* parse_uevent_info() failed for at least one field. */
			goto next;
		}

		if (bus_type != BUS_USB && bus_type != BUS_BLUETOOTH) {
			/* We only know how to handle USB and BT devices. */
			goto next;
		}

		/* Check the VID/PID against the arguments */
		if ((vendor_id == 0x0 || vendor_id == dev_vid) &&
		    (product_id == 0x0 || product_id == dev_pid)) {
			struct hid_device_info *tmp;

			/* VID/PID match. Create the record. */
			tmp = malloc(sizeof(struct hid_device_info));
			if (cur_dev) {
				cur_dev->next = tmp;
			}
			else {
				root = tmp;
			}
			prev_dev = cur_dev;
			cur_dev = tmp;

			/* Fill out the record */
			cur_dev->next = NULL;
			cur_dev->path = dev_path? strdup(dev_path): NULL;

			/* VID/PID */
			cur_dev->vendor_id = dev_vid;
			cur_dev->product_id = dev_pid;

			/* Serial Number */
			cur_dev->serial_number = utf8_to_wchar_t(serial_number_utf8);

			/* Release Number */
			cur_dev->release_number = 0x0;

			/* Interface Number */
			cur_dev->interface_number = -1;

			switch (bus_type) {
				case BUS_USB:
					/* The device pointed to by raw_dev contains information about
					   the hidraw device. In order to get information about the
					   USB device, get the parent device with the
					   subsystem/devtype pair of "usb"/"usb_device". This will
					   be several levels up the tree, but the function will find
					   it. */
					usb_dev = udev_device_get_parent_with_subsystem_devtype(
							raw_dev,
							"usb",
							"usb_device");

					if (!usb_dev) {
						/* Free this device */
						free(cur_dev->serial_number);
						free(cur_dev->path);
						free(cur_dev);

						/* Take it off the device list. */
						if (prev_dev) {
							prev_dev->next = NULL;
							cur_dev = prev_dev;
						}
						else {
							cur_dev = root = NULL;
						}

						goto next;
					}

					/* Manufacturer and Product strings */
					cur_dev->manufacturer_string = copy_udev_string(usb_dev, device_string_names[DEVICE_STRING_MANUFACTURER]);
					cur_dev->product_string = copy_udev_string(usb_dev, device_string_names[DEVICE_STRING_PRODUCT]);

					/* Release Number */
					str = udev_device_get_sysattr_value(usb_dev, "bcdDevice");
					cur_dev->release_number = (str)? strtol(str, NULL, 16): 0x0;

					/* Get a handle to the interface's udev node. */
					intf_dev = udev_device_get_parent_with_subsystem_devtype(
							raw_dev,
							"usb",
							"usb_interface");
					if (intf_dev) {
						str = udev_device_get_sysattr_value(intf_dev, "bInterfaceNumber");
						cur_dev->interface_number = (str)? strtol(str, NULL, 16): -1;
					}

					break;

				case BUS_BLUETOOTH:
					/* Manufacturer and Product strings */
					cur_dev->manufacturer_string = wcsdup(L"");
					cur_dev->product_string = utf8_to_wchar_t(product_name_utf8);

					break;

				default:
					/* Unknown device type - this should never happen, as we
					 * check for USB and Bluetooth devices above */
					break;
			}
		}

	next:
		free(serial_number_utf8);
		free(product_name_utf8);
		udev_device_unref(raw_dev);
		/* hid_dev, usb_dev and intf_dev don't need to be (and can't be)
		   unref()d.  It will cause a double-free() error.  I'm not
		   sure why.  */
	}
	/* Free the enumerator and udev objects. */
	udev_enumerate_unref(enumerate);
	udev_unref(udev);

	return root;
}

void  HID_API_EXPORT hid_free_enumeration(struct hid_device_info *devs)
{
	struct hid_device_info *d = devs;
	while (d) {
		struct hid_device_info *next = d->next;
		free(d->path);
		free(d->serial_number);
		free(d->manufacturer_string);
		free(d->product_string);
		free(d);
		d = next;
	}
}

hid_device * hid_open(unsigned short vendor_id, unsigned short product_id, const wchar_t *serial_number)
{
	struct hid_device_info *devs, *cur_dev;
	const char *path_to_open = NULL;
	hid_device *handle = NULL;

	devs = hid_enumerate(vendor_id, product_id);
	cur_dev = devs;
	while (cur_dev) {
		if (cur_dev->vendor_id == vendor_id &&
		    cur_dev->product_id == product_id) {
			if (serial_number) {
				if (wcscmp(serial_number, cur_dev->serial_number) == 0) {
					path_to_open = cur_dev->path;
					break;
				}
			}
			else {
				path_to_open = cur_dev->path;
				break;
			}
		}
		cur_dev = cur_dev->next;
	}

	if (path_to_open) {
		/* Open the device */
		handle = hid_open_path(path_to_open);
	}

	hid_free_enumeration(devs);

	return handle;
}

hid_device * HID_API_EXPORT hid_open_path(const char *path)
{
	hid_device *dev = NULL;

	hid_init();

	dev = new_hid_device();

	/* OPEN HERE */
	dev->device_handle = open(path, O_RDWR);

	/* If we have a good handle, return it. */
	if (dev->device_handle > 0) {

		/* Get the report descriptor */
		int res, desc_size = 0;
		struct hidraw_report_descriptor rpt_desc;

		memset(&rpt_desc, 0x0, sizeof(rpt_desc));

		/* Get Report Descriptor Size */
		res = ioctl(dev->device_handle, HIDIOCGRDESCSIZE, &desc_size);
		if (res < 0)
			perror("HIDIOCGRDESCSIZE");


		/* Get Report Descriptor */
		rpt_desc.size = desc_size;
		res = ioctl(dev->device_handle, HIDIOCGRDESC, &rpt_desc);
		if (res < 0) {
			perror("HIDIOCGRDESC");
		} else {
			/* Determine if this device uses numbered reports. */
			dev->uses_numbered_reports =
				uses_numbered_reports(rpt_desc.value,
				                      rpt_desc.size);
		}

		return dev;
	}
	else {
		/* Unable to open any devices. */
		free(dev);
		return NULL;
	}
}


int HID_API_EXPORT hid_write(hid_device *dev, const unsigned char *data, size_t length)
{
	int bytes_written;

	bytes_written = write(dev->device_handle, data, length);

	return bytes_written;
}


int HID_API_EXPORT hid_read_timeout(hid_device *dev, unsigned char *data, size_t length, int milliseconds)
{
	int bytes_read;

	if (milliseconds >= 0) {
		/* Milliseconds is either 0 (non-blocking) or > 0 (contains
		   a valid timeout). In both cases we want to call poll()
		   and wait for data to arrive.  Don't rely on non-blocking
		   operation (O_NONBLOCK) since some kernels don't seem to
		   properly report device disconnection through read() when
		   in non-blocking mode.  */
		int ret;
		struct pollfd fds;

		fds.fd = dev->device_handle;
		fds.events = POLLIN;
		fds.revents = 0;
		ret = poll(&fds, 1, milliseconds);
		if (ret == -1 || ret == 0) {
			/* Error or timeout */
			return ret;
		}
		else {
			/* Check for errors on the file descriptor. This will
			   indicate a device disconnection. */
			if (fds.revents & (POLLERR | POLLHUP | POLLNVAL))
				return -1;
		}
	}

	bytes_read = read(dev->device_handle, data, length);
	if (bytes_read < 0 && (errno == EAGAIN || errno == EINPROGRESS))
		bytes_read = 0;

	if (bytes_read >= 0 &&
	    kernel_version != 0 &&
	    kernel_version < KERNEL_VERSION(2,6,34) &&
	    dev->uses_numbered_reports) {
		/* Work around a kernel bug. Chop off the first byte. */
		memmove(data, data+1, bytes_read);
		bytes_read--;
	}

	return bytes_read;
}

int HID_API_EXPORT hid_read(hid_device *dev, unsigned char *data, size_t length)
{
	return hid_read_timeout(dev, data, length, (dev->blocking)? -1: 0);
}

int HID_API_EXPORT hid_set_nonblocking(hid_device *dev, int nonblock)
{
	/* Do all non-blocking in userspace using poll(), since it looks
	   like there's a bug in the kernel in some versions where
	   read() will not return -1 on disconnection of the USB device */

	dev->blocking = !nonblock;
	return 0; /* Success */
}


int HID_API_EXPORT hid_send_feature_report(hid_device *dev, const unsigned char *data, size_t length)
{
	int res;

	res = ioctl(dev->device_handle, HIDIOCSFEATURE(length), data);
	if (res < 0)
		perror("ioctl (SFEATURE)");

	return res;
}

int HID_API_EXPORT hid_get_feature_report(hid_device *dev, unsigned char *data, size_t length)
{
	int res;

	res = ioctl(dev->device_handle, HIDIOCGFEATURE(length), data);
	if (res < 0)
		perror("ioctl (GFEATURE)");


	return res;
}


void HID_API_EXPORT hid_close(hid_device *dev)
{
	if (!dev)
		return;
	close(dev->device_handle);
	free(dev);
}


int HID_API_EXPORT_CALL hid_get_manufacturer_string(hid_device *dev, wchar_t *string, size_t maxlen)
{
	return get_device_string(dev, DEVICE_STRING_MANUFACTURER, string, maxlen);
}

int HID_API_EXPORT_CALL hid_get_product_string(hid_device *dev, wchar_t *string, size_t maxlen)
{
	return get_device_string(dev, DEVICE_STRING_PRODUCT, string, maxlen);
}

int HID_API_EXPORT_CALL hid_get_serial_number_string(hid_device *dev, wchar_t *string, size_t maxlen)
{
	return get_device_string(dev, DEVICE_STRING_SERIAL, string, maxlen);
}

/*
int HID_API_EXPORT_CALL hid_get_indexed_string(hid_device *dev, int string_index, wchar_t *string, size_t maxlen)
{
	return -1;
}


HID_API_EXPORT const wchar_t * HID_API_CALL  hid_error(hid_device *dev)
{
	return NULL;
}
*/
#else
/*******************************************************
 HIDAPI - Multi-Platform library for
 communication with HID devices.

 Alan Ott
 Signal 11 Software

 8/22/2009

 Copyright 2009, All Rights Reserved.

 At the discretion of the user of this library,
 this software may be licensed under the terms of the
 GNU General Public License v3, a BSD-Style license, or the
 original HIDAPI license as outlined in the LICENSE.txt,
 LICENSE-gpl3.txt, LICENSE-bsd.txt, and LICENSE-orig.txt
 files located at the root of the source distribution.
 These files may also be found in the public source
 code repository located at:
        http://github.com/signal11/hidapi .
********************************************************/

#include <windows.h>

#ifndef _NTDEF_
typedef LONG NTSTATUS;
#endif

#ifdef __MINGW32__
#include <ntdef.h>
#include <winbase.h>
#endif

#ifdef __CYGWIN__
#include <ntdef.h>
#define _wcsdup wcsdup
#endif

/* The maximum number of characters that can be passed into the
   HidD_Get*String() functions without it failing.*/
#define MAX_STRING_WCHARS 0xFFF

/*#define HIDAPI_USE_DDK*/

#ifdef __cplusplus
extern "C" {
#endif
    #include <setupapi.h>
    #include <winioctl.h>
    #ifdef HIDAPI_USE_DDK
        #include <hidsdi.h>
    #endif

    /* Copied from inc/ddk/hidclass.h, part of the Windows DDK. */
    #define HID_OUT_CTL_CODE(id)  \
        CTL_CODE(FILE_DEVICE_KEYBOARD, (id), METHOD_OUT_DIRECT, FILE_ANY_ACCESS)
    #define IOCTL_HID_GET_FEATURE                   HID_OUT_CTL_CODE(100)

#ifdef __cplusplus
} /* extern "C" */
#endif

#include <stdio.h>
#include <stdlib.h>


#include "hidapi.h"

#undef MIN
#define MIN(x,y) ((x) < (y)? (x): (y))

#ifdef _MSC_VER
    /* Thanks Microsoft, but I know how to use strncpy(). */
    #pragma warning(disable:4996)
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HIDAPI_USE_DDK
    /* Since we're not building with the DDK, and the HID header
       files aren't part of the SDK, we have to define all this
       stuff here. In lookup_functions(), the function pointers
       defined below are set. */
    typedef struct _HIDD_ATTRIBUTES{
        ULONG Size;
        USHORT VendorID;
        USHORT ProductID;
        USHORT VersionNumber;
    } HIDD_ATTRIBUTES, *PHIDD_ATTRIBUTES;

    typedef USHORT USAGE;
    typedef struct _HIDP_CAPS {
        USAGE Usage;
        USAGE UsagePage;
        USHORT InputReportByteLength;
        USHORT OutputReportByteLength;
        USHORT FeatureReportByteLength;
        USHORT Reserved[17];
        USHORT fields_not_used_by_hidapi[10];
    } HIDP_CAPS, *PHIDP_CAPS;
    typedef void* PHIDP_PREPARSED_DATA;
    #define HIDP_STATUS_SUCCESS 0x110000

    typedef BOOLEAN (__stdcall *HidD_GetAttributes_)(HANDLE device, PHIDD_ATTRIBUTES attrib);
    typedef BOOLEAN (__stdcall *HidD_GetSerialNumberString_)(HANDLE device, PVOID buffer, ULONG buffer_len);
    typedef BOOLEAN (__stdcall *HidD_GetManufacturerString_)(HANDLE handle, PVOID buffer, ULONG buffer_len);
    typedef BOOLEAN (__stdcall *HidD_GetProductString_)(HANDLE handle, PVOID buffer, ULONG buffer_len);
    typedef BOOLEAN (__stdcall *HidD_SetFeature_)(HANDLE handle, PVOID data, ULONG length);
    typedef BOOLEAN (__stdcall *HidD_GetFeature_)(HANDLE handle, PVOID data, ULONG length);
    typedef BOOLEAN (__stdcall *HidD_GetIndexedString_)(HANDLE handle, ULONG string_index, PVOID buffer, ULONG buffer_len);
    typedef BOOLEAN (__stdcall *HidD_GetPreparsedData_)(HANDLE handle, PHIDP_PREPARSED_DATA *preparsed_data);
    typedef BOOLEAN (__stdcall *HidD_FreePreparsedData_)(PHIDP_PREPARSED_DATA preparsed_data);
    typedef NTSTATUS (__stdcall *HidP_GetCaps_)(PHIDP_PREPARSED_DATA preparsed_data, HIDP_CAPS *caps);
    typedef BOOLEAN (__stdcall *HidD_SetNumInputBuffers_)(HANDLE handle, ULONG number_buffers);

    static HidD_GetAttributes_ HidD_GetAttributes;
    static HidD_GetSerialNumberString_ HidD_GetSerialNumberString;
    static HidD_GetManufacturerString_ HidD_GetManufacturerString;
    static HidD_GetProductString_ HidD_GetProductString;
    static HidD_SetFeature_ HidD_SetFeature;
    static HidD_GetFeature_ HidD_GetFeature;
    static HidD_GetIndexedString_ HidD_GetIndexedString;
    static HidD_GetPreparsedData_ HidD_GetPreparsedData;
    static HidD_FreePreparsedData_ HidD_FreePreparsedData;
    static HidP_GetCaps_ HidP_GetCaps;
    static HidD_SetNumInputBuffers_ HidD_SetNumInputBuffers;

    static HMODULE lib_handle = NULL;
    static BOOLEAN initialized = FALSE;
#endif /* HIDAPI_USE_DDK */

struct hid_device_ {
        HANDLE device_handle;
        BOOL blocking;
        USHORT output_report_length;
        size_t input_report_length;
        void *last_error_str;
        DWORD last_error_num;
        BOOL read_pending;
        char *read_buf;
        OVERLAPPED ol;
};

static hid_device *new_hid_device()
{
    hid_device *dev = (hid_device*) calloc(1, sizeof(hid_device));
    dev->device_handle = INVALID_HANDLE_VALUE;
    dev->blocking = TRUE;
    dev->output_report_length = 0;
    dev->input_report_length = 0;
    dev->last_error_str = NULL;
    dev->last_error_num = 0;
    dev->read_pending = FALSE;
    dev->read_buf = NULL;
    memset(&dev->ol, 0, sizeof(dev->ol));
    dev->ol.hEvent = CreateEvent(NULL, FALSE, FALSE /*initial state f=nonsignaled*/, NULL);

    return dev;
}

static void free_hid_device(hid_device *dev)
{
    CloseHandle(dev->ol.hEvent);
    CloseHandle(dev->device_handle);
    LocalFree(dev->last_error_str);
    free(dev->read_buf);
    free(dev);
}

static void register_error(hid_device *device, const char *op)
{
    WCHAR *ptr, *msg;

    FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER |
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        GetLastError(),
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPVOID)&msg, 0/*sz*/,
        NULL);

    /* Get rid of the CR and LF that FormatMessage() sticks at the
       end of the message. Thanks Microsoft! */
    ptr = msg;
    while (*ptr) {
        if (*ptr == '\r') {
            *ptr = 0x0000;
            break;
        }
        ptr++;
    }

    /* Store the message off in the Device entry so that
       the hid_error() function can pick it up. */
    LocalFree(device->last_error_str);
    device->last_error_str = msg;
}

#ifndef HIDAPI_USE_DDK
static int lookup_functions()
{
    lib_handle = LoadLibraryA("hid.dll");
    if (lib_handle) {
#define RESOLVE(x) x = (x##_)GetProcAddress(lib_handle, #x); if (!x) return -1;
        RESOLVE(HidD_GetAttributes);
        RESOLVE(HidD_GetSerialNumberString);
        RESOLVE(HidD_GetManufacturerString);
        RESOLVE(HidD_GetProductString);
        RESOLVE(HidD_SetFeature);
        RESOLVE(HidD_GetFeature);
        RESOLVE(HidD_GetIndexedString);
        RESOLVE(HidD_GetPreparsedData);
        RESOLVE(HidD_FreePreparsedData);
        RESOLVE(HidP_GetCaps);
        RESOLVE(HidD_SetNumInputBuffers);
#undef RESOLVE
    }
    else
        return -1;

    return 0;
}
#endif

static HANDLE open_device(const char *path, BOOL enumerate)
{
    HANDLE handle;
    DWORD desired_access = (enumerate)? 0: (GENERIC_WRITE | GENERIC_READ);
    DWORD share_mode = FILE_SHARE_READ|FILE_SHARE_WRITE;

    handle = CreateFileA(path,
        desired_access,
        share_mode,
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,/*FILE_ATTRIBUTE_NORMAL,*/
        0);

    return handle;
}

int HID_API_EXPORT hid_init(void)
{
#ifndef HIDAPI_USE_DDK
    if (!initialized) {
        if (lookup_functions() < 0) {
            hid_exit();
            return -1;
        }
        initialized = TRUE;
    }
#endif
    return 0;
}

int HID_API_EXPORT hid_exit(void)
{
#ifndef HIDAPI_USE_DDK
    if (lib_handle)
        FreeLibrary(lib_handle);
    lib_handle = NULL;
    initialized = FALSE;
#endif
    return 0;
}

struct hid_device_info HID_API_EXPORT * HID_API_CALL hid_enumerate(unsigned short vendor_id, unsigned short product_id)
{
    BOOL res;
    struct hid_device_info *root = NULL; /* return object */
    struct hid_device_info *cur_dev = NULL;

    /* Windows objects for interacting with the driver. */
    GUID InterfaceClassGuid = {0x4d1e55b2, 0xf16f, 0x11cf, {0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30} };
    SP_DEVINFO_DATA devinfo_data;
    SP_DEVICE_INTERFACE_DATA device_interface_data;
    SP_DEVICE_INTERFACE_DETAIL_DATA_A *device_interface_detail_data = NULL;
    HDEVINFO device_info_set = INVALID_HANDLE_VALUE;
    int device_index = 0;
    int i;

    if (hid_init() < 0)
        return NULL;

    /* Initialize the Windows objects. */
    memset(&devinfo_data, 0x0, sizeof(devinfo_data));
    devinfo_data.cbSize = sizeof(SP_DEVINFO_DATA);
    device_interface_data.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    /* Get information for all the devices belonging to the HID class. */
    device_info_set = SetupDiGetClassDevsA(&InterfaceClassGuid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    /* Iterate over each device in the HID class, looking for the right one. */

    for (;;) {
        HANDLE write_handle = INVALID_HANDLE_VALUE;
        DWORD required_size = 0;
        HIDD_ATTRIBUTES attrib;

        res = SetupDiEnumDeviceInterfaces(device_info_set,
            NULL,
            &InterfaceClassGuid,
            device_index,
            &device_interface_data);

        if (!res) {
            /* A return of FALSE from this function means that
               there are no more devices. */
            break;
        }

        /* Call with 0-sized detail size, and let the function
           tell us how long the detail struct needs to be. The
           size is put in &required_size. */
        res = SetupDiGetDeviceInterfaceDetailA(device_info_set,
            &device_interface_data,
            NULL,
            0,
            &required_size,
            NULL);

        /* Allocate a long enough structure for device_interface_detail_data. */
        device_interface_detail_data = (SP_DEVICE_INTERFACE_DETAIL_DATA_A*) malloc(required_size);
        device_interface_detail_data->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);

        /* Get the detailed data for this device. The detail data gives us
           the device path for this device, which is then passed into
           CreateFile() to get a handle to the device. */
        res = SetupDiGetDeviceInterfaceDetailA(device_info_set,
            &device_interface_data,
            device_interface_detail_data,
            required_size,
            NULL,
            NULL);

        if (!res) {
            /* register_error(dev, "Unable to call SetupDiGetDeviceInterfaceDetail");
               Continue to the next device. */
            goto cont;
        }

        /* Make sure this device is of Setup Class "HIDClass" and has a
           driver bound to it. */
        for (i = 0; ; i++) {
            char driver_name[256];

            /* Populate devinfo_data. This function will return failure
               when there are no more interfaces left. */
            res = SetupDiEnumDeviceInfo(device_info_set, i, &devinfo_data);
            if (!res)
                goto cont;

            res = SetupDiGetDeviceRegistryPropertyA(device_info_set, &devinfo_data,
                           SPDRP_CLASS, NULL, (PBYTE)driver_name, sizeof(driver_name), NULL);
            if (!res)
                goto cont;

            if (strcmp(driver_name, "HIDClass") == 0) {
                /* See if there's a driver bound. */
                res = SetupDiGetDeviceRegistryPropertyA(device_info_set, &devinfo_data,
                           SPDRP_DRIVER, NULL, (PBYTE)driver_name, sizeof(driver_name), NULL);
                if (res)
                    break;
            }
        }

        //wprintf(L"HandleName: %s\n", device_interface_detail_data->DevicePath);

        /* Open a handle to the device */
        write_handle = open_device(device_interface_detail_data->DevicePath, TRUE);

        /* Check validity of write_handle. */
        if (write_handle == INVALID_HANDLE_VALUE) {
            /* Unable to open the device. */
            //register_error(dev, "CreateFile");
            goto cont_close;
        }


        /* Get the Vendor ID and Product ID for this device. */
        attrib.Size = sizeof(HIDD_ATTRIBUTES);
        HidD_GetAttributes(write_handle, &attrib);
        //wprintf(L"Product/Vendor: %x %x\n", attrib.ProductID, attrib.VendorID);

        /* Check the VID/PID to see if we should add this
           device to the enumeration list. */
        if ((vendor_id == 0x0 || attrib.VendorID == vendor_id) &&
            (product_id == 0x0 || attrib.ProductID == product_id)) {

            #define WSTR_LEN 512
            const char *str;
            struct hid_device_info *tmp;
            PHIDP_PREPARSED_DATA pp_data = NULL;
            HIDP_CAPS caps;
            BOOLEAN res;
            NTSTATUS nt_res;
            wchar_t wstr[WSTR_LEN]; /* TODO: Determine Size */
            size_t len;

            /* VID/PID match. Create the record. */
            tmp = (struct hid_device_info*) calloc(1, sizeof(struct hid_device_info));
            if (cur_dev) {
                cur_dev->next = tmp;
            }
            else {
                root = tmp;
            }
            cur_dev = tmp;

            /* Get the Usage Page and Usage for this device. */
            res = HidD_GetPreparsedData(write_handle, &pp_data);
            if (res) {
                nt_res = HidP_GetCaps(pp_data, &caps);
                if (nt_res == HIDP_STATUS_SUCCESS) {
                    cur_dev->usage_page = caps.UsagePage;
                    cur_dev->usage = caps.Usage;
                }

                HidD_FreePreparsedData(pp_data);
            }

            /* Fill out the record */
            cur_dev->next = NULL;
            str = device_interface_detail_data->DevicePath;
            if (str) {
                len = strlen(str);
                cur_dev->path = (char*) calloc(len+1, sizeof(char));
                strncpy(cur_dev->path, str, len+1);
                cur_dev->path[len] = '\0';
            }
            else
                cur_dev->path = NULL;

            /* Serial Number */
            res = HidD_GetSerialNumberString(write_handle, wstr, sizeof(wstr));
            wstr[WSTR_LEN-1] = 0x0000;
            if (res) {
                cur_dev->serial_number = _wcsdup(wstr);
            }

            /* Manufacturer String */
            res = HidD_GetManufacturerString(write_handle, wstr, sizeof(wstr));
            wstr[WSTR_LEN-1] = 0x0000;
            if (res) {
                cur_dev->manufacturer_string = _wcsdup(wstr);
            }

            /* Product String */
            res = HidD_GetProductString(write_handle, wstr, sizeof(wstr));
            wstr[WSTR_LEN-1] = 0x0000;
            if (res) {
                cur_dev->product_string = _wcsdup(wstr);
            }

            /* VID/PID */
            cur_dev->vendor_id = attrib.VendorID;
            cur_dev->product_id = attrib.ProductID;

            /* Release Number */
            cur_dev->release_number = attrib.VersionNumber;

            /* Interface Number. It can sometimes be parsed out of the path
               on Windows if a device has multiple interfaces. See
               http://msdn.microsoft.com/en-us/windows/hardware/gg487473 or
               search for "Hardware IDs for HID Devices" at MSDN. If it's not
               in the path, it's set to -1. */
            cur_dev->interface_number = -1;
            if (cur_dev->path) {
                char *interface_component = strstr(cur_dev->path, "&mi_");
                if (interface_component) {
                    char *hex_str = interface_component + 4;
                    char *endptr = NULL;
                    cur_dev->interface_number = strtol(hex_str, &endptr, 16);
                    if (endptr == hex_str) {
                        /* The parsing failed. Set interface_number to -1. */
                        cur_dev->interface_number = -1;
                    }
                }
            }
        }

cont_close:
        CloseHandle(write_handle);
cont:
        /* We no longer need the detail data. It can be freed */
        free(device_interface_detail_data);

        device_index++;

    }

    /* Close the device information handle. */
    SetupDiDestroyDeviceInfoList(device_info_set);

    return root;

}

void  HID_API_EXPORT HID_API_CALL hid_free_enumeration(struct hid_device_info *devs)
{
    /* TODO: Merge this with the Linux version. This function is platform-independent. */
    struct hid_device_info *d = devs;
    while (d) {
        struct hid_device_info *next = d->next;
        free(d->path);
        free(d->serial_number);
        free(d->manufacturer_string);
        free(d->product_string);
        free(d);
        d = next;
    }
}


HID_API_EXPORT hid_device * HID_API_CALL hid_open(unsigned short vendor_id, unsigned short product_id, const wchar_t *serial_number)
{
    /* TODO: Merge this functions with the Linux version. This function should be platform independent. */
    struct hid_device_info *devs, *cur_dev;
    const char *path_to_open = NULL;
    hid_device *handle = NULL;

    devs = hid_enumerate(vendor_id, product_id);
    cur_dev = devs;
    while (cur_dev) {
        if (cur_dev->vendor_id == vendor_id &&
            cur_dev->product_id == product_id) {
            if (serial_number) {
                if (wcscmp(serial_number, cur_dev->serial_number) == 0) {
                    path_to_open = cur_dev->path;
                    break;
                }
            }
            else {
                path_to_open = cur_dev->path;
                break;
            }
        }
        cur_dev = cur_dev->next;
    }

    if (path_to_open) {
        /* Open the device */
        handle = hid_open_path(path_to_open);
    }

    hid_free_enumeration(devs);

    return handle;
}

HID_API_EXPORT hid_device * HID_API_CALL hid_open_path(const char *path)
{
    hid_device *dev;
    HIDP_CAPS caps;
    PHIDP_PREPARSED_DATA pp_data = NULL;
    BOOLEAN res;
    NTSTATUS nt_res;

    if (hid_init() < 0) {
        return NULL;
    }

    dev = new_hid_device();

    /* Open a handle to the device */
    dev->device_handle = open_device(path, FALSE);

    /* Check validity of write_handle. */
    if (dev->device_handle == INVALID_HANDLE_VALUE) {
        /* Unable to open the device. */
        register_error(dev, "CreateFile");
        goto err;
    }

    /* Set the Input Report buffer size to 64 reports. */
    res = HidD_SetNumInputBuffers(dev->device_handle, 64);
    if (!res) {
        register_error(dev, "HidD_SetNumInputBuffers");
        goto err;
    }

    /* Get the Input Report length for the device. */
    res = HidD_GetPreparsedData(dev->device_handle, &pp_data);
    if (!res) {
        register_error(dev, "HidD_GetPreparsedData");
        goto err;
    }
    nt_res = HidP_GetCaps(pp_data, &caps);
    if (nt_res != HIDP_STATUS_SUCCESS) {
        register_error(dev, "HidP_GetCaps");
        goto err_pp_data;
    }
    dev->output_report_length = caps.OutputReportByteLength;
    dev->input_report_length = caps.InputReportByteLength;
    HidD_FreePreparsedData(pp_data);

    dev->read_buf = (char*) malloc(dev->input_report_length);

    return dev;

err_pp_data:
        HidD_FreePreparsedData(pp_data);
err:
        free_hid_device(dev);
        return NULL;
}

int HID_API_EXPORT HID_API_CALL hid_write(hid_device *dev, const unsigned char *data, size_t length)
{
    DWORD bytes_written;
    BOOL res;

    OVERLAPPED ol;
    unsigned char *buf;
    memset(&ol, 0, sizeof(ol));

    /* Make sure the right number of bytes are passed to WriteFile. Windows
       expects the number of bytes which are in the _longest_ report (plus
       one for the report number) bytes even if the data is a report
       which is shorter than that. Windows gives us this value in
       caps.OutputReportByteLength. If a user passes in fewer bytes than this,
       create a temporary buffer which is the proper size. */
    if (length >= dev->output_report_length) {
        /* The user passed the right number of bytes. Use the buffer as-is. */
        buf = (unsigned char *) data;
    } else {
        /* Create a temporary buffer and copy the user's data
           into it, padding the rest with zeros. */
        buf = (unsigned char *) malloc(dev->output_report_length);
        memcpy(buf, data, length);
        memset(buf + length, 0, dev->output_report_length - length);
        length = dev->output_report_length;
    }

    res = WriteFile(dev->device_handle, buf, length, NULL, &ol);

    if (!res) {
        if (GetLastError() != ERROR_IO_PENDING) {
            /* WriteFile() failed. Return error. */
            register_error(dev, "WriteFile");
            bytes_written = -1;
            goto end_of_function;
        }
    }

    /* Wait here until the write is done. This makes
       hid_write() synchronous. */
    res = GetOverlappedResult(dev->device_handle, &ol, &bytes_written, TRUE/*wait*/);
    if (!res) {
        /* The Write operation failed. */
        register_error(dev, "WriteFile");
        bytes_written = -1;
        goto end_of_function;
    }

end_of_function:
    if (buf != data)
        free(buf);

    return bytes_written;
}


int HID_API_EXPORT HID_API_CALL hid_read_timeout(hid_device *dev, unsigned char *data, size_t length, int milliseconds)
{
    DWORD bytes_read = 0;
    size_t copy_len = 0;
    BOOL res;

    /* Copy the handle for convenience. */
    HANDLE ev = dev->ol.hEvent;

    if (!dev->read_pending) {
        /* Start an Overlapped I/O read. */
        dev->read_pending = TRUE;
        memset(dev->read_buf, 0, dev->input_report_length);
        ResetEvent(ev);
        res = ReadFile(dev->device_handle, dev->read_buf, dev->input_report_length, &bytes_read, &dev->ol);

        if (!res) {
            if (GetLastError() != ERROR_IO_PENDING) {
                /* ReadFile() has failed.
                   Clean up and return error. */
                CancelIo(dev->device_handle);
                dev->read_pending = FALSE;
                goto end_of_function;
            }
        }
    }

    if (milliseconds >= 0) {
        /* See if there is any data yet. */
        res = WaitForSingleObject(ev, milliseconds);
        if (res != WAIT_OBJECT_0) {
            /* There was no data this time. Return zero bytes available,
               but leave the Overlapped I/O running. */
            return 0;
        }
    }

    /* Either WaitForSingleObject() told us that ReadFile has completed, or
       we are in non-blocking mode. Get the number of bytes read. The actual
       data has been copied to the data[] array which was passed to ReadFile(). */
    res = GetOverlappedResult(dev->device_handle, &dev->ol, &bytes_read, TRUE/*wait*/);

    /* Set pending back to false, even if GetOverlappedResult() returned error. */
    dev->read_pending = FALSE;

    if (res && bytes_read > 0) {
        if (dev->read_buf[0] == 0x0) {
            /* If report numbers aren't being used, but Windows sticks a report
               number (0x0) on the beginning of the report anyway. To make this
               work like the other platforms, and to make it work more like the
               HID spec, we'll skip over this byte. */
            bytes_read--;
            copy_len = length > bytes_read ? bytes_read : length;
            memcpy(data, dev->read_buf+1, copy_len);
        }
        else {
            /* Copy the whole buffer, report number and all. */
            copy_len = length > bytes_read ? bytes_read : length;
            memcpy(data, dev->read_buf, copy_len);
        }
    }

end_of_function:
    if (!res) {
        register_error(dev, "GetOverlappedResult");
        return -1;
    }

    return copy_len;
}

int HID_API_EXPORT HID_API_CALL hid_read(hid_device *dev, unsigned char *data, size_t length)
{
    return hid_read_timeout(dev, data, length, (dev->blocking)? -1: 0);
}

int HID_API_EXPORT HID_API_CALL hid_set_nonblocking(hid_device *dev, int nonblock)
{
    dev->blocking = !nonblock;
    return 0; /* Success */
}

int HID_API_EXPORT HID_API_CALL hid_send_feature_report(hid_device *dev, const unsigned char *data, size_t length)
{
    BOOL res = HidD_SetFeature(dev->device_handle, (PVOID)data, length);
    if (!res) {
        register_error(dev, "HidD_SetFeature");
        return -1;
    }

    return length;
}


int HID_API_EXPORT HID_API_CALL hid_get_feature_report(hid_device *dev, unsigned char *data, size_t length)
{
    BOOL res;
#if 0
    res = HidD_GetFeature(dev->device_handle, data, length);
    if (!res) {
        register_error(dev, "HidD_GetFeature");
        return -1;
    }
    return 0; /* HidD_GetFeature() doesn't give us an actual length, unfortunately */
#else
    DWORD bytes_returned;

    OVERLAPPED ol;
    memset(&ol, 0, sizeof(ol));

    res = DeviceIoControl(dev->device_handle,
        IOCTL_HID_GET_FEATURE,
        data, length,
        data, length,
        &bytes_returned, &ol);

    if (!res) {
        if (GetLastError() != ERROR_IO_PENDING) {
            /* DeviceIoControl() failed. Return error. */
            register_error(dev, "Send Feature Report DeviceIoControl");
            return -1;
        }
    }

    /* Wait here until the write is done. This makes
       hid_get_feature_report() synchronous. */
    res = GetOverlappedResult(dev->device_handle, &ol, &bytes_returned, TRUE/*wait*/);
    if (!res) {
        /* The operation failed. */
        register_error(dev, "Send Feature Report GetOverLappedResult");
        return -1;
    }

    /* bytes_returned does not include the first byte which contains the
       report ID. The data buffer actually contains one more byte than
       bytes_returned. */
    bytes_returned++;

    return bytes_returned;
#endif
}

void HID_API_EXPORT HID_API_CALL hid_close(hid_device *dev)
{
    if (!dev)
        return;
    CancelIo(dev->device_handle);
    free_hid_device(dev);
}

int HID_API_EXPORT_CALL HID_API_CALL hid_get_manufacturer_string(hid_device *dev, wchar_t *string, size_t maxlen)
{
    BOOL res;

    res = HidD_GetManufacturerString(dev->device_handle, string, sizeof(wchar_t) * MIN(maxlen, MAX_STRING_WCHARS));
    if (!res) {
        register_error(dev, "HidD_GetManufacturerString");
        return -1;
    }

    return 0;
}

int HID_API_EXPORT_CALL HID_API_CALL hid_get_product_string(hid_device *dev, wchar_t *string, size_t maxlen)
{
    BOOL res;

    res = HidD_GetProductString(dev->device_handle, string, sizeof(wchar_t) * MIN(maxlen, MAX_STRING_WCHARS));
    if (!res) {
        register_error(dev, "HidD_GetProductString");
        return -1;
    }

    return 0;
}

int HID_API_EXPORT_CALL HID_API_CALL hid_get_serial_number_string(hid_device *dev, wchar_t *string, size_t maxlen)
{
    BOOL res;

    res = HidD_GetSerialNumberString(dev->device_handle, string, sizeof(wchar_t) * MIN(maxlen, MAX_STRING_WCHARS));
    if (!res) {
        register_error(dev, "HidD_GetSerialNumberString");
        return -1;
    }

    return 0;
}

int HID_API_EXPORT_CALL HID_API_CALL hid_get_indexed_string(hid_device *dev, int string_index, wchar_t *string, size_t maxlen)
{
    BOOL res;

    res = HidD_GetIndexedString(dev->device_handle, string_index, string, sizeof(wchar_t) * MIN(maxlen, MAX_STRING_WCHARS));
    if (!res) {
        register_error(dev, "HidD_GetIndexedString");
        return -1;
    }

    return 0;
}


HID_API_EXPORT const wchar_t * HID_API_CALL  hid_error(hid_device *dev)
{
    return (wchar_t*)dev->last_error_str;
}


/*#define PICPGM*/
/*#define S11*/
#define P32
#ifdef S11
  unsigned short VendorID = 0xa0a0;
    unsigned short ProductID = 0x0001;
#endif

#ifdef P32
  unsigned short VendorID = 0x04d8;
    unsigned short ProductID = 0x3f;
#endif


#ifdef PICPGM
  unsigned short VendorID = 0x04d8;
  unsigned short ProductID = 0x0033;
#endif


#if 0
int __cdecl main(int argc, char* argv[])
{
    int res;
    unsigned char buf[65];

    UNREFERENCED_PARAMETER(argc);
    UNREFERENCED_PARAMETER(argv);

    /* Set up the command buffer. */
    memset(buf,0x00,sizeof(buf));
    buf[0] = 0;
    buf[1] = 0x81;


    /* Open the device. */
    int handle = open(VendorID, ProductID, L"12345");
    if (handle < 0)
        printf("unable to open device\n");


    /* Toggle LED (cmd 0x80) */
    buf[1] = 0x80;
    res = write(handle, buf, 65);
    if (res < 0)
        printf("Unable to write()\n");

    /* Request state (cmd 0x81) */
    buf[1] = 0x81;
    write(handle, buf, 65);
    if (res < 0)
        printf("Unable to write() (2)\n");

    /* Read requested state */
    read(handle, buf, 65);
    if (res < 0)
        printf("Unable to read()\n");

    /* Print out the returned buffer. */
    for (int i = 0; i < 4; i++)
        printf("buf[%d]: %d\n", i, buf[i]);

    return 0;
}
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
