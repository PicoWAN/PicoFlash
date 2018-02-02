/*
 * PicoFlash - A command-line tool to flash a PicoTAG firmware
 *
 * Copyright (c) 2017, Archos S.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef __APPLE__
#include <machine/endian.h>
#include <libkern/OSByteOrder.h>

#define htobe16(x) OSSwapHostToBigInt16(x)
#define htole16(x) OSSwapHostToLittleInt16(x)
#define be16toh(x) OSSwapBigToHostInt16(x)
#define le16toh(x) OSSwapLittleToHostInt16(x)

#define htobe32(x) OSSwapHostToBigInt32(x)
#define htole32(x) OSSwapHostToLittleInt32(x)
#define be32toh(x) OSSwapBigToHostInt32(x)
#define le32toh(x) OSSwapLittleToHostInt32(x)

#define htobe64(x) OSSwapHostToBigInt64(x)
#define htole64(x) OSSwapHostToLittleInt64(x)
#define be64toh(x) OSSwapBigToHostInt64(x)
#define le64toh(x) OSSwapLittleToHostInt64(x)

#define __BIG_ENDIAN    BIG_ENDIAN
#define __LITTLE_ENDIAN LITTLE_ENDIAN
#define __BYTE_ORDER    BYTE_ORDER
#else
#include <endian.h>
#endif

#define TTY_SPEED			B115200

#define UPDATE_CHECKSUM_SIZE		4 //4 bytes checksum

#define WAIT_FOR_BLOCK_MSG		"Wait for block"
#define BLOCK_SIZE_MSG			"Block size"
#define INVALID_CHECKSUM_MSG		"Invalid checksum"
#define WELCOME_MSG			"PicoBoot Version"
#define UPDATE_COMPLETE_MSG		"Firmware update complete"


static int set_interface_attribs(int fd, int speed, int parity) {
	struct termios tty;

	memset(&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
		fprintf(stderr, "Cannot get attr: %d\n", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	// 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;		// disable break processing
	tty.c_iflag &= ~ICRNL;		// disable CR to NL
	tty.c_lflag = 0;		// no signaling chars, no echo,
					// no canonical processing
	tty.c_oflag = 0;		// no remapping, no delays

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);	// shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls,
						// enable reading
	tty.c_cflag &= ~CRTSCTS;		// disable flow control
	tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~HUPCL;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "Cannot set attr: %d\n", errno);
		return -1;
	}

	return 0;
}

static int set_blocking(int fd, int should_block) {
	struct termios tty;

	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		fprintf(stderr, "Cannot get attr: %d\n", errno);
		return -1;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;	// 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "Cannot set attr: %d\n", errno);
		return -1;
	}

	return 0;
}

static void usage(char * name) {
	fprintf(stdout, "Usage:\n\n");
	fprintf(stdout, "   %s <tty_device> <binary_firmware>\n\n", name);
}

static char * wait_for_line(FILE *f, char *buf, unsigned int buf_size, unsigned int timeout_sec) {
	char *start;
	fd_set rfds;
	struct timeval tv;

	FD_ZERO(&rfds);
	FD_SET(fileno(f), &rfds);

	if (timeout_sec > 0) {
		tv.tv_sec = timeout_sec;
		tv.tv_usec = 0;

		if (select(fileno(f) + 1, &rfds, NULL, NULL, &tv) <= 0) {
			fprintf(stderr, "Oops: timeout reached\n");
			return NULL;
		}
	}

	if (!fgets(buf, buf_size, f)) {
		fprintf(stderr, "Oops: fgets error (%s)\n", strerror(errno));
		return NULL;
	}

	/* Trim leading \0 and trailing \n\r */
	start = buf;
	while (*start == '\0') {
		start++;
	}

	if (start[strnlen(start, buf_size) - 1] == '\n') {
		buf[strnlen(start, buf_size) - 1] = '\0';
	}
	if (start[strnlen(start, buf_size) - 1] == '\r') {
		start[strnlen(start, buf_size) - 1] = '\0';
	}

	return start;
}

static uint32_t compute_checksum(unsigned char *buffer, unsigned int size) {
	unsigned int i;
	uint32_t cks = 0;

	for (i = 0; i < size; i++) {
		cks += buffer[i];
	}

	return cks;
}

static void reboot_accessory(int fd) {
#ifdef __APPLE__
	char zero = 0;

	/* The Apple driver does not support breaks, so  send a very slow 0x00
	 * instead, and wait to be sure it has been sent before changing back
	 * the serial speed (~15 ms @ 600 bps).
	 */
	set_interface_attribs(fd, B600, 0);
	write(fd, &zero, 1);
	usleep(15000);
	set_interface_attribs(fd, TTY_SPEED, 0);
#else
	tcsendbreak(fd, 0);
#endif
}

int main(int argc, char **argv) {
	int fd_tty;
	FILE *f_tty = NULL, *f_fw = NULL;
	char line_buf[1024];
	char *line_begin;
	float boot_version_f = 0;
	unsigned int boot_version = 0;
	unsigned int block_size;
	unsigned int read_bytes;
	unsigned char *block_buffer;
	uint32_t checksum;
	unsigned int fw_size;
	int ret = 0;

	if (argc < 3) {
		usage(argv[0]);
		return -1;
	}

	f_fw = fopen(argv[2], "r");
	if (f_fw == NULL) {
		fprintf(stderr, "Failed to open %s: %s\n", argv[2], strerror (errno));
		return -1;
	}

	fseek(f_fw, 0, SEEK_END);

	/* If the size is not a multiple of 4, the file will be padded with 0xFF */
	fw_size = (ftell(f_fw) + 3) & ~(0x3);

	fseek(f_fw, 0, SEEK_SET);

	fd_tty = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
	if (fd_tty < 0) {
		fprintf(stderr, "Failed to open %s: %s\n", argv[1], strerror (errno));
		ret = -1;
		goto out;
	}

	f_tty = fdopen(fd_tty, "r+");
	if (f_tty == NULL) {
		fprintf(stderr, "Failed to fdopen %s: %s\n", argv[1], strerror (errno));
		ret = -1;
		goto out;
	}

	set_interface_attribs(fd_tty, TTY_SPEED, 0);	// set speed to 115200 bps, 8n1 (no parity)
	set_blocking(fd_tty, 0);			// set not-blocking

	/* Reboot the tag to stop the flow of data */
	reboot_accessory(fd_tty);

	/* Be sure we start from a sane point (for
	 * some reason, tcflush is not enough)
	 */
	tcflush(fd_tty, TCIOFLUSH);
	while (read(fd_tty, line_buf, 1) > 0);

	set_blocking(fd_tty, 1);			// set blocking

	f_tty = fdopen(fd_tty, "r+");
	if (f_tty == NULL) {
		fprintf(stderr, "Failed to fdopen %s: %s\n", argv[1], strerror (errno));
		ret = -1;
		goto out;
	}

	/* Reboot the tag once again */
	reboot_accessory(fd_tty);

	fprintf(stdout, "\n< Waiting for bootloader ... >\n");

	/* Detect the bootloader's welcome message */
	while ((line_begin = wait_for_line(f_tty, line_buf, 1024, 0)) != NULL) {
		if (sscanf(line_begin, WELCOME_MSG" %f", &boot_version_f) == 1) {
			boot_version = (unsigned int) (boot_version_f * 10);
			fprintf(stdout, "\nPicoBoot version %.1f detected !\n", boot_version/10.0f);
			break;
		}
	}

	if (line_begin == NULL) {
		/* Something went wrong */
		fprintf(stderr, "Unable to detect PicoBoot !\n");
		ret = -1;
		goto out;
	}

	/* Interrupt the boot sequence */
	fputc('\n', f_tty);
	fflush(f_tty);

	usleep(100000);

	fprintf(f_tty, "UPDATE=%u\n", fw_size);

	fflush(f_tty);

	/* Retrieve the expected block size */
	while ((line_begin = wait_for_line(f_tty, line_buf, 1024, 2)) != NULL) {
		if (sscanf(line_begin, BLOCK_SIZE_MSG" %u", &block_size) == 1) {
			block_size -= UPDATE_CHECKSUM_SIZE;
			fprintf(stdout, "Expected block size is %u bytes\n", block_size);
			break;
		}
	}

	if (line_begin == NULL) {
		/* Something went wrong */
		fprintf(stderr, "Unable to retrieve the block size !\n");
		ret = -1;
		goto out;
	}

	block_buffer = (unsigned char *)malloc(block_size);
	if (block_buffer == NULL) {
		/* Something went wrong */
		fprintf(stderr, "Out of memory !\n");
		ret = -1;
		goto out;
	}

	fprintf(stdout, "\nUploading \"%s\" (%u bytes) ...\n", argv[2], fw_size);

	/* Upload loop */
	while (1) {
		/* Read a block */
		if ((read_bytes = fread(block_buffer, 1, block_size, f_fw)) == 0) {
			if (feof(f_fw)) {
				/* Everything has been uploaded */
				break;
			}

			/* Something went wrong */
			fprintf(stderr, "Failed to read from %s: %s\n", argv[2], strerror (errno));
			ret = -1;
			goto out;
		}

		/* Padding with 0xFF to get a multiple of 4 bytes */
		while (read_bytes % 4 != 0)  {
			block_buffer[read_bytes] = 0xFF;
			read_bytes++;
		}

		checksum = htole32(compute_checksum(block_buffer, read_bytes));

		/* Wait for the bootloader to be ready to receive a block message */
		while ((line_begin = wait_for_line(f_tty, line_buf, 1024, 5)) != NULL) {
			if (strncmp(line_begin, WAIT_FOR_BLOCK_MSG, strlen(WAIT_FOR_BLOCK_MSG)) == 0) {
				break;
			} else if (strncmp(line_begin, INVALID_CHECKSUM_MSG, strlen(INVALID_CHECKSUM_MSG)) == 0) {
				fprintf(stderr, "Bootloader reported an invalid checksum !\n");
				ret = -1;
				goto out;
			}
		}

		if (line_begin == NULL) {
			/* Something went wrong */
			fprintf(stderr, "Unable to sync with the bootloader !\n");
			ret = -1;
			goto out;
		}

		fprintf(stdout, "   Sending block (%u bytes) ...\n", read_bytes);

		/* Upload a block */
		if (fwrite(block_buffer, 1, read_bytes, f_tty) != read_bytes) {
			/* Something went wrong */
			fprintf(stderr, "Failed to write to %s: %s\n", argv[1], strerror (errno));
			ret = -1;
			goto out;
		}

		/* Upload the checksum */
		if (fwrite((unsigned char *) &checksum, 4, 1, f_tty) != 1) {
			/* Something went wrong */
			fprintf(stderr, "Failed to write to %s: %s\n", argv[1], strerror (errno));
			ret = -1;
			goto out;
		}

		/* Be sure everything has been flushed */
		fflush(f_tty);

		if (read_bytes != block_size) {
			/* Assume end of file */
			break;
		}
	}

	/* Be sure the last block has been properly received */
	while ((line_begin = wait_for_line(f_tty, line_buf, 1024, 5)) != NULL) {
		if (strncmp(line_begin, UPDATE_COMPLETE_MSG, strlen(UPDATE_COMPLETE_MSG)) == 0) {
			break;
		} else if (strncmp(line_begin, INVALID_CHECKSUM_MSG, strlen(INVALID_CHECKSUM_MSG)) == 0) {
			fprintf(stderr, "Bootloader reported an invalid checksum !\n");
			ret = -1;
			goto out;
		}
	}

	fprintf(stdout, "\nUpdate successful !\n");

out:
	if (f_tty != NULL) {
		fclose(f_tty);
	}
	if (f_fw != NULL) {
		fclose(f_fw);
	}

	fprintf(stdout, "\n");

	return ret;
}
