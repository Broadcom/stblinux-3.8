/*---------------------------------------------------------------------------

    Broadcom                                                     /\
    Connecting everything(R)                              _     /  \     _
    _____________________________________________________/ \   /    \   / \_
                                                            \_/      \_/

 Copyright (c) 2007-2014 Broadcom Corporation
 All rights reserved.

 Redistribution and use of this software in source and binary forms, with or
 without modification, are permitted provided that the following conditions
 are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 * Neither the name of Broadcom Corporation nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission of Broadcom Corporation.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 File: pmlib.c

 Description:
 Power management API for Broadcom STB/DTV peripherals

 ------------------------------------------------------------------------- */

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <glob.h>
#include <libgen.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/sockios.h>
#include <linux/ethtool.h>
#include <linux/if.h>

#include <pmlib.h>

struct brcm_pm_priv {
	struct brcm_pm_state last_state;
	struct brcm_pm_cfg cfg;
	int has_eth1;
};

#define BUF_SIZE	64
#define MAX_ARGS	16

#define SYS_SATA_STAT	"/sys/devices/platform/brcmstb/sata_power"
#define SYS_SRPD_GLOB	"/sys/bus/platform/drivers/brcmstb_memc/*/srpd"
#define SYS_TP1_STAT	"/sys/devices/system/cpu/cpu1/online"
#define SYS_TP2_STAT	"/sys/devices/system/cpu/cpu2/online"
#define SYS_TP3_STAT	"/sys/devices/system/cpu/cpu3/online"
#define SYS_CPUFREQ_CUR_FREQ "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq"
#define SYS_CPUFREQ_SETSPEED "/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed"
#define SYS_CPUFREQ_AVAIL "/sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies"
#define SYS_CPUFREQ_GOV	"/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
#define SYS_STANDBY	"/sys/power/state"
#define HALT_PATH	"/sbin/halt"
#define AHCI_DEV_NAME_GLOB	"strict-ahci.*"
#define SATA_SCSI_DEVICE "/sys/devices/platform/" AHCI_DEV_NAME_GLOB "/ata*/host*/target*/*/scsi_device/*/device"
#define SATA_DELETE_GLOB SATA_SCSI_DEVICE "/delete"
#define SATA_RESCAN_GLOB "/sys/class/scsi_host/host*/scan"
#define AHCI_PATH "/sys/bus/platform/drivers/ahci"
#define SATA_DEVICE_PATH "/sys/bus/platform/devices/" AHCI_DEV_NAME_GLOB
#define SATA_UNBIND_PATH AHCI_PATH "/unbind"
#define SATA_BIND_PATH	AHCI_PATH "/bind"

static int file_exists(const char *path)
{
	return !access(path, F_OK);
}

static int sysfs_get(const char *path, unsigned int *out)
{
	FILE *f;
	unsigned int tmp;
	char buf[BUF_SIZE];

	f = fopen(path, "r");
	if (!f)
		return -1;
	if (fgets(buf, BUF_SIZE, f) != buf) {
		fclose(f);
		return -1;
	}
	fclose(f);
	if (sscanf(buf, "0x%x", &tmp) != 1 && sscanf(buf, "%u", &tmp) != 1)
		return -1;
	*out = tmp;
	return 0;
}

static int sysfs_set(const char *path, int in)
{
	FILE *f;
	char buf[BUF_SIZE];

	f = fopen(path, "w");
	if (!f)
		return -1;
	sprintf(buf, "%u", in);
	if ((fputs(buf, f) < 0) || (fflush(f) < 0)) {
		fclose(f);
		return -1;
	}
	fclose(f);
	return 0;
}

static int sysfs_set_string(const char *path, const char *in)
{
	FILE *f;

	f = fopen(path, "w");
	if (!f)
		return -1;
	if ((fputs(in, f) < 0) || (fflush(f) < 0)) {
		fclose(f);
		return -1;
	}
	fclose(f);
	return 0;
}

static int sysfs_get_string(const char *path, char *in, int size)
{
	FILE *f;
	size_t len;

	f = fopen(path, "r");
	if (!f)
		return -1;
	if (fgets(in, size, f) < 0) {
		fclose(f);
		return -1;
	}
	fclose(f);
	len = strnlen(in, size);
	if (in[len-1] == '\n')
		in[len-1] = '\0';
	return 0;
}

static int run(char *prog, ...)
{
	va_list ap;
	int status, i = 1;
	pid_t pid;
	char *args[MAX_ARGS], *a;

	va_start(ap, prog);

	pid = fork();
	if (pid < 0)
		return -1;

	if (pid != 0) {
		wait(&status);
		va_end(ap);
		return WEXITSTATUS(status) ? -1 : 0;
	}

	/* child */
	args[0] = prog;
	do {
		a = va_arg(ap, char *);
		args[i++] = a;
	} while (a);

	execv(prog, args);
	_exit(1);

	va_end(ap);		/* never reached */
	return 0;
}

static int brcm_pm_eth1_check(void)
{
	struct ifreq ifr;
	struct ethtool_drvinfo drvinfo;
	int ret = 0;
	int fd;

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
		return 0;

	memset(&ifr, 0, sizeof(ifr));
	memset(&drvinfo, 0, sizeof(drvinfo));

	drvinfo.cmd = ETHTOOL_GDRVINFO;
	ifr.ifr_data = (caddr_t)&drvinfo;
	strcpy(ifr.ifr_name, "eth1");

	if (ioctl(fd, SIOCETHTOOL, &ifr) == 0) {
		if (strcmp(drvinfo.driver, "BCMINTMAC") == 0)
			ret = 1;
		if (strcmp(drvinfo.driver, "BCMUNIMAC") == 0)
			ret = 1;
	}

	close(fd);

	return ret;
}

void *brcm_pm_init(void)
{
	struct brcm_pm_priv *ctx;

	ctx = (void *)malloc(sizeof(*ctx));
	if (!ctx)
		goto bad;

	/* this is the current PLL frequency going into the CPU */
	if (sysfs_get(SYS_CPUFREQ_CUR_FREQ,
		      (unsigned int *)&ctx->last_state.cpu_base) != 0) {
		/* cpufreq not supported on this platform */
		ctx->last_state.cpu_base = BRCM_PM_UNDEF;
	}

	if (brcm_pm_get_status(ctx, &ctx->last_state) != 0)
		goto bad_free;

	ctx->has_eth1 = brcm_pm_eth1_check();

	return ctx;

bad_free:
	free(ctx);
bad:
	return NULL;
}

void brcm_pm_close(void *vctx)
{
	free(vctx);
}

int brcm_pm_get_cfg(void *vctx, struct brcm_pm_cfg *cfg)
{
	struct brcm_pm_priv *ctx = vctx;

	*cfg = ctx->cfg;
	return 0;
}

int brcm_pm_set_cfg(void *vctx, struct brcm_pm_cfg *cfg)
{
	struct brcm_pm_priv *ctx = vctx;

	ctx->cfg = *cfg;
	return 0;
}

static int get_srpd_status(void)
{
	glob_t g;
	int i;
	unsigned int val = 0;

	if (glob(SYS_SRPD_GLOB, GLOB_NOSORT, NULL, &g) != 0)
		return BRCM_PM_UNDEF;

	for (i = 0; i < (int)g.gl_pathc; i++) {
		if (sysfs_get(g.gl_pathv[i], &val) != 0)
			return BRCM_PM_UNDEF;

		/* Stop with the first non-zero value */
		if (val > 0)
			break;
	}
	globfree(&g);

	return val;
}

static int set_srpd(int val)
{
	glob_t g;
	int i;

	if (val < 0)
		return 1;

	if (glob(SYS_SRPD_GLOB, GLOB_NOSORT, NULL, &g) != 0)
		return BRCM_PM_UNDEF;

	for (i = 0; i < (int)g.gl_pathc; i++) {
		if (sysfs_set(g.gl_pathv[i], val) != 0)
			return 1;
	}
	globfree(&g);

	return 0;
}

static int get_sata_status(void);

int brcm_pm_get_status(void *vctx, struct brcm_pm_state *st)
{
	struct brcm_pm_priv *ctx = vctx;

	/* read status from /proc */
	if (sysfs_get(SYS_TP1_STAT, (unsigned int *)&st->tp1_status) != 0)
		st->tp1_status = BRCM_PM_UNDEF;
	if (sysfs_get(SYS_TP2_STAT, (unsigned int *)&st->tp2_status) != 0)
		st->tp2_status = BRCM_PM_UNDEF;
	if (sysfs_get(SYS_TP3_STAT, (unsigned int *)&st->tp3_status) != 0)
		st->tp3_status = BRCM_PM_UNDEF;
	if (sysfs_get(SYS_CPUFREQ_CUR_FREQ, (unsigned int *)&st->cpu_base) != 0)
		st->cpu_base = BRCM_PM_UNDEF;
	if (sysfs_get(SYS_CPUFREQ_SETSPEED,
				(unsigned int *)&st->cpufreq_setspeed) != 0)
		st->cpufreq_setspeed = BRCM_PM_UNDEF;
	if (sysfs_get_string(SYS_CPUFREQ_AVAIL, st->cpufreq_avail,
			     CPUFREQ_AVAIL_MAXLEN) != 0)
		strcpy(st->cpufreq_avail, "");
	if (sysfs_get_string(SYS_CPUFREQ_GOV, st->cpufreq_gov,
			     CPUFREQ_GOV_MAXLEN) != 0)
		strcpy(st->cpufreq_gov, "");

	st->sata_status = get_sata_status();

	st->srpd_status = get_srpd_status();

	if (st != &ctx->last_state)
		memcpy(&ctx->last_state, st, sizeof(*st));

	return 0;
}

static int sata_rescan_hosts(void)
{
	glob_t g;
	int i, ret = 0;

	if (glob(SATA_RESCAN_GLOB, GLOB_NOSORT, NULL, &g) != 0)
		return -1;

	for (i = 0; i < (int)g.gl_pathc; i++)
		ret |= sysfs_set_string(g.gl_pathv[i], "0 - 0");
	globfree(&g);

	return ret;
}

static int sata_delete_devices(void)
{
	glob_t g;
	int i, ret = 0;

	if (glob(SATA_DELETE_GLOB, GLOB_NOSORT, NULL, &g) != 0)
		return 0;

	for (i = 0; i < (int)g.gl_pathc; i++)
		ret |= sysfs_set(g.gl_pathv[i], 1);

	globfree(&g);

	return ret;
}

static int get_sata_status(void)
{
	glob_t g;
	int i, is_on = 1;
	char *path;
	size_t len;

	if (glob(SATA_DEVICE_PATH, GLOB_NOSORT, NULL, &g) != 0)
		/* No AHCI devices present? */
		return BRCM_PM_UNDEF;

	/* Allocate at least space for AHCI_PATH/strict-ahci.XX */
	len = strlen(AHCI_PATH) + strlen(AHCI_DEV_NAME_GLOB) + 10;
	path = calloc(len, sizeof(*path));

	/* Check if any strict-ahci.* devices are present at AHCI_PATH */
	for (i = 0; i < (int)g.gl_pathc; i++) {
		strncat(path, AHCI_PATH "/", len - 1);
		strncat(path, basename(g.gl_pathv[i]), len - 1);
		if (!file_exists(path)) {
			/* At least one device is unbound */
			is_on = 0;
			goto out;
		}
	}

out:
	globfree(&g);
	free(path);

	return is_on;
}

static int sata_set_power(int on)
{
	glob_t g;
	int i, ret = 0;
	const char *dest = on ? SATA_BIND_PATH : SATA_UNBIND_PATH;

	if (glob(SATA_DEVICE_PATH, GLOB_NOSORT, NULL, &g) != 0)
		return 0;

	for (i = 0; i < (int)g.gl_pathc; i++)
		ret |= sysfs_set_string(dest, basename(g.gl_pathv[i]));

	globfree(&g);

	return ret;
}

int brcm_pm_set_status(void *vctx, struct brcm_pm_state *st)
{
	struct brcm_pm_priv *ctx = vctx;
	int ret = 0;

#define CHANGED(element) \
	((st->element != BRCM_PM_UNDEF) && \
	 (st->element != ctx->last_state.element))

#define CHANGED_STRING(element, maxlen) \
	((st->element && ctx->last_state.element) && \
	 (strncmp(st->element, ctx->last_state.element, maxlen)) != 0)

	if (CHANGED(sata_status)) {
		if (st->sata_status) {
			ret |= sata_set_power(1);
			ret |= sata_rescan_hosts();
		} else {
			/* Remove SCSI devices, triggering HDD spin-down */
			ret |= sata_delete_devices();
			/* Small delay before yanking the device entirely */
			usleep(100000);
			ret |= sata_set_power(0);
		}
		ctx->last_state.sata_status = st->sata_status;
	}

	if (CHANGED(tp1_status))
		ret |= sysfs_set(SYS_TP1_STAT, st->tp1_status);

	if (CHANGED(tp2_status))
		ret |= sysfs_set(SYS_TP2_STAT, st->tp2_status);

	if (CHANGED(tp3_status))
		ret |= sysfs_set(SYS_TP3_STAT, st->tp3_status);

	if (CHANGED_STRING(cpufreq_gov, CPUFREQ_GOV_MAXLEN))
		ret |= sysfs_set_string(SYS_CPUFREQ_GOV, st->cpufreq_gov);

	if (CHANGED(cpufreq_setspeed))
		ret |= sysfs_set(SYS_CPUFREQ_SETSPEED, st->cpufreq_setspeed);

	if (CHANGED(srpd_status))
		ret |= set_srpd(st->srpd_status);

#undef CHANGED
#undef CHANGED_STRING

	return ret;
}

int brcm_pm_suspend(void *vctx, int suspend_mode)
{
	if (suspend_mode == BRCM_PM_STANDBY)
		return sysfs_set_string(SYS_STANDBY, "standby");
	if (suspend_mode == BRCM_PM_SUSPEND)
		return sysfs_set_string(SYS_STANDBY, "mem");
	if (suspend_mode == BRCM_PM_HIBERNATE)
		return sysfs_set_string(SYS_STANDBY, "disk");
	if (suspend_mode == BRCM_PM_IRW_HALT)
		return run(HALT_PATH, NULL);
	return -1;
}
