/*
 * =====================================================================================
 *
 *       Filename:  main.c
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  西元2020年07月30日 02時46分18秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lai Liang-Wei (github/william6849), william68497@gmail.com
 *   Organization:
 *
 * =====================================================================================
 */

#include <fcntl.h>
#include <linux/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>


int main(int argc, char *argv[])
{
    int dd;

    if (argc < 2) {
        printf("Usage:\n\t%s /dev/<raw dev name>\n", argv[0]);
        return 1;
    }

    if ((dd = open("/dev/sraids", O_RDWR)) < 0) {
        printf("error: open /dev/sraids: %m");
        return dd;
    }

    printf("do it... <%s>\n", argv[1]);
    if (ioctl(dd, 0, argv[1]) < 0) {
        fprintf(stderr, "Kernel call returned: %m");
        return 1;
    }
    printf("OK\n");

    return 0;
}
