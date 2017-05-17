/*
 *  COPYRIGHT (c) 1989-2012.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <rtems/test.h>
#include <stdio.h>
#include <bsp/i2c.h>
#include <libcpu/am335x.h>
#include <rtems.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <rtems/score/assert.h>
#include <dev/i2c/eeprom.h>

#include <rtems/shell.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <rtems/userenv.h>

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_STUB_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_ZERO_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK

#define CONFIGURE_USE_IMFS_AS_BASE_FILESYSTEM

#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32

#define CONFIGURE_MAXIMUM_USER_EXTENSIONS 1

#define CONFIGURE_UNLIMITED_ALLOCATION_SIZE 32
#define CONFIGURE_UNLIMITED_OBJECTS
#define CONFIGURE_UNIFIED_WORK_AREAS

#define CONFIGURE_STACK_CHECKER_ENABLED

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT_TASK_INITIAL_MODES RTEMS_DEFAULT_MODES
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT

#define CONFIGURE_INIT

#include <rtems/confdefs.h>


#define CONFIGURE_SHELL_COMMAND_CP
#define CONFIGURE_SHELL_COMMAND_PWD
#define CONFIGURE_SHELL_COMMAND_LS
#define CONFIGURE_SHELL_COMMAND_LN
#define CONFIGURE_SHELL_COMMAND_LSOF
#define CONFIGURE_SHELL_COMMAND_CHDIR
#define CONFIGURE_SHELL_COMMAND_CD
#define CONFIGURE_SHELL_COMMAND_MKDIR
#define CONFIGURE_SHELL_COMMAND_RMDIR
#define CONFIGURE_SHELL_COMMAND_CAT
#define CONFIGURE_SHELL_COMMAND_MV
#define CONFIGURE_SHELL_COMMAND_RM
#define CONFIGURE_SHELL_COMMAND_MALLOC_INFO
#include <rtems/shellconfig.h>


/* I2C address of CAT24C256 eeprom
   EEPROM SIZE 32 KB Ref: BBB SRM */
#define I2C_SLAVE_ADDR         (0x50)
#define EEPROM_SIZE 78
#define EEPROM_PATH "/dev/i2c-0.eeprom"

/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument argument);

const char rtems_test_name[] = "GSOC 2016 I2C TESTING";
rtems_printer rtems_test_printer;


rtems_task Init(
  rtems_task_argument ignored
)
{
  rtems_test_begin();
  int i;
  int rv,fd_bus,fd_in_dev;
  uint8_t in[EEPROM_SIZE];
  struct stat st;
  off_t off;
  ssize_t n;

  rtems_shell_env_t env;

  

  /*bus registration */
  rv = bbb_register_i2c_0();
 
  // bus_PATH, i2c_base, clock_speed, irq
  fd_bus = open(BBB_I2C_0_BUS_PATH, O_RDWR);
 printf("open bus \n");
  //assert(fd_bus >= 0);
  printf("register EEPROM \n");
  /* I2C EEPROM registration */
  rv = i2c_dev_register_eeprom(
      BBB_I2C_0_BUS_PATH, // bus path
      EEPROM_PATH, //dev path
      I2C_SLAVE_ADDR, // slave addr
      2, //address_byte
      64, // page_size_in_bytes
      256, // size_in_bytes
      0 // program time out in ms
     );
  
 

 _Assert(rv == 0);
// exit( 0 );

  fd_in_dev = open(EEPROM_PATH, O_RDWR);
 printf("fd_in_dev:%d\n", fd_in_dev);
  _Assert(fd_in_dev >=0);

  printf("open eeprom \n");

  rv = fstat(fd_in_dev, &st);
  _Assert(rv == 0);
  _Assert(st.st_blksize == 8);
  _Assert(st.st_size == sizeof(in));


printf("read func\n");
  n = read(fd_in_dev, &in[0], sizeof(in));

printf("The EEPROM INFO:\n");
 for(i=0;i<sizeof(in);++i)
   {
printf(" %x\n,",in[i]);

   }

    
  printf("EXIT from test case\n");
 // close(fd_bus);
// unlink(BBB_I2C_2_BUS_PATH);
  rtems_test_end();
}

#define CONFIGURE_MICROSECONDS_PER_TICK 2000
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 7

#define CONFIGURE_MAXIMUM_TASKS            1
#define CONFIGURE_FILESYSTEM_IMFS

#define CONFIGURE_MAXIMUM_SEMAPHORES    1

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE 

#define CONFIGURE_INIT_TASK_STACK_SIZE (RTEMS_MINIMUM_STACK_SIZE + 2 * EEPROM_SIZE)

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
