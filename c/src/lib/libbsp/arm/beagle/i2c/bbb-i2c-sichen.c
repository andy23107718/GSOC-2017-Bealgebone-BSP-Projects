/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBoard I2C bus initialization and API Support.
 */

/*
 * Copyright (c) 2016 Punit Vara <punitvara at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include <bsp/i2c.h>
#include <libcpu/am335x.h>
#include <rtems/irq-extension.h>
#include <bsp/bbb-gpio.h>
#include <rtems/score/assert.h>

#define u16 unsigned int

static int am335x_i2c_set_clock(i2c_bus *base, unsigned long clock);
static void omap24_i2c_init(i2c_bus *base);
static void flush_fifo(i2c_bus *base);
static int wait_for_bb(i2c_bus *base);
static int omap24_i2c_probe(i2c_bus *base);
static u16 wait_for_event(i2c_bus *base);
static int am335x_i2c_read(i2c_bus *base, unsigned char chip, uint addr, int alen, unsigned char *buffer, 
                           int len);
static int read_eeprom(i2c_bus *base,struct am335x_baseboard_id *header);
static int am335x_i2c_write(i2c_bus *base, unsigned char chip, uint addr,int alen, unsigned char *buffer, 
                            int len);
/*
static bool am335x_i2c_pinmux(bbb_i2c_bus *bus)
{
  bool status =true;
    // We will check i2c_bus_id in am335x_i2c_bus_register
    // Apart from mode and pull_up register what about SCREWCTRL & RXACTIVE ??
  if (bus->i2c_bus_id == I2C1) {
    REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_CS0) = (BBB_MUXMODE(MODE2) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D1) = (BBB_MUXMODE(MODE2) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_TXD) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_RXD) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
  } else if (bus->i2c_bus_id == I2C2) {
    REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_RTSN) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_CTSN) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D0) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_SCLK) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
  } else {
  status = false;  
  }
  return status;   
}
*/

/* ref. Table 21-4 I2C Clock Signals */
/* 
 For I2C1/2
 Interface clock - 100MHz - CORE_LKOUTM4 / 2 - pd_per_l4ls_gclk
 Functional clock - 48MHz - PER_CLKOUTM2 / 4 - pd_per_ic2_fclk
*/

/*
static void am335x_i2c1_i2c2_module_clk_config(bbb_i2c_bus *bus)
{
*/
/*0x2 = SW_WKUP : SW_WKUP: Start a software forced wake-up
transition on the domain. */
/*
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) |=
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
  while((REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) &
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL) !=
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP);
*/

/* 0x2 = ENABLE : Module is explicitly enabled. Interface clock (if not
used for functions) may be gated according to the clock domain
state. Functional clocks are guarantied to stay present. As long as in
this configuration, power domain sleep transition cannot happen.*/
 /* REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKCTRL) |=
                        AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;
  while((REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKCTRL) &
      AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE) != AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE);
*/
/*0x2 = ENABLE : Module is explicitly enabled. Interface clock (if not
used for functions) may be gated according to the clock domain
state. Functional clocks are guarantied to stay present. As long as in
this configuration, power domain sleep transition cannot happen.*/
/*
  if (bus->i2c_bus_id == I2C1) {
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C1_CLKCTRL) |=
                             AM335X_CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE;
  while(REG((AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C1_CLKCTRL) &
     AM335X_CM_PER_I2C1_CLKCTRL_MODULEMODE) != AM335X_CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE);
  } else if (bus->i2c_bus_id == I2C2) {
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C2_CLKCTRL) |=
                             AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE_ENABLE;
  while(REG((AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C2_CLKCTRL) &
     AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE) != AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE_ENABLE);
  while(!(REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) &
           (AM335X_CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
            AM335X_CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_I2C_FCLK)));
  }
}
*/

static void am335x_i2c0_pinmux(bbb_i2c_bus *bus)
{
  printf("0x44e10000 + AM335X_CONF_I2C0_SDA:%x\n",0x44e10000 + AM335X_CONF_I2C0_SDA);
  printf("bus->regs:%x\n", bus->regs);
 
  REG(0x44e10000 + AM335X_CONF_I2C0_SDA) =
  (BBB_RXACTIVE | BBB_SLEWCTRL | BBB_PU_EN);

  REG(0x44e10000 + AM335X_CONF_I2C0_SCL) =
  (BBB_RXACTIVE | BBB_SLEWCTRL | BBB_PU_EN); 
}

static void I2C0ModuleClkConfig(void)
{
    /* Configuring L3 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) |=
          CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) |=
          CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) |=
          CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) |=
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) |=
          CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKTRCTRL));

    /* Checking fields for necessary values.  */

    /* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
    while((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)!=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)!=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
    ** attain the desired value.
    */
    while(CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) &
          CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));


    /* Configuring registers related to Wake-Up region. */

    /* Writing to MODULEMODE field of CM_WKUP_CONTROL_CLKCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
          CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
          CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_L3_AON_CLKSTCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) |=
          CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL));

    /* Writing to MODULEMODE field of CM_WKUP_I2C0_CLKCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) |=
          CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_MODULEMODE));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));

    /*
    ** Waiting for CLKACTIVITY_I2C0_GFCLK field in CM_WKUP_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_I2C0_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_I2C0_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_I2C0_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_IDLEST));
}

/*
void am335x_i2c_init(bbb_i2c_bus *bus, uint32_t input_clock)
{
  // am335x_i2c_pinmux()
  // am335x_i2c1_i2c2_module_clk_config
}
*/

static bool am335x_i2c_busbusy(volatile bbb_i2c_regs *regs)
{
  bool status;
  unsigned short stat;
  int timeout = I2C_TIMEOUT;

  REG(&regs->BBB_I2C_IRQSTATUS)=0xffff;
  printf("REG(&regs->BBB_I2C_IRQSTATUS_RAW):%x\n",REG(&regs->BBB_I2C_IRQSTATUS_RAW) );
 // printf("%x\n",0x1400 & 0x1000 );
 printf("REG(&regs->BBB_I2C_IRQSTATUS_RAW) & AM335X_I2C_IRQSTATUS_RAW_BB:%x\n",(REG(&regs->BBB_I2C_IRQSTATUS_RAW) & AM335X_I2C_IRQSTATUS_RAW_BB));
while(stat =( REG(&regs->BBB_I2C_IRQSTATUS_RAW) & AM335X_I2C_IRQSTATUS_RAW_BB) && timeout--)
  {

REG(&regs->BBB_I2C_IRQSTATUS)=stat;
    udelay(20);

  }

  if (timeout <= 0) {
    printf("Timed out in wait_for_bb: status=%04x\n",
           stat);
    return 1;
  }
  REG(&regs->BBB_I2C_IRQSTATUS)=0xffff;   /* clear delayed stuff*/
  return 0;

}

static void am335x_i2c_reset(bbb_i2c_bus *bus)
{
  volatile bbb_i2c_regs *regs = bus->regs;
  printk("reset bus->reg is %x \n",bus->regs);
  /* Disable I2C module at the time of initialization*/
  /*Should I use write32 ?? I guess mmio_clear is correct choice here*/
  REG(&regs->BBB_I2C_CON)=0x00;
  printk("inside BBB_I2C_CON value is %x \n",&regs->BBB_I2C_CON);
   REG(&regs->BBB_I2C_SYSC)= 0x2;
//  mmio_clear((&regs->BBB_I2C_CON),AM335X_I2C_CON_I2C_EN);

   REG(&regs->BBB_I2C_CON)= AM335X_I2C_CON_I2C_EN;

   while((REG(&regs->BBB_I2C_SYSS) &I2C_SYSS_RDONE)==0)  //wait reset done
   {
    udelay(100);

   }

   REG(&regs->BBB_I2C_CON)=0x00;

   am335x_i2c_set_clock(&bus->base, I2C_BUS_CLOCK_DEFAULT);

   REG(&regs->BBB_I2C_CON)= AM335X_I2C_CON_MST | AM335X_I2C_CON_I2C_EN;

//  mmio_clear((&regs->BBB_I2C_SYSC),AM335X_I2C_SYSC_AUTOIDLE); 

  //REG(bus->regs + AM335X_I2C_CON) &= ~(AM335X_I2C_CON_I2C_EN);
  //REG(bus->regs + AM335X_I2C_SYSC) &= ~(AM335X_I2C_SYSC_AUTOIDLE);
  
  /*
  can I clear all the interrupt here ?
  mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_IRQ_ENABLE_CLR), ??)
  */
}

/*
Possible values for msg->flag 
   * - @ref I2C_M_TEN,
   * - @ref I2C_M_RD,
   * - @ref I2C_M_STOP,
   * - @ref I2C_M_NOSTART,
   * - @ref I2C_M_REV_DIR_ADDR,
   * - @ref I2C_M_IGNORE_NAK,
   * - @ref I2C_M_NO_RD_ACK, and
   * - @ref I2C_M_RECV_LEN.
*/

static void am335x_i2c_set_address_size(const i2c_msg *msgs,volatile bbb_i2c_regs *regs)
{
    /*can be configured multiple modes here. Need to think about own address modes*/
  if ((msgs->flags & I2C_M_TEN) == 0)  {/* 7-bit mode slave address mode*/
  mmio_write(&regs->BBB_I2C_CON,(AM335X_I2C_CFG_7BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN)); 
  } else { /* 10-bit slave address mode*/
  mmio_write(&regs->BBB_I2C_CON,(AM335X_I2C_CFG_10BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN));
  }
  }

static void am335x_i2c_next_byte(bbb_i2c_bus *bus)
{
  i2c_msg *msg;
  
  printk("Enter next_byte\n");
  ++bus->msgs;
  --bus->msg_todo;

  msg = &bus->msgs[0];

  bus->current_msg_todo = msg->len;
  bus->current_msg_byte = msg->buf;
}

static unsigned int am335x_i2c_intrawstatus(volatile bbb_i2c_regs *regs)
{
  return (REG(&regs->BBB_I2C_IRQSTATUS_RAW));
}

static void am335x_i2c_masterint_enable(volatile bbb_i2c_regs *regs, unsigned int flag)
{
  printf("am335x_i2c_masterint_enable func\n");
  REG(&regs->BBB_I2C_IRQENABLE_SET) |= flag;
}

static void am335x_i2c_masterint_disable(volatile bbb_i2c_regs *regs, unsigned int flag)
{
 REG(&regs->BBB_I2C_IRQENABLE_CLR) = flag;
}

static void am335x_int_clear(volatile bbb_i2c_regs *regs, unsigned int flag)
{
  REG(&regs->BBB_I2C_IRQSTATUS) = flag;
}


static void am335x_clean_interrupts(volatile bbb_i2c_regs *regs)
{
  printf("am335x_clean_interrupts func\n");
  am335x_i2c_masterint_enable(regs,0x7FFF);
  am335x_int_clear(regs,0x7FFF);
  am335x_i2c_masterint_disable(regs,0x7FFF); 
}


static void am335x_i2c_setup_read_transfer(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs, const i2c_msg *msgs, bool send_stop)
{ 
  volatile unsigned int no_bytes;
  //am335x_i2c_masterint_enable(regs, AM335X_I2C_INT_RECV_READY);
   // No of data to be transmitted at a time


//bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
//  volatile bbb_i2c_regs *regs = bus->regs;
  struct am335x_baseboard_id header;

  omap24_i2c_probe(&bus->base);
   read_eeprom(&bus->base,&header);

/*
  REG(&regs->BBB_I2C_CNT) = 0x02;
  no_bytes = REG(&regs->BBB_I2C_CNT);

 // Set Slave address & Master enable, bring out of reset
  REG(&regs->BBB_I2C_SA) = msgs->addr;
  printf("slave address : %x\n",REG(&regs->BBB_I2C_SA));


  // I2C Controller in Master Mode
  REG(&regs->BBB_I2C_CON) = AM335X_I2C_CFG_MST_TX  | AM335X_I2C_CON_MST | AM335X_I2C_CON_START | AM335X_I2C_CON_I2C_EN | AM335X_I2C_CON_STOP;
  printk("set master in transmission mode %x \n",REG(&regs->BBB_I2C_CON));

while(am335x_i2c_busbusy(regs) != 0);
  printk("bus is free \n"); 


 

  // clear status of all interrupts
  am335x_clean_interrupts(regs);
  printk("\n set memory address to read\n");
    
  // transmit interrupt is enabled

  am335x_i2c_masterint_enable(regs,AM335X_I2C_IRQSTATUS_XRDY);
  printk("Enable transmit interrupt \n");

  

  //start condition 
  REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_START | AM335X_I2C_CON_I2C_EN;
  printk("start transmission \n");
 
  printk("CNT : %x\n", no_bytes);
  printf("BBB_I2C_DATA:%x\n", readb(&regs->BBB_I2C_DATA));

writeb(0x5,&regs->BBB_I2C_DATA);
printf("(0x50 >> 8)& 0xff:%x\n",(0x50 >> 8)& 0xff);
printf("REG(&regs->BBB_I2C_DATA):%x\n",readb(&regs->BBB_I2C_DATA) );
printf("&regs->BBB_I2C_DATA:%x\n",&regs->BBB_I2C_DATA);
//REG(&regs->BBB_I2C_IRQSTATUS)=AM335X_I2C_IRQSTATUS_XRDY;

udelay(10);

writeb(0x0,&regs->BBB_I2C_DATA);
//REG(&regs->BBB_I2C_DATA)= ( (0x50 >> 0)& 0xff);
printf("(0x50 >> 0)& 0xff:%x\n",(0x50 >> 0)& 0xff);
printf("REG(&regs->BBB_I2C_DATA):%x\n",readb(&regs->BBB_I2C_DATA) );

REG(&regs->BBB_I2C_IRQSTATUS)=AM335X_I2C_IRQSTATUS_XRDY;


 // no_bytes = REG(&regs->BBB_I2C_CNT);
  while(0 != no_bytes);
  printk("total msg count for tranmission is zero \n");
  while( !(am335x_i2c_intrawstatus(regs) & (AM335X_I2C_IRQSTATUS_ARDY)));
  
  printk("Enter read transfer \n");
   // No of data to be received at a time(msg_count!!)
  printk("msg_todo for read is %d \n",bus->msg_todo);
  REG(&regs->BBB_I2C_CNT) = bus->msg_todo;
  
  // clear status of all interrupts
  //am335x_clean_interrupts(regs);

  // I2C Controller in Master Mode
  REG(&regs->BBB_I2C_CON) = AM335X_I2C_CFG_MST_RX | AM335X_I2C_CON_I2C_EN | AM335X_I2C_CON_MST;
  printk("Set master to receiver mode %x \n", REG(&regs->BBB_I2C_CON));
  // receive interrupt is enabled
  am335x_i2c_masterint_enable(regs, AM335X_I2C_INT_RECV_READY | AM335X_I2C_INT_STOP_CONDITION);
  
  if (send_stop) {
    // stop condition
    printk("stop to read\n");
    REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_STOP; 
  } else {
    // start condition
    printk("start to read\n");
    REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_START;
  }
  while(am335x_i2c_busbusy(regs) == 0);
  */
  printk("Exit read transfer\n");
}


static void am335x_i2c_continue_read_transfer(
  bbb_i2c_bus *bus,
  volatile bbb_i2c_regs *regs
)
{
  printk("enter continue read transfer \n");
  bus->current_msg_byte[bus->already_transferred] = REG(&regs->BBB_I2C_DATA);
  bus->already_transferred++;
  am335x_int_clear(regs,AM335X_I2C_INT_RECV_READY);
  printk("clear RRDY in continue read transfer\n");
  
  if (bus->already_transferred == REG(&regs->BBB_I2C_CNT)) {
    printk("continue read transfer finished \n");
    //am335x_i2c_setup_read_transfer(bus,regs,false);
    am335x_i2c_masterint_disable(regs, AM335X_I2C_INT_RECV_READY);
    printk("disable RRDY in continue read transfer\n");
    REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_STOP;
    printk("stop condition in continue read transfer %x\n",REG(&regs->BBB_I2C_CON));
  }
}

static void am335x_i2c_continue_write(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs)
{ 
   REG(&regs->BBB_I2C_DATA) = 0x00;
   am335x_int_clear(regs, AM335X_I2C_IRQSTATUS_XRDY);
   printk("clear XRDY continue write\n");
   /*
   if (bus->already_transferred == REG(&regs->BBB_I2C_CNT)) {
   printk("\n finished transfer \n");
   am335x_i2c_masterint_disable(regs, AM335X_I2C_IRQSTATUS_XRDY);
   printk("disable XRDY continue write \n");
   REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_STOP;
   } else {
     printk("write memory address \n");
     REG(&regs->BBB_I2C_DATA) = *bus->current_msg_byte;
  }
   */

  /* 
   if (bus->already_transferred == bus->msg_todo) {
     printk("finished transfer \n");
     am335x_int_clear(regs, AM335X_I2C_IRQSTATUS_XRDY);
     REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_STOP;
   } else { 
     printk("remaining byte \n");
     REG(&regs->BBB_I2C_DATA) = bus->current_msg_byte[bus->already_transferred];
     printk("%s",REG(&regs->BBB_I2C_DATA));
     bus->already_transferred++;   
   }
   */
}

static void am335x_i2c_setup_write_transfer(bbb_i2c_bus *bus,volatile bbb_i2c_regs *regs)
{
  volatile unsigned int no_bytes; 
  printk(" \n Enter write transfer \n"); 
 
  // Following data count specify bytes to be transmitted
  REG(&regs->BBB_I2C_CNT) = bus->msg_todo;
  no_bytes = REG(&regs->BBB_I2C_CNT);
  // clear status of all interrupts
  // Already cleaned during reset
  am335x_clean_interrupts(regs);
  
  // I2C Controller in Master transmitter Mode
  REG(&regs->BBB_I2C_CON) = AM335X_I2C_CFG_MST_TX | AM335X_I2C_CON_I2C_EN;
  printk("enable master in transmiter mode setup write %x\n",REG(&regs->BBB_I2C_CON));
  
  // transmit interrupt is enabled
  am335x_i2c_masterint_enable(regs,AM335X_I2C_IRQSTATUS_XRDY);
  printk("enable XRDY setup write\n");
 
  //start condition 
  REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_START;
  printk("set start condition in setup write %x \n",REG(&regs->BBB_I2C_CON));
  
  while(am335x_i2c_busbusy(regs) == 0);
  printk("CNT in setup write : %x \n",REG(&regs->BBB_I2C_CNT));
  printk("setup write msg_todo %x \n",bus->current_todo);
  while(0 != no_bytes);
  printk("check whether ???\n");
  printk("RAW =  %x",REG(&regs->BBB_I2C_IRQSTATUS_RAW));
  while( !((am335x_i2c_intrawstatus(regs)) & (AM335X_I2C_IRQSTATUS_ARDY)));
  printk("exit setup write \n");
}


static void am335x_i2c_setup_transfer(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs)
{
  const i2c_msg *msgs = bus->msgs;
  uint32_t msg_todo = bus->msg_todo;
  bool send_stop = false;
  uint32_t i;

  printk("Enter setup transfer\n");
  bus->current_todo = msgs[0].len;

  for (i = 1; i < msg_todo && (msgs[i].flags & I2C_M_NOSTART) != 0; ++i) {
    bus->current_todo += msgs[i].len;
  }

  regs = bus->regs;
  printf("REG(&regs->BBB_I2C_DATA):%x\n",REG(&regs->BBB_I2C_DATA) );
 // REG(&bus->regs->BBB_I2C_BUF) |= AM335X_I2C_BUF_TXFIFO_CLR;
 // REG(&bus->regs->BBB_I2C_BUF) |= AM335X_I2C_BUF_RXFIFO_CLR;
printf("REG(&regs->BBB_I2C_DATA):%x\n",REG(&regs->BBB_I2C_DATA) );

 // am335x_i2c_set_address_size(msgs,regs);
bus->read = (msgs->flags & I2C_M_RD) != 0;

printf("bus->read:%d\n",bus->read );
  //bus->read = ((bus->read == true) ? 0:1); 
  bus->already_transferred = (bus->read == true) ? 0 : 1;

  if (bus->read) {
    if (REG(&regs->BBB_I2C_CNT) == 1) {
      send_stop = true;
    }
    printk("configure to read bus\n");
    am335x_i2c_setup_read_transfer(bus,regs,msgs,send_stop);
  } else {
    printk("configure to write bus\n");
    am335x_i2c_setup_write_transfer(bus,regs);
  }
  
}

static void am335x_i2c_interrupt(void *arg)
{
  bbb_i2c_bus *bus = arg;
  volatile bbb_i2c_regs *regs = bus->regs;
  /* get status of enabled interrupts */
  uint32_t irqstatus = REG(&regs->BBB_I2C_IRQSTATUS);
  bool done = false;
  printk("\n inside interrupt function \n");
  /* Clear all enabled interrupt except receive ready and transmit ready interrupt in status register */ 
  REG(&regs->BBB_I2C_IRQSTATUS) = (irqstatus & ~(AM335X_I2C_IRQSTATUS_RRDY | AM335X_I2C_IRQSTATUS_XRDY));
  printk("\n irqstatus = %x \n",REG(&regs->BBB_I2C_IRQSTATUS));

  if (irqstatus & AM335X_I2C_INT_RECV_READY) {
     printk("\nInside receive interrupt\n");
    am335x_i2c_continue_read_transfer(bus, regs);
  }
 
  if (irqstatus & AM335X_I2C_IRQSTATUS_XRDY) {
    printk("\ninside transmit interrupt \n");
    am335x_i2c_continue_write(bus,regs);
  }
 
  if (irqstatus & AM335X_I2C_IRQSTATUS_NACK) {
    done = true;
    printk("inside NACK\n");
    am335x_i2c_masterint_disable(regs,AM335X_I2C_IRQSTATUS_NACK);
  }

  if (irqstatus & AM335X_I2C_IRQSTATUS_BF) {
    done = true;
    printk("inside BF \n ");
  }

  if (done) {
    uint32_t err = irqstatus & BBB_I2C_IRQ_ERROR;
    printk("interrupt done \n");
   
    am335x_i2c_next_byte(bus);

    if (bus->msg_todo == 0 || err != 0) {
    rtems_status_code sc;
  
    // am335x_i2c_disable_interrupts(regs);
    am335x_i2c_masterint_disable(regs, (AM335X_I2C_IRQSTATUS_RRDY | AM335X_I2C_IRQSTATUS_XRDY | AM335X_I2C_IRQSTATUS_BF));

    REG(&regs->BBB_I2C_IRQSTATUS) = err;
  
    sc = rtems_event_transient_send(bus->task_id);
    _Assert(sc == RTEMS_SUCCESSFUL);
    (void) sc;
    } else {
      am335x_i2c_setup_transfer(bus, regs);
    }
  }
}

static int am335x_i2c_transfer(i2c_bus *base, i2c_msg *msgs, uint32_t msg_count)
{
  rtems_status_code sc;
  bbb_i2c_bus *bus = (bbb_i2c_bus *)base;
  volatile bbb_i2c_regs *regs;
  uint32_t i;
 printk("\n enter transfer\n ");
  rtems_task_wake_after(1);
  

  if (msg_count < 1){
    return 1;
  }
 
  for (i=0; i<msg_count;++i) {
      if ((msgs[i].flags & I2C_M_RECV_LEN) != 0) {
        return -EINVAL;
      }
  }
  printf("bus->regs:%x\n",bus->regs );
  bus->msgs = &msgs[0];
  bus->msg_todo = msg_count;
  printk("total msg = msg_count : %x \n",bus->msg_todo);
  bus->current_msg_todo = msgs[0].len;// current data size
  //bus->current_msg_byte = msgs[0].buf;// current data
  printk("\n current_msg_todo %x \n ",msgs[0].len);
  printk("\n current_msg_byte %x \n ",msgs[0].buf);
  //printf("8011A5CC:%x\n",  *(unsigned int *)(0x8011A5CC) );
  bus->task_id = rtems_task_self();

  regs = bus->regs;

 // REG(&regs->BBB_I2C_IRQENABLE_SET) = BBB_I2C_IRQ_USED;
  am335x_i2c_setup_transfer(bus,regs);
  
/*
  sc = rtems_event_transient_receive(RTEMS_WAIT, bus->base.timeout);
  // If timeout then return timeout error
  if (sc != RTEMS_SUCCESSFUL) {
    am335x_i2c_reset(bus);

    rtems_event_transient_clear();

    return -ETIMEDOUT;
  }
   */
  printk("exit transfer\n");

  // return bus->regs->BBB_I2C_IRQSTATUS == 0 ? 0 : -EIO;
  return 0;
}

static int am335x_i2c_set_clock(i2c_bus *base, unsigned long clock)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;
  uint32_t prescaler,divider;

  printk("set clock start\n"); 
  REG(&regs->BBB_I2C_CON)=0;

  prescaler = (BBB_I2C_SYSCLK / BBB_I2C_INTERNAL_CLK) -1;
  printf("prescaler:%d\n",prescaler );
  printk("PSC offset %x \n ",&regs->BBB_I2C_PSC);
  printk("PSC offset %x \n", &bus->regs->BBB_I2C_PSC);
  //mmio_write((&regs->BBB_I2C_PSC), prescaler);
  REG(&bus->regs->BBB_I2C_PSC) = prescaler;
  
  divider = BBB_I2C_INTERNAL_CLK/(2*clock);
  printf("divider:%d\n", divider);
  printk("SCLL offset %x \n",&bus->regs->BBB_I2C_SCLL); 
  //mmio_write((&regs->BBB_I2C_SCLL), (divider - 7));
  REG(&bus->regs->BBB_I2C_SCLL) = (divider - 7);
  //mmio_write((&regs->BBB_I2C_SCLH), (divider - 5));
  printk("SCHL offset %x\n",&bus->regs->BBB_I2C_SCLH);
  REG(&bus->regs->BBB_I2C_SCLH) = (divider - 5);

 REG(&regs->BBB_I2C_CON)=I2C_CON_EN;

writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);  /* clear all pending status */

  printk("set clock end \n");
  return 0;
}

static void am335x_i2c_destroy(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  rtems_status_code sc;
  printk(" starting destroy\n"); 
  sc = rtems_interrupt_handler_remove(bus->irq, am335x_i2c_interrupt, bus);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void)sc;
  printk("end destroy\n");
  i2c_bus_destroy_and_free(&bus->base);
}



int am335x_i2c_bus_register(
  const char *bus_path,
  uintptr_t register_base,
  uint32_t input_clock,
  rtems_vector_number irq
)
{
  
  bbb_i2c_bus *bus;
  rtems_status_code sc;
  int err;
  /*check bus number is >0 & <MAX*/

  bus = (bbb_i2c_bus *) i2c_bus_alloc_and_init(sizeof(*bus));
  
  if (bus == NULL) {
    return -1;
  }

  bus->regs = (volatile bbb_i2c_regs *) register_base;
 
// 1. Enable clock for I2CX
 I2C0ModuleClkConfig();
// 2. pinmux setup
  am335x_i2c0_pinmux(bus);
// 3. RESET : Disable Master, autoideal 
 // am335x_i2c_reset(bus);
// 4. configure bus speed  
  bus->input_clock = input_clock; // By default 100KHz. Normally pass 100KHz as argument 
 
/*
  printk("Before set clock \n"); 
  err = am335x_i2c_set_clock(&bus->base, I2C_BUS_CLOCK_DEFAULT);
 
  if (err != 0) {
    (*bus->base.destroy)(&bus->base);
    
    rtems_set_errno_and_return_minus_one(-err);
  }
   bus->irq = irq;
  */
omap24_i2c_init(&bus->base);

  //bring I2C out of reset

//printf("REG(&regs->BBB_I2C_IRQSTATUS):%x\n",REG(&bus->regs->BBB_I2C_IRQSTATUS));
  // REG(&bus->regs->BBB_I2C_CON) |= AM335X_I2C_CON_I2C_EN;
 
  // 5. Start interrupt service routine & one interrupt at a time 
/*
  sc  = rtems_interrupt_handler_install(
    irq,
    "BBB I2C",
    RTEMS_INTERRUPT_UNIQUE,
    am335x_i2c_interrupt,
    bus
   );
  
  if (sc != RTEMS_SUCCESSFUL) {
    (*bus->base.destroy)(&bus->base);
 
    rtems_set_errno_and_return_minus_one(EIO);
  }
  */
  // 6. start transfer for reading and writing 
  bus->base.transfer = am335x_i2c_transfer;
  bus->base.set_clock = am335x_i2c_set_clock;
  bus->base.destroy = am335x_i2c_destroy;
  printk("exit register\n");
  return i2c_bus_register(&bus->base,bus_path);
}






static void omap24_i2c_init(i2c_bus *base)
{
   bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;

  struct am335x_baseboard_id header;
 
  int timeout = I2C_TIMEOUT;
  int deblock = 1;
printf("omap24_i2c_init func!!!!!!\n");
retry:
  if (readw(&bus->regs->BBB_I2C_CON) & I2C_CON_EN) {
    writew(0, &bus->regs->BBB_I2C_CON);
    udelay(50000);
  }

  writew(0x2, &bus->regs->BBB_I2C_SYSC); /* for ES2 after soft reset */
  udelay(1000);

  writew(I2C_CON_EN, &bus->regs->BBB_I2C_CON);
  while (!(readw(&bus->regs->BBB_I2C_SYSS) & I2C_SYSS_RDONE) && timeout--) {
    if (timeout <= 0) {
      puts("ERROR: Timeout in soft-reset\n");
      return;
    }
    udelay(1000);
  }

am335x_i2c_set_clock(&bus->base, I2C_BUS_CLOCK_DEFAULT);

  /* own address */
  writew(1, &bus->regs->BBB_I2C_OA);

//#if defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX)
  /*
   * Have to enable interrupts for OMAP2/3, these IPs don't have
   * an 'irqstatus_raw' register and we shall have to poll 'stat'
   */
 // writew(I2C_IE_XRDY_IE | I2C_IE_RRDY_IE | I2C_IE_ARDY_IE | I2C_IE_NACK_IE | I2C_IE_AL_IE, &i2c_base->ie);
//#endif
  udelay(1000);
  flush_fifo(&bus->base);
  writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);

  /* Handle possible failed I2C state */
  if (wait_for_bb(&bus->base))
    if (deblock == 1) {

      //omap24_i2c_deblock(&bus->base);
      printf("deblock\n");
      deblock = 0;
      goto retry;
    }


  //  omap24_i2c_probe(&bus->base);
 //  read_eeprom(&bus->base,&header);
    
}


static void flush_fifo(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;

 
  u16 stat;
printf("flush_fifo\n");
  /*
   * note: if you try and read data when its not there or ready
   * you get a bus error
   */
  while (1) {
    stat = readw(&bus->regs->BBB_I2C_IRQSTATUS);
    if (stat == I2C_STAT_RRDY) {
      readb(&bus->regs->BBB_I2C_DATA);
      writew(I2C_STAT_RRDY, &bus->regs->BBB_I2C_IRQSTATUS);
      udelay(1000);
    } else
      break;
  }
}


static int wait_for_bb(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;


  int timeout = I2C_TIMEOUT;
  u16 stat;
printf("wait_for_bb\n");
  writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);  /* clear current interrupts...*/
//printf("test1\n");
  /* Read RAW status */
//printf("%x\n", readw(&bus->regs->BBB_I2C_IRQSTATUS_RAW) & I2C_STAT_BB);
  while ((stat = readw(&bus->regs->BBB_I2C_IRQSTATUS_RAW) &
    I2C_STAT_BB) && timeout--) {

    writew(stat, &bus->regs->BBB_I2C_IRQSTATUS);
    udelay(200);
  }

  if (timeout <= 0) {
    printf("Timed out in wait_for_bb: status=%04x\n",
           stat);
    return 1;
  }
  writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);   /* clear delayed stuff*/
  return 0;
}


static int omap24_i2c_probe(i2c_bus *base)
{
 bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;

  unsigned char chip = 0x50;
  u16 status;
  int res = 1; /* default = fail */

printf("omap24_i2c_probe\n");
  if (chip == readw(&bus->regs->BBB_I2C_OA))
    return res;

  /* Wait until bus is free */
  if (wait_for_bb(&bus->base))
    return res;

  /* No data transfer, slave addr only */
  writew(chip, &bus->regs->BBB_I2C_SA);
  /* Stop bit needed here */
  writew(I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX |
         I2C_CON_STP, &bus->regs->BBB_I2C_CON);

  status = wait_for_event(&bus->base);
  printf("status:%x\n",status );

  if ((status & ~I2C_STAT_XRDY) == 0 || (status & I2C_STAT_AL)) {
    /*
     * With current high-level command implementation, notifying
     * the user shall flood the console with 127 messages. If
     * silent exit is desired upon unconfigured bus, remove the
     * following 'if' section:
     */
    if (status == I2C_STAT_XRDY)
      printf("i2c_probe: pads on bus probably not configured (status=0x%x)\n",status);

    goto pr_exit;
  }

  /* Check for ACK (!NAK) */
  if (!(status & I2C_STAT_NACK)) {
    printf("Device found\n");
    res = 0;        /* Device found */
    udelay(200);/* Required by AM335X in SPL */
    /* Abort transfer (force idle state) */
    writew(I2C_CON_MST | I2C_CON_TRX, &bus->regs->BBB_I2C_CON); /* Reset */
    udelay(1000);
    writew(I2C_CON_EN | I2C_CON_MST | I2C_CON_TRX |
           I2C_CON_STP, &bus->regs->BBB_I2C_CON);    /* STP */
  }
pr_exit:
  flush_fifo(&bus->base);
  writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);
  return res;
}


static u16 wait_for_event(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;

 
  u16 status;
  int timeout = I2C_TIMEOUT;
//printf("wait_for_event \n");
  do {
    udelay(200);

    /* Read RAW status */
    status = readw(&bus->regs->BBB_I2C_IRQSTATUS_RAW);

  } while (!(status &
       (I2C_STAT_ROVR | I2C_STAT_XUDF | I2C_STAT_XRDY |
        I2C_STAT_RRDY | I2C_STAT_ARDY | I2C_STAT_NACK |
        I2C_STAT_AL)) && timeout--);

  if (timeout <= 0) {
    printf("Timed out in wait_for_event: status=%04x\n",
           status);
    /*
     * If status is still 0 here, probably the bus pads have
     * not been configured for I2C, and/or pull-ups are missing.
     */
    printf("Check if pads/pull-ups of bus are properly configured\n");
    writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);
    status = 0;
  }

  return status;
}




static int am335x_i2c_read(i2c_bus *base, unsigned char chip, uint addr, int alen, unsigned char *buffer, 
                           int len)
{

  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;

  int i2c_error = 0;
  int i=0;
  u16 status;
printf("am335x_i2c_read\n");
  if (alen < 0) {
    puts("I2C read: addr len < 0\n");
    return 1;
  }
  if (len < 0) {
    puts("I2C read: data len < 0\n");
    return 1;
  }
  if (buffer == NULL) {
    puts("I2C read: NULL pointer passed\n");
    return 1;
  }

  if (alen > 2) {
    printf("I2C read: addr len %d not supported\n", alen);
    return 1;
  }

  if (addr + len > (1 << 16)) {
    puts("I2C read: address out of range\n");
    return 1;
  }

  /* Wait until bus not busy */
  if (wait_for_bb(&bus->base))
    return 1;
//printf("test2\n");
  /* Zero, one or two bytes reg address (offset) */
  writew(alen, &bus->regs->BBB_I2C_CNT);
  /* Set slave address */
  writew(chip, &bus->regs->BBB_I2C_SA);
//printf("test3\n");
  if (alen) {
    /* Must write reg offset first */

    /* Stop - Start (P-S) */
    writew(I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP |
           I2C_CON_TRX, &bus->regs->BBB_I2C_CON);

    /* Send register offset */
    while (1) {
      status = wait_for_event(&bus->base);
      printf("status:%x\n",status );
      /* Try to identify bus that is not padconf'd for I2C */
      if (status == I2C_STAT_XRDY) {
        i2c_error = 2;
        printf("i2c_read (addr phase): pads on bus probably not configured (status=0x%x)\n",
              status);
        goto rd_exit;
      }
      if (status == 0 || (status & I2C_STAT_NACK)) {
        i2c_error = 1;
        printf("i2c_read: error waiting for addr ACK (status=0x%x)\n",
               status);
        goto rd_exit;
      }
      if (alen) {
        if (status & I2C_STAT_XRDY) {
      //    printf("alen:%d\n",alen );
          alen--;
          printf("alen:%d\n",alen );
          printf("addr:%x\n",addr );
          printf("(addr >> (8 * alen)) & 0xff:%x\n",(addr >> (8 * alen)) & 0xff );
          /* Do we have to use byte access? */
          writeb((addr >> (8 * alen)) & 0xff,
                 &bus->regs->BBB_I2C_DATA);
          writew(I2C_STAT_XRDY, &bus->regs->BBB_I2C_IRQSTATUS);
        }
      }
      if (status & I2C_STAT_ARDY) {
           printf("I2C_STAT_ARDY !!!!!\n");
        writew(I2C_STAT_ARDY, &bus->regs->BBB_I2C_IRQSTATUS);
        break;
      }
    }
  }
  printf("&bus->regs->BBB_I2C_CNT:%x\n",readb(&bus->regs->BBB_I2C_CNT) );
  /* Set slave address */
  writew(chip, &bus->regs->BBB_I2C_SA);
  /* Read len bytes from slave */
  writew(len, &bus->regs->BBB_I2C_CNT);
    printf("&bus->regs->BBB_I2C_CNT:%x\n",readb(&bus->regs->BBB_I2C_CNT) );
  /* Need stop bit here */
  writew(I2C_CON_EN | I2C_CON_MST |
         I2C_CON_STT | I2C_CON_STP,
         &bus->regs->BBB_I2C_CON);
//printf("test4\n");


  /* Receive data */
  while (1) {
    status = wait_for_event(&bus->base);
 //   printf("test 5\n");
    /*
     * Try to identify bus that is not padconf'd for I2C. This
     * state could be left over from previous transactions if
     * the address phase is skipped due to alen=0.
     */
    if (status == I2C_STAT_XRDY) {
      i2c_error = 2;
      printf("i2c_read (data phase): pads on bus probably not configured (status=0x%x)\n",
              status);
      goto rd_exit;
    }
    if (status == 0 || (status & I2C_STAT_NACK)) {
     // printf("i2c_error = 1\n");
      i2c_error = 1;
      goto rd_exit;
    }
    if (status & I2C_STAT_RRDY) {
      char temp;
      temp=readb(&bus->regs->BBB_I2C_DATA);
      *buffer++ =temp; 

     *bus->msgs[0].buf++=temp;
  //   (*(uint8_t *) bus->current_msg_byte[0]) = readb(&bus->regs->BBB_I2C_DATA) & 0xFF;
      i++;
      writew(I2C_STAT_RRDY, &bus->regs->BBB_I2C_IRQSTATUS);
    }
    if (status & I2C_STAT_ARDY) {

      writew(I2C_STAT_ARDY, &bus->regs->BBB_I2C_IRQSTATUS);
      break;
    }
  }

rd_exit:
//printf("rd_exit\n");
//printf("i2c_error:%d\n",i2c_error);
  flush_fifo(&bus->base);
  writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);
  return i2c_error;
}



static int read_eeprom(i2c_bus *base,struct am335x_baseboard_id *header)
{

bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;

printf("sizeof(struct am335x_baseboard_id):%d\n",sizeof(struct am335x_baseboard_id) );
//printf("sizeof(struct am335x_baseboard_id):%d\n",sizeof(unsigned int) );
//printf("sizeof(struct am335x_baseboard_id):%d\n",sizeof(unsigned int) );
 am335x_i2c_read(&bus->base,0x50,0,2,(unsigned char *)header,sizeof(struct am335x_baseboard_id));
/*
printf("am335x_i2c_read end\n");
    printf("header->magic:%x\n", header->magic);
     printf("header->name[0]:%x\n", header->name[0]);
      printf("header->name[1]:%x\n", header->name[1]);
     printf("header->name[2]:%x\n", header->name[2]);
      printf("header->name[3]:%x\n", header->name[3]);
       printf("header->name[4]:%x\n", header->name[4]);
        printf("header->name[5]:%x\n", header->name[5]);
         printf("header->name[6]:%x\n", header->name[6]);
          printf("header->name[7]:%x\n", header->name[7]);
          */
}



static int am335x_i2c_write(i2c_bus *base, unsigned char chip, uint addr,int alen, unsigned char *buffer, 
                            int len)
{

  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;

  int i;
  u16 status;
  int i2c_error = 0;
  int timeout = I2C_TIMEOUT;

  if (alen < 0) {
    puts("I2C write: addr len < 0\n");
    return 1;
  }

  if (len < 0) {
    puts("I2C write: data len < 0\n");
    return 1;
  }

  if (buffer == NULL) {
    puts("I2C write: NULL pointer passed\n");
    return 1;
  }

  if (alen > 2) {
    printf("I2C write: addr len %d not supported\n", alen);
    return 1;
  }

  if (addr + len > (1 << 16)) {
    printf("I2C write: address 0x%x + 0x%x out of range\n",
           addr, len);
    return 1;
  }

  /* Wait until bus not busy */
  if (wait_for_bb(&bus->base))
    return 1;

  /* Start address phase - will write regoffset + len bytes data */
  writew(alen + len, &bus->regs->BBB_I2C_CNT);
  /* Set slave address */
  writew(chip,&bus->regs->BBB_I2C_SA);
  /* Stop bit needed here */
  writew(I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX |
         I2C_CON_STP,&bus->regs->BBB_I2C_CON);

  while (alen) {
    /* Must write reg offset (one or two bytes) */
    status = wait_for_event(&bus->base);
    /* Try to identify bus that is not padconf'd for I2C */
    if (status == I2C_STAT_XRDY) {
      i2c_error = 2;
      printf("i2c_write: pads on bus probably not configured (status=0x%x)\n",status);
      goto wr_exit;
    }
    if (status == 0 || (status & I2C_STAT_NACK)) {
      i2c_error = 1;
      printf("i2c_write: error waiting for addr ACK (status=0x%x)\n",
             status);
      goto wr_exit;
    }
    if (status & I2C_STAT_XRDY) {
      alen--;
      writeb((addr >> (8 * alen)) & 0xff, &bus->regs->BBB_I2C_DATA);
      writew(I2C_STAT_XRDY, &bus->regs->BBB_I2C_IRQSTATUS);
    } else {
      i2c_error = 1;
      printf("i2c_write: bus not ready for addr Tx (status=0x%x)\n",
             status);
      goto wr_exit;
    }
  }
  /* Address phase is over, now write data */
  for (i = 0; i < len; i++) {
    status = wait_for_event(&bus->base);
    if (status == 0 || (status & I2C_STAT_NACK)) {
      i2c_error = 1;
      printf("i2c_write: error waiting for data ACK (status=0x%x)\n",
             status);
      goto wr_exit;
    }
    if (status & I2C_STAT_XRDY) {
      writeb(buffer[i], &bus->regs->BBB_I2C_DATA);
      writew(I2C_STAT_XRDY, &bus->regs->BBB_I2C_IRQSTATUS);
    } else {
      i2c_error = 1;
      printf("i2c_write: bus not ready for data Tx (i=%d)\n",
             i);
      goto wr_exit;
    }
  }
  /*
   * poll ARDY bit for making sure that last byte really has been
   * transferred on the bus.
   */
  do {
    status = wait_for_event(&bus->base);
  } while (!(status & I2C_STAT_ARDY) && timeout--);
  if (timeout <= 0)
    printf("i2c_write: timed out writig last byte!\n");

wr_exit:
  flush_fifo(&bus->base);
  writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);
  return i2c_error;
}


