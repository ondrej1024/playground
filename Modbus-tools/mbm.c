/*****************************************************************
 * Modbus master command line tool
 * 
 * Author: Ondrej Wisniewski
 * 
 * Build with this command:
 * gcc mbm.c -o mbm -lmodbus
 * 
 * History:
 * 03/12/2015: First release
 * 13/04/2017: Add handling of Modbus function codes 0x03 and 0x06
 * 
 *****************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <modbus/modbus.h>


#define VERSION       "0.2"

/* Debug mode */
#define DEBUG         0

/* Serial port settings */
#define SERIAL_PORT   "/dev/ttyAMA0"
#define BAUDRATE      9600
#define RTS_DELAY     10

/* Register map settings */
#define MAX_REG       32


int main(int argc, char* argv[])
{
   modbus_t *mb;
   uint16_t tab_reg[MAX_REG]={0};
   uint16_t reg_val;
   int i, k, rc=0;
   char mode;
   int baudrate=BAUDRATE;
   int slave_addr;
   int start_addr;
   int num_reg=1;
   int poll_period=0;
   
   
   if (argc < 6)
   {
      printf("Modbus RTU master, ver %s (using libmodbus %s)\n", VERSION, LIBMODBUS_VERSION_STRING);
      printf("usage: mbm r|R <baudrate> <slave_addr> <start_addr> <num_reg> [<poll_period>]\n");
      printf("       mbm w|W <baudrate> <slave_addr> <start_addr> <reg_val> [<reg_val> ...]\n\n");
      printf("mode:  r - Modbus function code 0x03 (read holding registers)\n");
      printf("       R - Modbus function code 0x04 (read input registers)\n");
      printf("       w - Modbus function code 0x06 (preset single register)\n");
      printf("       W - Modbus function code 0x10 (preset multiple registers)\n\n");
      return 0;
   }

   /**************************************************************
    * Parse input parameters
    **************************************************************/
   
   i = 1;
   mode = argv[i++][0];
   baudrate   = atoi(argv[i++]);
   slave_addr = atoi(argv[i++]);
   start_addr = atoi(argv[i++]);
   switch (mode)
   {
      case 'r':
      case 'R':
         num_reg = atoi(argv[i++]);
         if (argc > i)
            poll_period = atoi(argv[i++]);
      break;
      
      case 'w':
         reg_val = (uint16_t)strtol(argv[i++], NULL, 0);
      break;
      
      case 'W':
         for (k=0; (k<MAX_REG) && (i<argc); k++, i++)
            tab_reg[k] = (uint16_t)strtol(argv[i], NULL, 0);
         num_reg = k;
      break;
      
      default:
         printf("Invalid mode: %c\n", mode);
         return -1;
   }
   
   if (num_reg > MAX_REG) num_reg = MAX_REG;
     
   
   /**************************************************************
    * Initialize communication port
    **************************************************************/
   
   /* Create Modbus context*/
   mb = modbus_new_rtu(SERIAL_PORT, baudrate, 'N', 8, 1);
   if (mb == NULL) {
      printf("Unable to create the libmodbus context\n");
      return -1;
   }
   
   /* Set debug mode */
   if (DEBUG)
      modbus_set_debug(mb, TRUE);
   
   /* Set slave address */
   modbus_set_slave(mb, slave_addr);
   
   /* Connect to serial port */
   if (DEBUG)
      printf("Connecting to slave addr %d\n", slave_addr);
   if (modbus_connect(mb) == -1)
   {
      printf("Connection failed: %s\n", modbus_strerror(errno));
      modbus_free(mb);
      return -1;
   }
   
   /* Set Modbus timeouts */
   modbus_set_response_timeout(mb, 2, 0); // 2s
   modbus_set_byte_timeout(mb, 0, 0);     // not used
   
   if (strstr(SERIAL_PORT, "USB") == NULL)
   {
      /* Enable RS485 direction control via RTS line */
      if (modbus_rtu_set_rts(mb, MODBUS_RTU_RTS_DOWN) == -1)
      {
         printf("Setting RTS mode failed: %s\n", modbus_strerror(errno));
         modbus_free(mb);
         return -1;
      }
      
      /* Set RTS control delay (before and after transmission) */
      if (RTS_DELAY>0)
      {
         if (modbus_rtu_set_rts_delay(mb, RTS_DELAY) == -1)
         {
            printf("Setting RTS delay failed: %s\n", modbus_strerror(errno));
            modbus_free(mb);
            return -1;
         }
      }
      if (DEBUG)
         printf("RTS delay is %dus\n", modbus_rtu_get_rts_delay(mb));
   }
   
   
   /**************************************************************
    * Main loop 
    **************************************************************/
   
   switch (mode)
   {
      case 'r':
         // Modbus function code 0x03 (read holding registers)
         do
         {  
            /* Read registers starting from given address */
            rc = modbus_read_registers(mb, start_addr, num_reg, tab_reg);
            if (rc != num_reg) 
            {
               printf("Unable to read holding registers: %s\n", modbus_strerror(errno));
               break;
            }
            
            /* Print received register values */
            for (i=0; i<num_reg; i++)
               printf("%d: reg %d: 0x%04X (%d)\n", i, start_addr+i, tab_reg[i], tab_reg[i]);
            
            usleep(poll_period*1000000);
            
         } while (poll_period);
      break;
      
      case 'R':
         // Modbus function code 0x04 (read input registers)
         do
         {  
            /* Read registers starting from given address */
            rc = modbus_read_input_registers(mb, start_addr, num_reg, tab_reg);
            if (rc != num_reg) 
            {
               printf("Unable to read input registers: %s\n", modbus_strerror(errno));
               break;
            }
            
            /* Print received register values */
            for (i=0; i<num_reg; i++)
               printf("%d: reg %d: 0x%04X (%d)\n", i, start_addr+i, tab_reg[i], tab_reg[i]);
            
            usleep(poll_period*1000000);
            
         } while (poll_period);
      break;
      
      case 'w':
         // Modbus function code 0x06 (preset single register)
         rc = modbus_write_register(mb, start_addr, reg_val);
         if (rc != 1) 
         {
            printf("Unable to write single register: %s\n", modbus_strerror(errno));
         }
         else
         {
            printf("reg %d: 0x%04X (%d)\n", start_addr, reg_val, reg_val);
         }
      break;

      case 'W':
         // Modbus function code 0x10 (preset multiple registers)
         rc = modbus_write_registers(mb, start_addr, num_reg, tab_reg);
         if (rc != num_reg) 
         {
            printf("Unable to write multiple registers: %s\n", modbus_strerror(errno));
         }
         else
         {
            for (i=0; i<num_reg; i++)
               printf("%d: reg %d: 0x%04X (%d)\n", i, start_addr+i, tab_reg[i], tab_reg[i]);
         }
      break;
      
      default:;
   }
   
   /**************************************************************
    * Clean up end exit
    **************************************************************/
   modbus_close(mb);
   modbus_free(mb);
   
   return rc;
}
