/*****************************************************************
 * Modbus slave command line tool
 * 
 * Author: Ondrej Wisniewski
 * 
 * Build with this command:
 * gcc mbs.c -o mbs -lmodbus
 * 
 * History:
 * 28/04/2017: First release
 * 
 *****************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <unistd.h>
#include <modbus/modbus.h>


#define VERSION       "0.1"

/* Debug mode */
#define DEBUG         0

/* Serial port settings */
//#define SERIAL_PORT   "/dev/ttyUSB0"
#define SERIAL_PORT   "/dev/ttyAMA0"
#define BAUDRATE      9600
#define RTS_DELAY     10

/* Register map settings */
#define MAX_REG       32

/* Modbus request structure */
typedef struct {
   uint8_t slave_addr;
   uint8_t fc;
   uint8_t reg_addr_hi;
   uint8_t reg_addr_lo;
   uint8_t reg_val_hi;
   uint8_t reg_val_lo;
} modbus_request_t;

/* Flag to indicate exit from main loop */
static int cont=1;

/* Device register map */
uint16_t reg_map[MAX_REG];


int read_reg(int reg_addr, uint16_t* reg_val)
{
   *reg_val = reg_map[reg_addr];
   
   if (DEBUG)
      printf("DBG: Read val %d from addr %d\n", *reg_val, reg_addr);
  
  return 0;
}


int write_reg(int reg_addr, uint16_t reg_val)
{
   reg_map[reg_addr] = reg_val;
   
   if (DEBUG)
      printf("DBG: Wrote val %d to addr %d\n", reg_val, reg_addr);
  
  return 0;
}


int main(int argc, char* argv[])
{
   modbus_t *mb;
   int header_length;
   int i, k, rc=0;
   int baudrate=BAUDRATE;
   int slave_addr, own_addr;
   
   
   if (argc < 3)
   {
      printf("Modbus RTU slave, ver %s (using libmodbus %s)\n", VERSION, LIBMODBUS_VERSION_STRING);
      printf("usage: mbs <baudrate> <slave_addr>\n\n");
      return 0;
   }

   openlog("modbus server", LOG_PID|LOG_CONS, LOG_USER);

   /**************************************************************
    * Parse input parameters
    **************************************************************/
   
   i = 1;
   baudrate   = atoi(argv[i++]);
   own_addr = atoi(argv[i++]);
     
   
   /**************************************************************
    * Initialize communication port
    **************************************************************/
   
   /* Create Modbus context*/
   mb = modbus_new_rtu(SERIAL_PORT, baudrate, 'N', 8, 1);
   if (mb == NULL) {
      syslog(LOG_DAEMON | LOG_ERR, "Unable to create the libmodbus context\n");
      return -1;
   }
   
   header_length = modbus_get_header_length(mb);
   
   /* Set debug mode */
   if (DEBUG)
      modbus_set_debug(mb, TRUE);
   
   /* Set slave address */
   modbus_set_slave(mb, own_addr);
   
   /* Connect to serial port */
   if (DEBUG)
      printf("Connecting to slave addr %d\n", own_addr);
   if (modbus_connect(mb) == -1)
   {
      syslog(LOG_DAEMON | LOG_ERR, "Connection failed: %s\n", modbus_strerror(errno));
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
         syslog(LOG_DAEMON | LOG_ERR, "Setting RTS mode failed: %s\n", modbus_strerror(errno));
         modbus_free(mb);
         return -1;
      }
      
      /* Set RTS control delay (before and after transmission) */
      if (RTS_DELAY>0)
      {
         if (modbus_rtu_set_rts_delay(mb, RTS_DELAY) == -1)
         {
            syslog(LOG_DAEMON | LOG_ERR, "Setting RTS delay failed: %s\n", modbus_strerror(errno));
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
   while (cont)
   {
      uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
      int rc;
   
      /* Receive data from client */    
      rc = modbus_receive(mb, query);      /* rc is the query size */
      if (rc == -1) 
      { 
         syslog(LOG_DAEMON | LOG_ERR, "Slave #%d: modbus_receive() failed: %s", 
                                       own_addr, modbus_strerror(errno));
      }
      else 
      { 
         /* Get information from request buffer */
         modbus_request_t *modbus_request;
         int operation;
         int reg_addr;
         uint16_t reg_val;
         uint16_t exception_code;
         modbus_mapping_t *mb_mapping;
         
         modbus_request = (modbus_request_t *)&query[header_length-1];
         
         slave_addr = modbus_request->slave_addr;
         operation  = modbus_request->fc;
         reg_addr   = (int)modbus_request->reg_addr_hi<<8 | (int)modbus_request->reg_addr_lo;
         reg_val    = (int)modbus_request->reg_val_hi<<8 | (int)modbus_request->reg_val_lo;
         
         exception_code = 0;
         
         if (DEBUG)
            printf("DBG: received request for slave %d, op %d, addr %d, reg_val %d\n", 
                                            slave_addr, operation, reg_addr, reg_val);
         
         /* Check if slave address matches with our own address */
         if (slave_addr != own_addr)
         { 
            continue;
         }
         
         
         /* Initialise new response data structure */
         mb_mapping = modbus_mapping_new(0,0,MAX_REG,0);
         if (mb_mapping == NULL) 
         {
            syslog(LOG_DAEMON | LOG_ERR, "Slave #%d: Failed to allocate the mapping: %s", 
                                                 own_addr, modbus_strerror(errno));
            modbus_free(mb);
            return -1;
         }
         
         /* Perform requested operation */
         switch (operation)
         {
            case 0x03:  /* FC Read Holding Registers */
            case 0x04:  /* FC Read Input Registers */
               if (reg_addr < MAX_REG) 
               {
                  /* Call the "Read register" handler function */ 
                  if (read_reg(reg_addr, &reg_val) != 0)
                  {  /* Error during register read occured */
                     exception_code = MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                  }
                  else
                  {  /* Copy read value into response buffer */
                     mb_mapping->tab_registers[reg_addr] = reg_val;
                  }
               }
               else
               {  /* Register address out of range */
                  exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
               }
               break;	
         
            case 0x06:  /* FC Write single register */
               if (reg_addr < MAX_REG) 
               {
                  /* Call the "Write register" handler function */
                  if (write_reg(reg_addr, reg_val) != 0)
                  {  
                     /* Error during register write occured */
                     exception_code = MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                  }
               }
               else
               {  /* Register address out of range */
                  exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
               }
               break;
      
            default:
               syslog(LOG_DAEMON | LOG_ERR, "Invalid operation %d\n", operation);
               exception_code = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
               
         } //end switch statement
   
         /* Send reply to client */
         if (exception_code == 0)
         {
            rc = modbus_reply(mb, query, rc, mb_mapping);
            if (rc == -1) 
            {
               syslog(LOG_DAEMON | LOG_ERR, "Slave #%d: Failed to send reply to the client: %s", 
                                                        own_addr, modbus_strerror(errno));
            }
         }
         else
         {
            rc = modbus_reply_exception(mb, query, exception_code);
            if (rc == -1) 
            {
               syslog(LOG_DAEMON | LOG_ERR, "Slave #%d: Failed to send exception reply to the client: %s", 
                                                        own_addr, modbus_strerror(errno));
            }
         }
         modbus_mapping_free(mb_mapping); 
      }
   } // end of main server loop
      
   /**************************************************************
    * Clean up end exit
    **************************************************************/
   modbus_close(mb);
   modbus_free(mb);
   
   return rc;
}
