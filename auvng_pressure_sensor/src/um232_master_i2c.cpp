#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include "../include/ftd2xx.h"

bool bCommandEchod = false;
	FT_STATUS ftStatus;					// Status defined in D2XX to indicate operation result
	FT_HANDLE ftHandle;					// Handle of FT232H device port 

	BYTE OutputBuffer[1024];			// Buffer to hold MPSSE commands and data to be sent to FT232H
	BYTE InputBuffer[1024];				// Buffer to hold Data bytes read from FT232H
	
	DWORD dwClockDivisor = 0x00C8;		// 100khz
	
	DWORD dwNumBytesToSend = 0; 		// Counter used to hold number of bytes to be sent
	DWORD dwNumBytesSent = 0;			// Holds number of bytes actually sent (returned by the read function)

	DWORD dwNumInputBuffer = 0;			// Number of bytes which we want to read
	DWORD dwNumBytesRead = 0;			// Number of bytes actually read
	DWORD ReadTimeoutCounter = 0;		// Used as a software timeout counter when the code checks the Queue Status

	BYTE ByteDataRead[4];				// Array for storing the data which was read from the I2C Slave
	BOOL DataInBuffer  = 0;				// Flag which code sets when the GetNumBytesAvailable returned is > 0 
	BYTE DataByte = 0;					// Used to store data bytes read from and written to the I2C Slave

void SetI2CLinesIdle(void)
{
	dwNumBytesToSend = 0;			//Clear output buffer

	// Set the idle states for the AD lines
	OutputBuffer[dwNumBytesToSend++] = 0x80;	// Command to set directions of ADbus and data values for pins set as o/p
	OutputBuffer[dwNumBytesToSend++] = 0xFF;    // Set all 8 lines to high level (only affects pins which are output)
	OutputBuffer[dwNumBytesToSend++] = 0xFB;	// Set all pins as output except bit 2 which is the data_in

	// IDLE line states are ...
	// AD0 (SCL) is output high (open drain, pulled up externally)
	// AD1 (DATA OUT) is output high (open drain, pulled up externally)
	// AD2 (DATA IN) is input (therefore the output value specified is ignored)
	// AD3 to AD7 are inputs (not used in this application)

	// Set the idle states for the AC lines
	OutputBuffer[dwNumBytesToSend++] = 0x82;	// Command to set directions of ACbus and data values for pins set as o/p
	OutputBuffer[dwNumBytesToSend++] = 0xFF;	// Set all 8 lines to high level (only affects pins which are output)
	OutputBuffer[dwNumBytesToSend++] = 0x40;	// Only bit 6 is output

	// IDLE line states are ...
	// AC6 (LED) is output driving high
	// AC0/1/2/3/4/5/7 are inputs (not used in this application)

	ftStatus = FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);		//Send off the commands
}



int main(int argc, char *argv[])
{
	char name[] = "UM232H";
	DWORD dwCount;
	DWORD devIndex = 0;
	char Buf[64];
	
	ftStatus = FT_OpenEx(name, FT_OPEN_BY_DESCRIPTION, &ftHandle);
	if(ftStatus != FT_OK) {
		/* 
			This can fail if the ftdi_sio driver is loaded
		 	use lsmod to check this and rmmod ftdi_sio to remove
			also rmmod usbserial
		 */
		printf("FT_Open() failed\n");
		getchar();
		return 1;
	}
	else
    {   
		// #########################################################################################
		// After opening the device, Put it into MPSSE mode
		// #########################################################################################
				
		// Print message to show port opened successfully
		printf("Successfully opened FT232H device! \n");

		// Reset the FT232H
		ftStatus |= FT_ResetDevice(ftHandle); 
		if(ftStatus != FT_OK) 
		{
			printf("Reset Device failed\n");
		}

		printf("Reset Device Done \n");	
		
		// Purge USB receive buffer ... Get the number of bytes in the FT232H receive buffer and then read them
		ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumInputBuffer);	 
		if ((ftStatus == FT_OK) && (dwNumInputBuffer > 0))
		{
			FT_Read(ftHandle, &InputBuffer, dwNumInputBuffer, &dwNumBytesRead);  	
		}

		ftStatus |= FT_SetUSBParameters(ftHandle, 65536, 65535);			// Set USB request transfer sizes
		ftStatus |= FT_SetChars(ftHandle, false, 0, false, 0);				// Disable event and error characters
		ftStatus |= FT_SetTimeouts(ftHandle, 5000, 5000);					// Set the read and write timeouts to 5 seconds
		ftStatus |= FT_SetLatencyTimer(ftHandle, 16);						// Keep the latency timer at default of 16ms
		ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x00); 					// Reset the mode to whatever is set in EEPROM
		ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x02);	 					// Enable MPSSE mode
		
		// Inform the user if any errors were encountered
		if (ftStatus != FT_OK)
		{
			printf("failure to initialize FT232H device! \n");
			getchar();
			return 1;
		}

		usleep(50); 

		// #########################################################################################
		// Synchronise the MPSSE by sending bad command AA to it
		// #########################################################################################

		dwNumBytesToSend = 0;																// Used as an index to the buffer
		OutputBuffer[dwNumBytesToSend++] = 0xAA;											// Add an invalid command 0xAA
		ftStatus = FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);		// Send off the invalid command
		
		// Check if the bytes were sent off OK
		if(dwNumBytesToSend != dwNumBytesSent)
		{
			printf("Write timed out! \n");
			getchar();
			return 1;
		}

		printf("Write AA to MPSSE Done \n");

		// Now read the response from the FT232H. It should return error code 0xFA followed by the actual bad command 0xAA
		// Wait for the two bytes to come back 

		dwNumInputBuffer = 0;
		ReadTimeoutCounter = 0;

		ftStatus = FT_GetQueueStatus(ftHandle, &dwNumInputBuffer);						// Get number of bytes in the input buffer

		while ((dwNumInputBuffer < 2) && (ftStatus == FT_OK) && (ReadTimeoutCounter < 500))
		{
			// Sit in this loop until
			// (1) we receive the two bytes expected
			// or (2) a hardware error occurs causing the GetQueueStatus to return an error code
			// or (3) we have checked 500 times and the expected byte is not coming 
			ftStatus = FT_GetQueueStatus(ftHandle, &dwNumInputBuffer);	// Get number of bytes in the input buffer
			ReadTimeoutCounter ++;
			usleep(1);												// short delay
		}

		if(ftStatus == FT_OK){
			printf("Read MPSSE Done \n");
			printf("ReadTimeoutCounter:%d \n", ReadTimeoutCounter);
		}


		// If the loop above exited due to the byte coming back (not an error code and not a timeout)
		// then read the bytes available and check for the error code followed by the invalid character
		if ((ftStatus == FT_OK) && (ReadTimeoutCounter < 500))
		{
			ftStatus = FT_Read(ftHandle, &InputBuffer, dwNumInputBuffer, &dwNumBytesRead); // Now read the data

			// Check if we have two consecutive bytes in the buffer with value 0xFA and 0xAA
			bCommandEchod = false;
			for (dwCount = 0; dwCount < dwNumBytesRead - 1; dwCount++)							
			{
				if ((InputBuffer[dwCount] == BYTE(0xFA)) && (InputBuffer[dwCount+1] == BYTE(0xAA)))
				{
					bCommandEchod = true;
					break;
				}
			}
		}
		// If the device did not respond correctly, display error message and exit.

		if (bCommandEchod == false) 
		{	
			printf("fail to synchronize MPSSE with command 0xAA \n");
			getchar();
			return 1;
		}

		printf("Sync MPSSE Done \n");

		// #########################################################################################
		// Synchronise the MPSSE by sending bad command AB to it
		// #########################################################################################

		dwNumBytesToSend = 0;																// Used as an index to the buffer
		OutputBuffer[dwNumBytesToSend++] = 0xAB;											// Add an invalid command 0xAB
		ftStatus = FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);		// Send off the invalid command
		
		// Check if the bytes were sent off OK
		if(dwNumBytesToSend != dwNumBytesSent)
		{
			printf("Write timed out! \n");
			getchar();
			return 1;
		}


		// Now read the response from the FT232H. It should return error code 0xFA followed by the actual bad command 0xAA
		// Wait for the two bytes to come back 

		dwNumInputBuffer = 0;
		ReadTimeoutCounter = 0;

		ftStatus = FT_GetQueueStatus(ftHandle, &dwNumInputBuffer);						// Get number of bytes in the input buffer

		while ((dwNumInputBuffer < 2) && (ftStatus == FT_OK) && (ReadTimeoutCounter < 500))
		{
			// Sit in this loop until
			// (1) we receive the two bytes expected
			// or (2) a hardware error occurs causing the GetQueueStatus to return an error code
			// or (3) we have checked 500 times and the expected byte is not coming 
			ftStatus = FT_GetQueueStatus(ftHandle, &dwNumInputBuffer);	// Get number of bytes in the input buffer
			ReadTimeoutCounter ++;
			usleep(1);													// short delay
		}

		// If the loop above exited due to the byte coming back (not an error code and not a timeout)
		// then read the bytes available and check for the error code followed by the invalid character
		if ((ftStatus == FT_OK) && (ReadTimeoutCounter < 500))
		{
			ftStatus = FT_Read(ftHandle, &InputBuffer, dwNumInputBuffer, &dwNumBytesRead); // Now read the data

			// Check if we have two consecutive bytes in the buffer with value 0xFA and 0xAB
			bCommandEchod = false;
			for (dwCount = 0; dwCount < dwNumBytesRead - 1; dwCount++)							
			{
				if ((InputBuffer[dwCount] == BYTE(0xFA)) && (InputBuffer[dwCount+1] == BYTE(0xAB)))
				{
					bCommandEchod = true;
					break;
				}
			}
		}
		// If the device did not respond correctly, display error message and exit.

			if (bCommandEchod == false) 
		{	
			printf("fail to synchronize MPSSE with command 0xAB \n");
			getchar();
			return 1;
		}


		printf("MPSSE synchronized with BAD command \n");

		// #########################################################################################
		// Configure the MPSSE settings
		// #########################################################################################

		dwNumBytesToSend = 0;							// Clear index to zero
		OutputBuffer[dwNumBytesToSend++] = 0x8A; 		// Disable clock divide-by-5 for 60Mhz master clock
		OutputBuffer[dwNumBytesToSend++] = 0x97;		// Ensure adaptive clocking is off
		OutputBuffer[dwNumBytesToSend++] = 0x8C; 		// Enable 3 phase data clocking, data valid on both clock edges for I2C

		OutputBuffer[dwNumBytesToSend++] = 0x9E; 		// Enable the FT232H's drive-zero mode on the lines used for I2C ...
		OutputBuffer[dwNumBytesToSend++] = 0x07;		// ... on the bits 0, 1 and 2 of the lower port (AD0, AD1, AD2)...
		OutputBuffer[dwNumBytesToSend++] = 0x00;		// ...not required on the upper port AC 0-7

		OutputBuffer[dwNumBytesToSend++] = 0x85;		// Ensure internal loopback is off

		ftStatus = FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);	// Send off the commands

		// Now configure the dividers to set the SCLK frequency which we will use
		// The SCLK clock frequency can be worked out by the algorithm (when divide-by-5 is off)
		// SCLK frequency  = 60MHz /((1 +  [(1 +0xValueH*256) OR 0xValueL])*2)
		dwNumBytesToSend = 0;													// Clear index to zero
		OutputBuffer[dwNumBytesToSend++] = 0x86; 								// Command to set clock divisor
		OutputBuffer[dwNumBytesToSend++] = dwClockDivisor & 0xFF;				// Set 0xValueL of clock divisor
		OutputBuffer[dwNumBytesToSend++] = (dwClockDivisor >> 8) & 0xFF;		// Set 0xValueH of clock divisor
		ftStatus = FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);	// Send off the commands

		usleep(20);																// Short delay 	
				
		// #########################################################################################
		// Configure the I/O pins of the MPSSE
		// #########################################################################################

		// Call the I2C function to set the lines of port AD to their required states
		SetI2CLinesIdle();

		// Also set the required states of port AC0-7. Bit 6 is used as an active-low LED, the others are unused
		// After this instruction, bit 6 will drive out high (LED off)
		//dwNumBytesToSend = 0;						// Clear index to zero
		//OutputBuffer[dwNumBytesToSend++] = '\x82';	// Command to set directions of upper 8 pins and force value on bits set as output
		//OutputBuffer[dwNumBytesToSend++] = '\xFF';  // Write 1's to all bits, only affects those set as output
		//OutputBuffer[dwNumBytesToSend++] = '\x40';	// Set bit 6 as an output
		//ftStatus = FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);	// Send off the commands

		usleep(30);		//Delay for a while
	}
		BOOL bSucceed = TRUE;

	// #########################################################################################
	// #########################################################################################
	// MAIN PROGRAM
	// #########################################################################################
	// #########################################################################################
	
	
	

	
	

}