/**
 * @file    fork-uart.c
 * 
 * @brief Serial Port Programming in C (Serial Port Read)  
 * Non Cannonical mode   
 * Sellecting the Serial port Number on Linux   
 * /dev/ttyUSBx - when using USB to Serial Converter, where x can be 0,1,2...etc 
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc  
 * termios structure -  /usr/include/asm-generic/termbits.h  
 * use "man termios" to get more info about  termios structure
 * @author  Chat GPT
 * @date    2024-10-07
 */	
#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions 
#include <unistd.h>  // UNIX Standard Definitions 
#include <errno.h>   // ERROR Number Definitions
#include <stdlib.h>  
#include <string.h>  
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>      // For signal handling

// device port série à utiliser 
const char *portTTY = "/dev/ttyS1";
//const char *portTTY = "/dev/ttyGS0"; 
//const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter


int init_serial_port()
{
	// Opening the Serial Port 
	int fd = open(portTTY, O_RDWR | O_NOCTTY);  
	if(fd == -1) // Error Checking
	{
	return -1;
	}
	
	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port 
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed  
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
	SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG);  //Disable echo, Disable signal  
	SerialPortSettings.c_lflag |= ICANON;  // Non-Cannonical mode 
	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

	// Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 2; // Read at least 1 character(s) 
	SerialPortSettings.c_cc[VTIME] = 0; // Wait 3sec (0 for indefinetly) 

	if(tcsetattr(fd, TCSANOW, &SerialPortSettings)!= 0) // Set the attributes to the termios structure
	{
	printf("\n  Erreur! configuration des attributs du port serie");
	return -1;
	}
 return fd;
}



void parent_process(int fd) 
{
    char read_buffer[256];   
    int bytes_read;  

    printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série...\n");
    
    while (1) 
	{
        bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);
        if (bytes_read > 0)
		 {
            read_buffer[bytes_read] = '\0'; // Null-terminate the string
            printf(" processus Père: nombres d'octets recus : %d --> %s\n", bytes_read, read_buffer);
            if (read_buffer[0] == '!')
			{
                break; // Exit if '!' is detected
            }
        }
    } 
    printf("Fin du Père\n");
}

void child_process(int fd) 
{
    char input_buffer[256];

    printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (terminal)...\n");
    
    while (1) {
        fgets(input_buffer, sizeof(input_buffer), stdin);
        write(fd, input_buffer, strlen(input_buffer));
        if (input_buffer[0] == 'q') 
		{
            
            break; // Exit if 'q' is detected
        }
    }
    printf("Fin du Fils\n");
}



int main(void) 
{
  int fd = init_serial_port();
  if (fd == -1) 
  {
   return EXIT_FAILURE;
  }

    pid_t pid = fork();

    if (pid < 0) {
        perror("Erreur de fork");
        return EXIT_FAILURE;
    } else if (pid == 0) {
        // Child process
        child_process(fd);
    } else {
        // Parent process
        parent_process(fd);
        wait(NULL); // Wait for the child to finish
    }

    close(fd); // Close the serial port
    return EXIT_SUCCESS;
}