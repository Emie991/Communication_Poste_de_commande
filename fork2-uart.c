#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions
#include <sys/types.h> // For pid_t
#include <sys/wait.h> // For wait()
#include <stdlib.h>   // For exit()

// Device port série à utiliser 
const char *portTTY = "/dev/ttyS1"; 

void configure_serial(int fd) {
    struct termios SerialPortSettings; // Create the structure 
    tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port

    // Setting the Baud rate
    cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed   
    cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  

    // 8N1 Mode 
    SerialPortSettings.c_cflag &= ~PARENB; // No Parity
    SerialPortSettings.c_cflag &= ~CSTOPB; // 1 Stop bit
    SerialPortSettings.c_cflag &= ~CSIZE;  // Clears the mask for setting the data size
    SerialPortSettings.c_cflag |= CS8;      // Set the data bits = 8
    SerialPortSettings.c_cflag &= ~CRTSCTS; // No Hardware flow Control
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control

    SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG); // Disable echo, Disable signal  
    SerialPortSettings.c_lflag &= ~ICANON; // Non-Cannonical mode 
    SerialPortSettings.c_oflag &= ~OPOST; // No Output Processing

    SerialPortSettings.c_cc[VMIN] = 0; // Read at least 0 character(s) 
    SerialPortSettings.c_cc[VTIME] = 100; // Wait 10 sec (0 for indefinitely) 

    // Set the attributes to the termios structure
    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        perror("Erreur! configuration des attributs du port serie");
        exit(EXIT_FAILURE);
    }
}

void write_to_serial(int fd) {
    char write_buffer[] = "ABCDE12345"; // Buffer containing characters to write into port
    int bytes_written = write(fd, write_buffer, sizeof(write_buffer)); // Write data to serial port 

    printf("\nEcriture de %d octets : %s ecrit sur le port %s\n", bytes_written, write_buffer, portTTY);
    close(fd); // Close the Serial port
}

void read_from_serial(int fd) {
    char read_buffer[32];   // Buffer to store the data received 
    int bytes_read = read(fd, &read_buffer, sizeof(read_buffer)); // Read the data 

    printf("Bytes Recus : %d", bytes_read); // Print the number of bytes read
    close(fd); // Close the serial port
}

int main(void) {
    int fd_portUART;

    // Open the Serial Port
    fd_portUART = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_portUART == -1) {
        perror("Erreur! ouverture de port");
        return EXIT_FAILURE;
    }
    
    configure_serial(fd_portUART);

    pid_t pid_write = fork();
    if (pid_write < 0) {
        perror("Fork for writing failed");
        return EXIT_FAILURE;
    }

    if (pid_write == 0) {
        // Child process for writing
        write_to_serial(fd_portUART);
        exit(EXIT_SUCCESS);
    }

    pid_t pid_read = fork();
    if (pid_read < 0) {
        perror("Fork for reading failed");
        return EXIT_FAILURE;
    }

    if (pid_read == 0) {
        // Child process for reading
        fd_portUART = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_portUART == -1) {
            perror("Erreur! ouverture de port pour lecture");
            exit(EXIT_FAILURE);
        }
        configure_serial(fd_portUART);
        read_from_serial(fd_portUART);
        exit(EXIT_SUCCESS);
    }

    // Main process
    int n = 1; 
    while (n < 10) {
        printf("1 faire quelques trucs...\n");
        n++;
        sleep(3);
    }

    // Wait for both child processes to finish
    wait(NULL); // attendre la fin de l'enfant 1
    wait(NULL); // attendre la fin de l'enfant 2

    close(fd_portUART); // Fermer le port série 
    printf("Fin du processus Principal\n");

    return EXIT_SUCCESS;
}