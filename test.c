// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <termios.h>
// #include <errno.h>

// #define UART_PORT "/dev/ttyS1"  // UART1 sur Beaglebone Blue
// #define MAX_DATA_SIZE 8        // Taille maximale des données reçues


// // Fonction pour configurer le port série
// int configure_uart(int fd) 
// {
//     struct termios options;
//     if (tcgetattr(fd, &options) < 0) {
//         perror("tcgetattr");
//         return -1;
//     }
    
//     // Configurer la vitesse de transmission (9600 bauds, 8 bits, sans parité, 1 bit d'arrêt)
//     cfsetispeed(&options, B115200);
//     cfsetospeed(&options, B115200);

//     // Désactivation du contrôle de flux, pas de parité, 8 bits de données, 1 bit d'arrêt
//     options.c_cflag &= ~PARENB;    // Pas de parité
//     options.c_cflag &= ~CSTOPB;    // 1 bit d'arrêt
//     options.c_cflag &= ~CSIZE;     // Masquer la taille des données
//     options.c_cflag |= CS8;        // 8 bits de données
//     // options.c_cflag &= ~CRTSCTS;   // Pas de contrôle de flux matériel

//     // Activer les options de lecture et de réception
//     options.c_cflag |= CREAD | CLOCAL;

//     // Appliquer les modifications
//     if (tcsetattr(fd, TCSANOW, &options) < 0) {
//         perror("tcsetattr");
//         return -1;
//     }

//     // Vider les tampons d'entrée et de sortie
//     tcflush(fd, TCIOFLUSH);

//     return 0;
// }

// // Fonction pour lire les données de l'UART
// int read_uart(int fd, char *buffer, size_t size)
// {
//     int n = read(fd, buffer, size);
//     if (n < 0) {
//         perror("read");
//         return -1;
//     }
//     return n;
// }

// // Fonction pour envoyer des données via UART
// int write_uart(int fd, const char *data)
// {
//     int n = write(fd, data, strlen(data));
//     if (n < 0) {
//         perror("write");
//         return -1;
//     }
//     return n;
// }

// int main()
// {
//     int fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd < 0) {
//         perror("open");
//         return -1;
//     }

//     // Configurer le port série
//     if (configure_uart(fd) < 0) {
//         close(fd);
//         return -1;
//     }

//     // Données à envoyer
//     const char *data_to_send = "$N,25.50,1,Arret";
//     if (write_uart(fd, data_to_send) < 0) 
//     {
//         close(fd);
//         return -1;
//     }
//     printf("Données envoyées: %s\n", data_to_send);

//     char received_data[MAX_DATA_SIZE];

//     // Boucle infinie pour lire les données entrantes
//     while (1) 
//     {
//         int bytes_received = read_uart(fd, received_data, MAX_DATA_SIZE);

//         if (bytes_received > 0) 
//         {
//             printf("Données reçues: %s\n", received_data);
//         }
//         else if (bytes_received == 0)
//         {
//             printf("Aucune donnée reçue pour le moment\n"); 
//         }
//         else 
//         {
//             printf("Erreur lors de la lecture des données\n"); 
//         }
//         // Ajouter un délai pour éviter de surcharger le CPU
//         usleep(100000); // 100 ms
//     }

//     // Fermer le port série
//     close(fd);

//     return 0;
// }


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/wait.h>
#include <signal.h>

#define UART_PORT "/dev/ttyS1"  // UART1 sur Beaglebone Blue
#define MAX_DATA_SIZE 8         // Taille maximale des données reçues

volatile sig_atomic_t keep_running = 1;

void sigint_handler(int sig)
{
    keep_running = 0;
}

// Fonction pour configurer le port série
int configure_uart(int fd) 
{
    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        perror("tcgetattr");
        return -1;
    }
    
    // Configurer la vitesse de transmission (115200 bauds, 8 bits, sans parité, 1 bit d'arrêt)
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Désactivation du contrôle de flux, pas de parité, 8 bits de données, 1 bit d'arrêt
    options.c_cflag &= ~PARENB;    // Pas de parité
    options.c_cflag &= ~CSTOPB;    // 1 bit d'arrêt
    options.c_cflag &= ~CSIZE;     // Masquer la taille des données
    options.c_cflag |= CS8;        // 8 bits de données

    // Activer les options de lecture et de réception
    options.c_cflag |= CREAD | CLOCAL;

    // Appliquer les modifications
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("tcsetattr");
        return -1;
    }

    // Vider les tampons d'entrée et de sortie
    tcflush(fd, TCIOFLUSH);

    return 0;
}

// Fonction pour lire les données de l'UART
int read_uart(int fd, char *buffer, size_t size)
{
    int n = read(fd, buffer, size);
    if (n < 0) {
        perror("read");
        return -1;
    }
    buffer[n] = '\0'; // Terminer la chaîne avec un caractère nul
    return n;
}

// Fonction pour envoyer des données via UART
int write_uart(int fd, const char *data)
{
    int n = write(fd, data, strlen(data));
    if (n < 0) {
        perror("write");
        return -1;
    }
    return n;
}

int main()
{
    // Gestion du signal Ctrl+C pour une sortie propre
    signal(SIGINT, sigint_handler);

    int fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    // Configurer le port série
    if (configure_uart(fd) < 0) {
        close(fd);
        return -1;
    }

    pid_t pid = fork();

    if (pid < 0) {
        // Erreur lors du fork
        perror("fork");
        close(fd);
        return -1;
    }
    else if (pid > 0) {
        // Processus parent: envoi des données
        while (keep_running) {
            const char *data_to_send = "$N,25.50,1,Arret";
            if (write_uart(fd, data_to_send) < 0) {
                break;
            }
            printf("Données envoyées: %s\n", data_to_send);
            sleep(1); // Envoyer les données toutes les secondes
        }
        // Attendre que le processus enfant se termine
        wait(NULL);
        close(fd);
        printf("Processus parent terminé.\n");
    }
    else {
        // Processus enfant: lecture des données
        char received_data[MAX_DATA_SIZE + 1]; // +1 pour le caractère nul

        while (keep_running) {
            int bytes_received = read_uart(fd, received_data, MAX_DATA_SIZE);

            if (bytes_received > 0) {
                printf("Données reçues: %s\n", received_data);
            }
            else if (bytes_received == 0) {
                // Aucune donnée reçue pour le moment
            }
            else {
                printf("Erreur lors de la lecture des données\n");
            }
            // Ajouter un délai pour éviter de surcharger le CPU
            usleep(100000); // 100 ms
        }
        close(fd);
        printf("Processus enfant terminé.\n");
    }

    return 0;
}
