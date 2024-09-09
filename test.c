#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define UART_PORT "/dev/ttyO1"  // UART1 sur Beaglebone Blue
#define MAX_DATA_SIZE 8        // Taille maximale des données reçues

// Fonction pour configurer le port série
int configure_uart(int fd) {
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600); // Vitesse de 9600 bauds
    cfsetospeed(&options, B9600); // Vitesse de 9600 bauds

    options.c_cflag &= ~PARENB;   // Pas de parité
    options.c_cflag &= ~CSTOPB;   // 1 bit de stop
    options.c_cflag &= ~CSIZE;    // Clear current char size mask
    options.c_cflag |= CS8;       // 8 bits de données

    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Pas de contrôle de flux logiciel
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Mode brut
    options.c_oflag &= ~OPOST;    // Mode brut

    tcsetattr(fd, TCSANOW, &options); // Appliquer les paramètres
    return 0;
}

// Fonction pour recevoir une trame UART
int receive_uart_data(int fd, char *data) {
    int n = read(fd, data, MAX_DATA_SIZE); // Lire jusqu'à 8 octets
    if (n < 0) {
        perror("Erreur de lecture UART");
        return -1;
    }
    return n; // Retourne le nombre d'octets lus
}

// Fonction pour envoyer une réponse via UART
void send_uart_data(int fd, const char *data) {
    int len = strlen(data);
    int n = write(fd, data, len); // Envoie la trame
    if (n < 0) {
        perror("Erreur d'écriture UART");
    }
}

int main() {
    int uart_fd;
    char received_data[MAX_DATA_SIZE];
    
    // Ouvrir le port UART1
    uart_fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd == -1) {
        perror("Erreur lors de l'ouverture du port série");
        return 1;
    }

    // Configurer le port UART
    configure_uart(uart_fd);

    while (1) {
        memset(received_data, 0, MAX_DATA_SIZE); // Réinitialiser le buffer
        int bytes_received = receive_uart_data(uart_fd, received_data);

        if (bytes_received > 0) {
            printf("Données reçues : %s\n", received_data);

            // Exemple de réponse basée sur la donnée reçue
            if (strcmp(received_data, "START") == 0) {
                send_uart_data(uart_fd, "Système démarré");
            } else if (strcmp(received_data, "STOP") == 0) {
                send_uart_data(uart_fd, "Système arrêté");
            } else {
                send_uart_data(uart_fd, "Commande non reconnue");
            }
        }
    }

    close(uart_fd); // Fermer le port UART
    return 0;
}
