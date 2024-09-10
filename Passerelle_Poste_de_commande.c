
// #define _GNU_SOURCE
// #define _DEFAULT_SOURCE

// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <termios.h>
// #include <errno.h>
// #include <sys/wait.h>
// #include <signal.h>
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <sys/ioctl.h>
// #include <net/if.h>
// #include <linux/can.h>
// #include <linux/can/raw.h>

// #define UART_PORT "/dev/ttyS1"      // UART1 sur Beaglebone Blue
// #define CAN_INTERFACE "can0"       // Interface CAN (changez en "can0" si vous utilisez une interface réelle)
// #define MAX_DATA_SIZE 256           // Taille maximale des données
// #define UART_BUFFER_SIZE 1024  

// volatile sig_atomic_t keep_running = 1;

// void sigint_handler(int sig)
// {
//     keep_running = 0;
// }

// // Fonction pour configurer le port série
// int configure_uart(int fd)
// {
//     struct termios options;
//     if (tcgetattr(fd, &options) < 0)
//     {
//         perror("tcgetattr");
//         return -1;
//     }

//     // Configurer la vitesse de transmission (9600 bauds, 8 bits, sans parité, 1 bit d'arrêt)
//     cfsetispeed(&options, B9600);
//     cfsetospeed(&options, B9600);

//     // Désactivation du contrôle de flux, pas de parité, 8 bits de données, 1 bit d'arrêt
//     options.c_cflag &= ~PARENB;    // Pas de parité
//     options.c_cflag &= ~CSTOPB;    // 1 bit d'arrêt
//     options.c_cflag &= ~CSIZE;     // Masquer la taille des données
//     options.c_cflag |= CS8;        // 8 bits de données
//     options.c_cflag &= ~CRTSCTS;   // Désactiver le contrôle de flux matériel
//     options.c_cflag |= CREAD | CLOCAL; // Activer la réception et ignorer les lignes de contrôle modem

//     options.c_iflag &= ~(IXON | IXOFF | IXANY); // Désactiver le contrôle de flux logiciel
//     options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Mode non canonique, pas d'écho, pas de signaux

//     options.c_oflag &= ~OPOST; // Pas de traitement de sortie

//     options.c_cc[VMIN] = 1;
//     options.c_cc[VTIME] = 0; // Timeout en dixièmes de seconde (ici, 1 seconde)

//     // Appliquer les modifications
//     if (tcsetattr(fd, TCSANOW, &options) < 0)
//     {
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
//     if (n < 0)
//     {
//         perror("read");
//         return -1;
//     }
//     return n;
// }

// // Fonction pour envoyer des données via UART
// int write_uart(int fd, const char *data, size_t size)
// {
//     int n = write(fd, data, size);
//     if (n < 0)
//     {
//         perror("write");
//         return -1;
//     }
//     return n;
// }

// // Fonction pour ouvrir le socket CAN
// int open_can_socket(const char *can_interface)
// {
//     int s;
//     struct ifreq ifr;
//     struct sockaddr_can addr;

//     // Ouvrir le socket
//     s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     if (s < 0)
//     {
//         perror("socket");
//         return -1;
//     }

//     // Spécifier l'interface CAN
//     strcpy(ifr.ifr_name, can_interface);
//     if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
//     {
//         perror("ioctl");
//         close(s);
//         return -1;
//     }

//     // Lier le socket à l'interface CAN
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;
//     if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
//     {
//         perror("bind");
//         close(s);
//         return -1;
//     }

//     return s;
// }

// // Fonction pour lire les données du CAN
// int read_can(int can_socket, struct can_frame *frame)
// {
//     int nbytes = read(can_socket, frame, sizeof(struct can_frame));
//     if (nbytes < 0)
//     {
//         perror("read can");
//         return -1;
//     }
//     else if (nbytes < sizeof(struct can_frame))
//     {
//         fprintf(stderr, "Trame CAN incomplète\n");
//         return -1;
//     }
//     return nbytes;
// }

// // Fonction pour envoyer des données via CAN
// int write_can(int can_socket, struct can_frame *frame)
// {
//     int nbytes = write(can_socket, frame, sizeof(struct can_frame));
//     if (nbytes < 0)
//     {
//         perror("write can");
//         return -1;
//     }
//     else if (nbytes < sizeof(struct can_frame))
//     {
//         fprintf(stderr, "Écriture CAN incomplète\n");
//         return -1;
//     }
//     return nbytes;
// }

// // Fonction pour envoyer des données sur le CAN
// void send_data_over_can(int can_socket, char couleur, float poids, int position, const char *mode)
// {
//    struct can_frame frame = {0};
//     frame.can_id = 0x123; // Identifiant CAN (à ajuster si nécessaire)

//     // Encode variables into CAN frame data
//     // data[0]: couleur (char)
//     // data[1-2]: poids (uint16_t in grammes)
//     // data[3]: position (uint8_t)
//     // data[4]: mode (char)

//     frame.can_dlc = 5; // Nombre d'octets de données

//     frame.data[0] = (u_int8_t)couleur;

//     u_int16_t poids_centieme = (u_int16_t)(poids);
//     frame.data[1] = (poids_centieme >> 8) & 0xFF;
//     frame.data[2] = poids_centieme & 0xFF;

//     frame.data[3] = (u_int8_t)position;

//     frame.data[4] = mode[0]; // Premier caractère du mode

//     // Envoyer la trame CAN
//     if (write_can(can_socket, &frame) > 0)
//     {
//         printf("Données envoyées sur le CAN: Couleur=%c, Poids=%.2f kg, Position=%d, Mode=%s\n",
//                couleur, poids, position, mode);
//     }
//     else
//     {
//         fprintf(stderr, "Erreur lors de l'envoi des données sur le CAN\n");
//     }
// }

// // Fonction pour recevoir des données du CAN et les traiter
// void receive_data_from_can(int can_socket, int uart_fd)
// {
//     struct can_frame frame;
//     int nbytes = read_can(can_socket, &frame);
//     if (nbytes > 0)
//     {
//         if (frame.can_dlc >= 5)
//         {
//             char couleur = frame.data[0];
//             int poids = (frame.data[1] << 8) | frame.data[2];
//             char position = frame.data[3];
//             char mode_char = frame.data[4];

//             float poids_g = poids;
             
//             char mode[16];
//             switch (mode_char)
//             {
//                 case 'T':
//                     strcpy(mode, "Test");
//                     break;
//                 case 'E':
//                     strcpy(mode, "Erreur");
//                     break;
//                 case 'A':
//                     strcpy(mode, "Attente"); 
//                     break;
//                 // Ajouter d'autres cas si nécessaire
//             }

//             printf("Couleur: %c, Poids: %.2f g, Position: %c, Mode: %c\n", couleur, poids_kg, position, mode);

//             // Préparer les données à envoyer sur l'UART
//             char uart_buffer[256];
//             snprintf(uart_buffer, sizeof(uart_buffer), "$%c,%.2f,%c,%c", couleur, poids_kg, position, mode);

//             write_uart(uart_fd, uart_buffer, strlen(uart_buffer));
//             printf("Données du CAN envoyées sur l'UART: %s\n", uart_buffer);
//         }
//         else
//         {
//             fprintf(stderr, "Trame CAN trop courte\n");
//         }
//     }
//     else
//     {
//         // Erreur ou aucune donnée
//     }
// }

// int main()
// {
//         // Gestion du signal Ctrl+C pour une sortie propre
//     signal(SIGINT, sigint_handler);

//     int fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd < 0)
//     {
//         perror("open UART");
//         return -1;
//     }

//     // Configurer le port série
//     if (configure_uart(fd) < 0)
//     {
//         close(fd);
//         return -1;
//     }



//     pid_t pid = fork();

//     if (pid < 0)
//     {
//         // Erreur lors du fork
//         perror("fork");
//         close(fd);
//         return -1;
//     }
//     else if (pid > 0)
//     {
//         // Processus parent: lire depuis l'UART et envoyer sur le CAN
//         // Ouvrir le socket CAN
//         int can_socket = open_can_socket(CAN_INTERFACE);
//         if (can_socket < 0)
//         {
//          close(fd);
//          return -1;
//         }
//         char uart_rx_buffer[UART_BUFFER_SIZE];
//         int uart_rx_length = 0;

//         while (keep_running)
//         {
//             char temp_buffer[256];
//             int bytes_received = read_uart(fd, temp_buffer, sizeof(temp_buffer));
//             if (bytes_received > 0)
//             {
//                 // Ajouter les données reçues au buffer de réception
//                 if (uart_rx_length + bytes_received < UART_BUFFER_SIZE)
//                 {
//                     memcpy(uart_rx_buffer + uart_rx_length, temp_buffer, bytes_received);
//                     uart_rx_length += bytes_received;
//                     uart_rx_buffer[uart_rx_length] = '\0'; // Assurer la terminaison de la chaîne

//                     // Rechercher des messages complets dans le buffer
//                     char *start_ptr = strchr(uart_rx_buffer, '$');
//                     while (start_ptr != NULL)
//                     {
//                         char *end_ptr = strchr(start_ptr, '\n');
//                         if (end_ptr != NULL)
//                         {
//                             // Un message complet a été trouvé
//                             size_t message_length = end_ptr - start_ptr + 1;
//                             char message[256];
//                             memcpy(message, start_ptr, message_length);
//                             message[message_length] = '\0';

//                             // Parser le message
//                             char couleur;
//                             float poids;
//                             int position;
//                             char mode[16];

//                             // Format des données: "$Couleur,Poids,Position,Mode\n"
//                             int parsed_items = sscanf(message, "$%c,%f,%d,%15s", &couleur, &poids, &position, mode);
//                             if (parsed_items == 4)
//                             {
//                                 // Envoyer les variables sur le CAN
//                                 send_data_over_can(can_socket, couleur, poids, position, mode);
//                                 printf("Données de l'UART envoyées sur le CAN: %s\n", message);
//                             }
//                             else
//                             {
//                                 fprintf(stderr, "Erreur de format des données UART: %s\n", message);
//                             }

//                             // Supprimer le message traité du buffer
//                             uart_rx_length -= (end_ptr - uart_rx_buffer) + 1;
//                             memmove(uart_rx_buffer, end_ptr + 1, uart_rx_length);
//                             uart_rx_buffer[uart_rx_length] = '\0';

//                             // Rechercher le prochain message
//                             start_ptr = strchr(uart_rx_buffer, '$');
//                         }
//                         else
//                         {
//                             // Pas de fin de message, attendre plus de données
//                             break;
//                         }
//                     }
//                 }
//                 else
//                 {
//                     // Buffer overflow, réinitialiser le buffer
//                     fprintf(stderr, "Overflow du buffer UART, réinitialisation du buffer\n");
//                     uart_rx_length = 0;
//                 }
//             }
//             else if (bytes_received == 0)
//             {
//                 // Aucune donnée reçue
//             }
//             else
//             {
//                 fprintf(stderr, "Erreur lors de la lecture des données UART\n");
//             }
//             usleep(100000); // 100 ms
//         }
//         // Attendre que le processus enfant se termine
//         wait(NULL);
//         close(fd);
//         close(can_socket);
//         printf("Processus parent terminé.\n");
//     }
//     else
//     {   
//         // Processus enfant: lire depuis le CAN et envoyer sur l'UART
//         // Ouvrir le socket CAN
//         int can_socket = open_can_socket(CAN_INTERFACE);
//         if (can_socket < 0)
//         {
//             close(fd);
//             return -1;
//         }

//         while (keep_running)
//         {
//             receive_data_from_can(can_socket, fd);
//             usleep(100000); // 100 ms
//         }
//         close(fd);
//         close(can_socket);
//         printf("Processus enfant terminé.\n");
//     }

//     return 0;
// }

/**
 * @file    Passerelle_Poste_de_commande.c
 * 
 * @brief   Application de communication entre UART et CAN. 
 *          Le programme lit les données UART et les transmet au bus CAN, et inversement.
 *          Permet la gestion des messages de commandes et des réponses via des trames CAN et UART.
 * 
 *          - UART utilisé : /dev/ttyS1 (Beaglebone Blue)
 *          - Interface CAN : can0
 *          - Gestion des signaux pour une terminaison propre
 *          - Communication en mode non canonique pour l'UART
 * 
 * @author  Émie-Jeanne Dupuis
 * @date    2024-11-21
 * 
 */

#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/wait.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>

#define UART_PORT "/dev/ttyS1"     // UART1 sur Beaglebone Blue
#define CAN_INTERFACE "can0"       // Interface CAN
#define MAX_DATA_SIZE 256          // Taille maximale des données CAN
#define UART_BUFFER_SIZE 1024      // Taille maximale du tampon UART


// Définition des IDs CAN utilisés pour identifier les messages
typedef enum
{
    Commandant = 0x120,
        com_mode,
        com_alarm,
        com_conversion,

    
    Station_Pese = 0x140,
        bal_mode,
        bal_poids,
        bal_conversion,
    
    Centre_tri = 0x150,
        ct_couleur,
        ct_mode,
    
    Gestion_transport = 0x160,
        gt_position,
        gt_statut,
} CAN_ID;


volatile sig_atomic_t keep_running = 1;


// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Gère le signal SIGINT pour permettre une terminaison propre du programme.
//  Paramètres d'entrées       : sig - le signal reçu
//  Paramètres de sortie       : Aucun
//  Notes                      : Met à jour la variable globale `keep_running`.
// *************************************************************************************************
void sigint_handler(int sig)
{
    keep_running = 0;
}

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Configure le port UART pour une communication série.
//  Paramètres d'entrées       : fd - descripteur de fichier du port série
//  Paramètres de sortie       : 0 en cas de succès, -1 en cas d'erreur
//  Notes                      : Configure la vitesse, la parité, et d'autres paramètres série.
// *************************************************************************************************
int configure_uart(int fd)
{
    struct termios options;

    // Récupère les paramètres actuels du port série
    if (tcgetattr(fd, &options) < 0)
    {
        perror("tcgetattr");
        return -1;
    }

    // Configurer la vitesse de transmission (9600 bauds, 8 bits, sans parité, 1 bit d'arrêt)
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Désactivation du contrôle de flux, pas de parité, 8 bits de données, 1 bit d'arrêt
    options.c_cflag &= ~PARENB;    // Pas de parité
    options.c_cflag &= ~CSTOPB;    // 1 bit d'arrêt
    options.c_cflag &= ~CSIZE;     // Masquer la taille des données
    options.c_cflag |= CS8;        // 8 bits de données
    options.c_cflag &= ~CRTSCTS;   // Désactiver le contrôle de flux matériel
    options.c_cflag |= CREAD | CLOCAL; // Activer la réception et ignorer les lignes de contrôle modem

    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Désactiver le contrôle de flux logiciel
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Mode non canonique, pas d'écho, pas de signaux

    options.c_oflag &= ~OPOST; // Pas de traitement de sortie

    options.c_cc[VMIN] = 1;  // Lire au moins 1 caractère
    options.c_cc[VTIME] = 0; // Pas de timeout

    // Applique les nouveaux paramètres
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        perror("tcsetattr");
        return -1;
    }

    // Vider les tampons d'entrée et de sortie
    tcflush(fd, TCIOFLUSH);

    return 0;
}


// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Ouvre et configure un socket pour l'interface CAN.
//  Paramètres d'entrées       : can_interface - nom de l'interface CAN (ex. "can0")
//  Paramètres de sortie       : Descripteur de socket en cas de succès, -1 en cas d'erreur
//  Notes                      : Lier le socket au nom d'interface CAN spécifié.
// *************************************************************************************************
int open_can_socket(const char *can_interface)
{
    // Descripteur de socket
    int s;
    // Structure pour configurer l'interface réseau
    struct ifreq ifr;
    // Structure pour lier l'adresse CAN
    struct sockaddr_can addr;

    // Ouvrir un socket CAN pour communiquer avec l'interface CAN
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        perror("socket");
        return -1;
    }
    
    // Configure le nom de l'interface CAN à utiliser
    strcpy(ifr.ifr_name, can_interface);

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        close(s);
        return -1;
    }

    // Lier le socket à l'interface CAN
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(s);
        return -1;
    }

    return s; // Retourne le descripteur du socket en cas de succès
}

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Gère les données reçues via UART et les transmet sur le CAN.
//  Paramètres d'entrées       : uart_fd - descripteur UART, can_socket - descripteur CAN
//  Paramètres de sortie       : Aucun
//  Notes                      : Identifie les commandes spécifiques et envoie des trames CAN correspondantes.
// *************************************************************************************************
void handle_uart_to_can(int uart_fd, int can_socket) 
{
    char uart_buffer[UART_BUFFER_SIZE];
    struct can_frame frame = {0}; // Initialiser la trame CAN

    // Lire les données du port UART
    int bytes_read = read(uart_fd, uart_buffer, sizeof(uart_buffer) - 1);

    if (bytes_read > 0)
    {
        // Ajouter un caractère de fin de chaîne pour s'assurer que les données sont bien une chaîne de caractères
        uart_buffer[bytes_read] = '\0';  
        // printf("Reçu via UART : %s\n", uart_buffer);
        
        if (strstr(uart_buffer, "$Alarme\n")) 
        {
             frame.can_id = com_alarm;
             frame.can_dlc = strlen("Alarme");
             strncpy((char *)frame.data, "Alarme", frame.can_dlc);
             write(can_socket, &frame, sizeof(frame));
             printf("Message d'alarme envoyé sur le CAN\n");
        } 
        else if (strstr(uart_buffer, "$Start\n")) 
        {
             frame.can_id = com_mode;
             frame.can_dlc = strlen("Start");
             strncpy((char *)frame.data, "Start", frame.can_dlc);
             write(can_socket, &frame, sizeof(frame));
             printf("Mode opération envoyé sur le CAN\n");
        } 
        else if (strstr(uart_buffer, "$Arret\n")) 
        {
            
             frame.can_id = com_mode;
             frame.can_dlc = strlen("Arret");
             strncpy((char *)frame.data, "Arret", frame.can_dlc);
             write(can_socket, &frame, sizeof(frame));
             printf("Mode arrêt envoyé sur le CAN\n");
        }
        else if (strstr(uart_buffer, "$Grammes\n")) 
        { 
             frame.can_id = com_conversion;
             frame.can_dlc = strlen("Grammes");
             strncpy((char *)frame.data, "Grammes", frame.can_dlc);
             write(can_socket, &frame, sizeof(frame));
             printf("Demande de conversion en grammes envoyé sur le CAN\n");
        } 
        else if (strstr(uart_buffer, "$Onces\n")) 
        {
             // Préparer une trame CAN pour demander une conversion en onces
             frame.can_id = com_conversion;
             frame.can_dlc = strlen("Onces");
             strncpy((char *)frame.data, "Onces", frame.can_dlc);
             write(can_socket, &frame, sizeof(frame));
             printf("Demande de conversion en onces envoyé sur le CAN\n");
        } 
    }
  }

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Gère les données reçues via CAN et les transmet sur l'UART.
//  Paramètres d'entrées       : can_socket - descripteur CAN, uart_fd - descripteur UART
//  Paramètres de sortie       : Aucun
//  Notes                      : Décode les messages CAN pour identifier et transmettre les informations via UART.
// *************************************************************************************************
void handle_can_to_uart(int can_socket, int uart_fd) 
{
    const char *mode;       // Variable pour stocker le mode reçu
    struct can_frame frame; // Structure pour contenir un message CAN

    // Lire un message CAN
    if (read(can_socket, &frame, sizeof(frame)) > 0) 
    {
        char uart_msg[256];

        // printf("ID CAN : 0x%X, DLC : %d, Données : ", frame.can_id, frame.can_dlc);
        // for (int i = 0; i < frame.can_dlc; i++) 
        // {
        //     printf("%02X ", frame.data[i]);
        // }
        // printf("\n");

        switch (frame.can_id) 
        {     

          case ct_mode:
          {
            if (strcmp((char *)frame.data, "Test") == 0) 
            {
                mode = "Test";
                printf("Mode Test");
                printf("\n");
            } 
            else if (strcmp((char *)frame.data, "Erreur") == 0) 
            {
                mode = "Erreur";
                printf("Mode Erreur");
                printf("\n");
            } 
            else if (strcmp((char *)frame.data, "Attente") == 0) 
            {
                mode = "Attente";
                printf("Mode Attente");
                printf("\n");
            } 
            else if (strcmp((char *)frame.data, "Ope") == 0) 
            {
                mode = "Operation";
                printf("Mode Operation");
                printf("\n");
            }  
            else if (strcmp((char *)frame.data, "Arret") == 0) 
            {
                mode = "Arret";
                printf("Mode Arret");
                printf("\n");
            } 
            snprintf(uart_msg, sizeof(uart_msg), "Mode:%s\n", mode);
            write(uart_fd, uart_msg, strlen(uart_msg));
            break;
          }
            case ct_couleur:
                snprintf(uart_msg, sizeof(uart_msg), "$Couleur,%c\n", frame.data[0]);
                printf("Caractère: %c\n", frame.data[0]);        
                write(uart_fd, uart_msg, strlen(uart_msg));
                break;
            case bal_poids:
                snprintf(uart_msg, sizeof(uart_msg), "$Poids,%s\n", frame.data);
                write(uart_fd, uart_msg, strlen(uart_msg));
                break;
            case gt_position:
                printf("Valeur brute reçue : 0x%X\n", frame.data[0]); 
                snprintf(uart_msg, sizeof(uart_msg), "$Position,%c\n", frame.data[0] + '0');
                write(uart_fd, uart_msg, strlen(uart_msg));
                break; 
            default:
                printf("ID CAN non géré : 0x%x\n", frame.can_id);
                break;
        }
    }
}


int main()
{
    signal(SIGINT, sigint_handler);
    
    // Ouvre le port UART en mode lecture/écriture
    int uart_fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd < 0) 
    {
        perror("open UART");
        return -1;
    }
    
    // Configure les paramètres du port UART
    if (configure_uart(uart_fd) < 0) 
    {
        close(uart_fd);
        return -1;
    }
    
    // Ouvre une socket CAN
    int can_socket = open_can_socket(CAN_INTERFACE);
    if (can_socket < 0) 
    {
        // Ferme le descripteur UART si l'ouverture de la socket CAN échoue
        close(uart_fd);
        return -1;
    }
    
    // Boucle principale pour lire et traiter les données jusqu'à ce que le signal d'arrêt soit reçu
    while (keep_running) 
    {

        fd_set read_fds; // Prépare un ensemble de descripteurs pour attendre des données
        FD_ZERO(&read_fds); // Initialise l'ensemble à vide
        FD_SET(uart_fd, &read_fds);  // Ajoute le descripteur UART
        FD_SET(can_socket, &read_fds);  // Ajoute le descripteur CAN
        
        // Détermine le descripteur avec la plus grande valeur
        int max_fd = (uart_fd > can_socket) ? uart_fd : can_socket;
        
        // Configure un délai d'attente (timeout) de 1 seconde
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        // Attend des données sur l'un des descripteurs (UART ou CAN)
        int ret = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        
        // Si des données sont disponibles
        if (ret > 0) 
        {
            // Si des données sont disponibles sur UART, les traite et les envoie vers CAN
            if (FD_ISSET(uart_fd, &read_fds)) 
            {
              handle_uart_to_can(uart_fd, can_socket);
            }

            // Si des données sont disponibles sur CAN, les traite et les envoie vers UART 
            if (FD_ISSET(can_socket, &read_fds)) 
            {
              handle_can_to_uart(can_socket, uart_fd);
            }
        }
    }

    close(uart_fd);    // Ferme le port UART
    close(can_socket); // Ferme la socket CAN

    return 0;
}







