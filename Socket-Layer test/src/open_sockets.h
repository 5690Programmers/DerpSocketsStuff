#include        <sys/ioctl.h> 

int open_clientside_socket(int *sockfd, struct sockaddr_in *serv_addr);
int open_serverside_socket(int *sockfd);
