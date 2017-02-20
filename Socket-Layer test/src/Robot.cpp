#include <iostream>
#include <memory>
#include <string>


#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <open_sockets.h>
#include <packets.h>
#include <inet.h>


class Robot: public frc::SampleRobot {
	frc::RobotDrive myRobot { 3, 2, 1, 0 };
	frc::Joystick stick { 0 };
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";


public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		myRobot.SetExpiration(0.1);

	}
		 float x[3];


		  int					sockfd=0;             /* listen socket file descriptor */
		  struct sockaddr_in	cli_addr;           /* write-to-client socket address */
		  char                  inbuffer[MAXLINE];  /* incoming data */
		  unsigned short        packid = 0;         /* incoming packet ID */
		  int                   n=0;                  /* packet length */
		  struct track_packet   track;              /* a track packet */


	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		// create socket to receive zed tracking data
		if (open_serverside_socket(&sockfd)) {
		    fprintf(stderr,"Can't open socket\n");
		    exit(1);
		  } else {
			  std::cout << "Socket opened on "<< sockfd << std::endl;
		  }

		// go start zed on jetson
		system("ssh ubuntu@10.56.90.48 \"/home/ubuntu/zed > /dev/null 2>&1 &\"");
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

/*	void Autonomous() {
		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
			std::cout << "Running custom Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.Drive(-0.5, 1.0); // spin at half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot.Drive(0.0, 0.0);  // stop robot
		} else {
			// Default Auto goes here
			std::cout << "Running default Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.Drive(-0.5, 0.0); // drive forwards half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot.Drive(0.0, 0.0);  // stop robot
		}
	}
*/

	/*
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() override {
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			// drive with arcade style (use right stick)
			myRobot.ArcadeDrive(stick);

			// read from zed socket
		    n = get_packet(sockfd, &cli_addr, inbuffer);

		    if (n>0) { // There was a message!

		    /* Find packet type */

		    memcpy(&packid,inbuffer,sizeof(track.packet_id));
		    packid = ntohs(packid);

		/* deal with that packet */

		    switch (packid) {
		    case PID_TRACK:
		      handle_track(inbuffer,n,&track);
		      std::cout <<"packet_id : " << track.packet_id << std::endl; /* print packet to stdout */
		      std::cout <<"sequence : "<< track.sequence << std::endl;
		      std::cout <<"region_id : "<< track.region_id << std::endl;
		      std::cout <<"range : "<< track.range << std::endl;
		      std::cout <<"x : "<< track.x << std::endl;
		      std::cout <<"y : "<< track.y << std::endl;
		      std::cout <<"aspect_ratio : "<< track.aspect_ratio << std::endl;
		      std::cout <<"\n"<< std::endl;
		      break;
		    default:
		      fprintf(stderr,"Unknown packet type %d and length %d\n",packid,n);

		    }






		    }



		}


			// wait for a motor update time
			frc::Wait(0.005);
	}


	/*
	 * Runs during test mode
	 */
	void Test() override {

	}
};



/*
packets.c
*/



void send_packet(int sockfd, struct sockaddr_in	*pserv_addr, char *buffer, int buffsiz)
{
  char	sendline[MAXLINE], recvline[MAXLINE + 1];
  int   n;

/* copy the data to be sent */

  memcpy(sendline,buffer,buffsiz);

/* Send the buffer */

  if (sendto(sockfd, sendline, buffsiz, 0, (struct sockaddr *) pserv_addr,
	     sizeof(*pserv_addr)) != buffsiz)
    fprintf(stderr,"dg_cli: sendto error on socket\n");

  return;
}

/*
   This routine fills a track packet with data, and copies that data
   into the space at outbuffer.  The size of the output buffer is returned.
 */
int pack_track(char *outbuffer, struct track_packet *outpacket)
/*    char *outbuffer;               buffer to be filled */
/*    struct track_packet *outpacket; the data packet */
{
  int buffsize;                     /* size of the buffer */

/* Fill packet to send */

/* convert to network byte order */

  outpacket->packet_id = htons(outpacket->packet_id);
  outpacket->sequence = htonl(outpacket->sequence);
  outpacket->region_id = htons(outpacket->region_id);
  outpacket->range = htonl(outpacket->range);
  outpacket->x = htons(outpacket->x);
  outpacket->y = htons(outpacket->y);
  outpacket->aspect_ratio = htonl(outpacket->aspect_ratio);

/* pack send info into the buffer */

  buffsize = sizeof(*outpacket);
  memcpy(outbuffer,outpacket,buffsize);
  return(buffsize);
}


int get_packet(int sockfd, struct sockaddr_in *pcli_addr, char *mesg)
/*     int		sockfd;          the socket to listen on */
/*     struct sockaddr_in	*pcli_addr;	 ptr to client sockaddr structure */
/*     char	        *mesg;           input buffer */
{
  int	       n;
  socklen_t clilen;

/* read a buffer from the socket */

  clilen = sizeof(*pcli_addr);
  n = (int)recvfrom(sockfd, mesg, MAXLINE, MSG_DONTWAIT, (struct sockaddr *)pcli_addr,
	       &clilen);
 // if (n < 0)
  //  fprintf(stderr,"dg_echo: recvfrom error");
  mesg[n] = 0;           /* null terminate */
  if (n>0) printf("From %s:\n",inet_ntoa(pcli_addr->sin_addr));
  return(n);
}

/* This routine deals with a track packet */

void handle_track(char *inbuffer, int n,struct track_packet *inpacket)
/*     char *inbuffer;               the input buffer */
/*    int n;                        the size of that buffer */
/*    struct track_packet *inpacket; a track packet */
{

/* Check that the packet size is correct, and fill the structure if it is */

  if (n != sizeof(*inpacket)) {
    fprintf(stderr,"Whoa!  %d is wrong packet size for this type (%d)\n",
	    n,sizeof(*inpacket));
    return;
  } else {
    memcpy(inpacket,inbuffer,n);
  }

/* Convert to local byte order */

  inpacket->packet_id = ntohs(inpacket->packet_id);
  inpacket->sequence = ntohl(inpacket->sequence);
  inpacket->region_id = ntohs(inpacket->region_id);
  inpacket->range = ntohl(inpacket->range);
  inpacket->x = ntohs(inpacket->x);
  inpacket->y = ntohs(inpacket->y);
  inpacket->aspect_ratio = ntohl(inpacket->aspect_ratio);
}





/*
open_sockets.c \/
*/

int open_clientside_socket(int *sockfd, struct sockaddr_in *serv_addr)

{
  struct sockaddr_in	cli_addr;   /* socket for the client to listen on */
  int flag_on  = 1;
  int flag_off = 0;

  /*
   * Fill in the structure "serv_addr" with the address of the
   * server that we want to send to.
   */

  memset((char *) serv_addr, 0, sizeof(*serv_addr));
  serv_addr->sin_family      = AF_INET;
  serv_addr->sin_addr.s_addr = inet_addr(SERV_HOST_ADDR);
  serv_addr->sin_port        = htons(SERV_UDP_PORT);

  /*
   * Open a UDP socket (an Internet datagram socket).
   */

  if ( (*sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    fprintf(stderr,"client: can't open datagram socket\n");
    return(-1);
  }

  /*
   * Bind a local address for the client to listen on.
   */

  memset((char *) &cli_addr, 0, sizeof(cli_addr));	/* zero out */
  cli_addr.sin_family      = AF_INET;
  cli_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  cli_addr.sin_port        = htons(CLI_UDP_PORT);
  if (bind(*sockfd, (struct sockaddr *) &cli_addr, sizeof(cli_addr)) < 0) {
    fprintf(stderr,"client: can't bind local address\n");
    return(-1);
  }

/* make this socket non-blocking, so we can retry if packets get lost */
  if (ioctl(*sockfd,FIONBIO,&flag_on) < 0)
    fprintf(stderr,"FIONBIO error - can't set non-blocking flag on socket\n");
  return(0);
}


/* This routine opens up the socket needed to listen for the client */

int open_serverside_socket(int *sockfd)
{
  struct sockaddr_in sock_addr;      /* the socket address */

  /*
   * Open a UDP socket (an Internet datagram socket).
   */

  if ( (*sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    fprintf(stderr,"server: can't open datagram socket");
    return(-1);
  }

  /*
   * Bind our local address so that the client can send to us.
   */

  memset((char *) &sock_addr, 0, sizeof(sock_addr));
  sock_addr.sin_family      = AF_INET;
  sock_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  sock_addr.sin_port        = htons(SERV_UDP_PORT);

  if (bind(*sockfd,
	   (struct sockaddr *) &sock_addr, sizeof(sock_addr)) < 0) {
    fprintf(stderr,"server: can't bind local address\n");
    return(-1);
  }

  return(0);
}


START_ROBOT_CLASS(Robot)
