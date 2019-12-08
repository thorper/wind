#include <ros/ros.h>
#include <sstream>

#include <std_msgs/String.h>
#include <wind/WindStamped.h>

#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include <sys/ioctl.h> //Use error checks for serial data

int main(int argc, char **argv)
{

  ros::init(argc, argv, "wind_node");

  ros::NodeHandle n;

  ros::Publisher wind_pub = n.advertise<wind::WindStamped>("wind_data",1000);

  // int fd = open("/dev/ttyUSB0",O_RDONLY | O_NOCTTY);
  // int fd = open("/dev/ttyUSB0",O_RDONLY | O_NOCTTY | O_NONBLOCK);
  // int fd = open("/dev/ttyUSB0",O_RDONLY | O_NONBLOCK);

  int fd = open("/dev/ttyUSB0",O_RDONLY);
  

    if(fd == -1){            // Error Checking 
           printf("\n  Error! in Opening ttyUSB0  ");
         return 1;}
    else{
           printf("\n  ttyUSB0 Opened Successfully ");}
    
  //------------------------------------Termios-------------------------------------------
  //Setting the Port Settings
  //www.cmrr.umn.edu/~strupp/serial.html#2_3_1
  struct termios SerialPortSettings;

  tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port 
  cfsetispeed(&SerialPortSettings,B57600); // Set Read  Speed as 19200                       

  // // // No parity (8N1):
  // // SerialPortSettings.c_cflag &= ~PARENB;
  // // SerialPortSettings.c_cflag &= ~CSTOPB;
  // // SerialPortSettings.c_cflag &= ~CSIZE;
  // // SerialPortSettings.c_cflag |= CS8;                               

  // SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                         
  // // SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines        
  
  
  // SerialPortSettings.c_iflag |= (IXON | IXOFF | IXANY);// Enable XON/XOFF flow control both i/p and o/p 
  // // SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);// Disable XON/XOFF flow control both i/p and o/p 
  
  // // //SerialPortSettings.c_iflag &= ~(IXANY); 
  
  // SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);// raw mode                             
  // // // SerialPortSettings.c_iflag |= (ICANON | ECHO | ECHOE | ISIG);// Cannonical mode                             

  // SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing

  // // Setting Time outs
  // SerialPortSettings.c_cc[VMIN] = 36; // Read at least 10 characters 21
  // SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly (tenths of a second elapses between bytes)

  if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
        printf("\n  ERROR ! in Setting attributes");
  else
        printf("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none"); 


  //------------------------------------Termios-------------------------------------------


  unsigned int count = 0;

  // ros::Rate loop_rate(250);

  while (ros::ok())
  {

    ros::Time sonic_start_time = ros::Time::now();

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    wind::WindStamped msg;


     /*------------------------------- Read data from serial port -----------------------------*/
  // try{

    char read_buffer[21]; //21
    int  bytes_read = 0;    // Number of bytes read by the read() system call 
    int i = 0;

    int bytes_avail = 0;
    ioctl(fd, FIONREAD, &bytes_avail);
    // printf("\n  bytes on input line is: %d\n", bytes_avail);
    ROS_INFO("\n  bytes on input line is: %d\n", bytes_avail);

	while(bytes_avail < 21) 
	{ 

    // usleep(2500)  means sleep @ 400 hz
    // usleep(5000)  means sleep @ 200 hz
    // usleep(10000) means sleep @ 100 hz
    // usleep(20000) means sleep @ 50  hz    
    usleep(2500);

    ioctl(fd, FIONREAD, &bytes_avail);
    // printf("\n  bytes on input line is: %d\n", bytes_avail);
    // ROS_INFO("\n  Now, bytes on input line is: %d\n", bytes_avail);
	// bytes_avail = 0;	
	  // int bytes_avail2 = 0;  
	  // printf("\n  S\n"); 
	  // usleep(8000); // microseconds at least (a+b) for sonic to take 1 sample
	  //                // Data aq. time (a) = 5000 microsec/sample 
	  //                // Anemometer process and put on lines (b) = 3000 microsec/sample
	  //                // Sonic Anemometer Operators Manual pg 11
	  // ioctl(fd, FIONREAD, &bytes_avail2); ///////////
	  // printf("\n  bytes on input line [NAPPED] is: %d\n", bytes_avail2); ///////////

	  // if(bytes_avail2 < 21){   
	  // printf("\n  SS\n");  
	  // throw 0;
	  // }
	}  
  // ROS_INFO("Count is %d\n",count);
    // Reading Line
    bytes_read = read(fd,read_buffer,sizeof(read_buffer)); // Read the data 
      tcflush(fd, TCIFLUSH); //Always flush after reading


    // ROS_INFO("Bytes Read were: %d\n",bytes_read);

    // Error Checking the Reading Line
    if(bytes_read == -1){    
        printf("\n  R\n");
         // throw 0;
    }
 
    //Print on one line--------------------------------------------------------
    // for(i=0;i<bytes_read;i++)
    // {  //printing only the received characters
    //     printf("%c",read_buffer[i]);
    //     printf("\n +----------------------------------+\n\n\n");
    // }
        // printf("%c",read_buffer[i]);
    // printf("\n +----------------------------------+\n\n\n");    //COMMENTED OUT FOR SPEED



    // Parse out U,V,W,T (with decimal point)
    int word_length = 5; //6
    char U[] = {'0','0','0','0','0','0','\0'};
    char V[] = {'0','0','0','0','0','0','\0'};
    char W[] = {'0','0','0','0','0','0','\0'};
    char T[] = {'0','0','0','0','0','0','\0'};

    for(int j=0;j<word_length;j++)
      {
        if(j == 3) //Adds decimal point
        {
          U[j] = '.';
          V[j] = '.';
          W[j] = '.';
          T[j] = '.';
        }
        if(j < 3) //adds 10s and 1s places
        {
          U[j] = read_buffer[j];
          V[j] = read_buffer[j+5]; //7
          W[j] = read_buffer[j+10]; //14
          T[j] = read_buffer[j+15]; //21
        }
        else //adds 10ths and 100ths places
        {
          U[j+1] = read_buffer[j];
          V[j+1] = read_buffer[j+5]; 
          W[j+1] = read_buffer[j+10];
          T[j+1] = read_buffer[j+15];        
        }
      }


    //Convert char arrays into doubles (X_d) to publish
    double u = atof(U);
    double v = atof(V);
    double w = atof(W);
    double t = atof(T);

    msg.seq = count;
    msg.stamp = sonic_start_time;
    msg.frame_id = '0';
    msg.u = u;
    msg.v = v;
    msg.w = w;
    msg.temp = t;

    wind_pub.publish(msg);

  // }catch(int x)
  // {
  // tcflush(fd, TCIFLUSH); 
  // }

    ros::spinOnce();
    // ros::spin();

    // loop_rate.sleep();
    ++count;
  }

  return 0;
}