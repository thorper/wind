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

  ros::Publisher wind_pub = n.advertise<wind::WindStamped>("wind_data", 1000);

  ros::Rate loop_rate(400);


  int fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(fd == -1)            // Error Checking 
           printf("\n  Error! in Opening ttyUSB0  ");
    else
           printf("\n  ttyUSB0 Opened Successfully ");

//------------------------------------Termios-------------------------------------------
  //Setting the Port Settings
  //www.cmrr.umn.edu/~strupp/serial.html#2_3_1
  struct termios SerialPortSettings;

  tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port 
  cfsetispeed(&SerialPortSettings,B57600); // Set Read  Speed as 19200                       
  cfsetospeed(&SerialPortSettings,B57600); // Set Write Speed as 9600                             
  
  // Even parity (7E1):
  // SerialPortSettings.c_cflag |= PARENB;
  // SerialPortSettings.c_cflag &= ~PARODD;
  // SerialPortSettings.c_cflag &= ~CSTOPB;
  // SerialPortSettings.c_cflag &= ~CSIZE;
  // SerialPortSettings.c_cflag |= CS7; 

  // No parity (8N1):
  SerialPortSettings.c_cflag &= ~PARENB;
  SerialPortSettings.c_cflag &= ~CSTOPB;
  SerialPortSettings.c_cflag &= ~CSIZE;
  SerialPortSettings.c_cflag |= CS8;                               

  // SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                         
  SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines        
  
  
  // SerialPortSettings.c_iflag |= (IXON | IXOFF | IXANY);// Enable XON/XOFF flow control both i/p and o/p 
  SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);// Disable XON/XOFF flow control both i/p and o/p 
  
  //SerialPortSettings.c_iflag &= ~(IXANY); 
  
  // SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);// raw mode                             
  SerialPortSettings.c_iflag |= (ICANON | ECHO | ECHOE | ISIG);// Cannonical mode                             

  SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing

  // Setting Time outs
  SerialPortSettings.c_cc[VMIN] = 0; // Read at least 10 characters 21
  SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly

  if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
        printf("\n  ERROR ! in Setting attributes");
  else
        printf("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none"); 
//------------------------------------Termios-------------------------------------------


  int count = 0;
  while (ros::ok())
  {

    ros::Time sonic_start_time = ros::Time::now();

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    wind::WindStamped msg;


     /*------------------------------- Read data from serial port -----------------------------*/
  try{
    char read_buffer[22];
    int  bytes_read = 0;    // Number of bytes read by the read() system call 
    int i = 0;


    int bytes_avail = 0;
    ioctl(fd, FIONREAD, &bytes_avail);
    printf("\n  bytes on input line is: %d\n", bytes_avail);

    // Error Checking

    if(bytes_avail < 21) 
    { 
      int bytes_avail2 = 0;  
      printf("\n  S\n"); 
      usleep(5000); // microseconds at least (a+b) for sonic to take 1 sample
                     // Data aq. time (a) = 5000 microsec/sample 
                     // Anemometer process and put on lines (b) = 3000 microsec/sample
                     // Sonic Anemometer Operators Manual pg 11 (8000)
      ioctl(fd, FIONREAD, &bytes_avail2); ///////////
      printf("\n  bytes on input line [NAPPED] is: %d\n", bytes_avail2); ///////////

      if(bytes_avail2 < 21){   
      printf("\n  SS\n");  
      throw 0;
      }
    }    
    
    // Reading Line
    bytes_read = read(fd,read_buffer,sizeof(read_buffer)); // Read the data 
      tcflush(fd, TCIFLUSH); //Always flush after reading

    // Error Checking the Reading Line
    if(bytes_read == -1){    
        printf("\n  R\n");
        throw 0;
    }
    
    // double t_from_anemo = ros::Time::now().toSec();

    // ROS_INFO("Time to and from Anemometer is %f seconds",t_from_anemo-t_to_anemo);

    // printf("Read Buffer is: ->%s<-",read_buffer);
    // printf("\n\n  Bytes Rxed -%d", bytes_read); // Print the number of bytes read   //COMMENTED OUT FOR SPEED
    // printf("\n\n  ");                                                               //COMMENTED OUT FOR SPEED

    //Print on one line--------------------------------------------------------
    for(i=0;i<bytes_read;i++)  //printing only the received characters
        printf("%c",read_buffer[i]);
    printf("\n +----------------------------------+\n\n\n");    //COMMENTED OUT FOR SPEED

        //Parse out U,V,W,T (with decimal point)
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

    ROS_INFO("U is %.6s m/s",U);
    ROS_INFO("V is %.6s m/s",V);
    ROS_INFO("W is %.6s m/s",W);
    ROS_INFO("T is %.6s deg C",T);

    //Convert char arrays into doubles (X_d) to publish
    double u = atof(U);
    double v = atof(V);
    double w = atof(W);
    double t = atof(T);

// #Two-integer timestamp that is expressed as:
// # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
// # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
// # time-handling sugar is provided by the client library
// time stamp


    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    

    msg.seq = count;
    msg.stamp = sonic_start_time;
    msg.frame_id = '0';
    msg.u = u;
    msg.v = v;
    msg.w = w;
    msg.temp = t;

    wind_pub.publish(msg);

  }catch(int x)
  {
  tcflush(fd, TCIFLUSH); 
  }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}