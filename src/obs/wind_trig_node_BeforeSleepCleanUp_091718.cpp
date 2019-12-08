#include <ros/ros.h>
#include <sstream>

#include <std_msgs/String.h>
#include <wind/WindStamped.h>

#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include <sys/ioctl.h> //Use error checks for serial data

#include <Eigen/Dense>


using namespace Eigen;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wind_trig_node");
  ros::NodeHandle n;

  ros::Publisher wind_pub = n.advertise<wind::WindStamped>("wind_data",50);

  int fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK);
  // int fd = open("/dev/ttyUSB3",O_RDWR);

    if(fd == -1)            // Error Checking 
           ROS_INFO("\n  Error! in Opening ttyUSB0  ");
    else
           ROS_INFO("\n  ttyUSB0 Opened Successfully ");

  //------------------------------------Termios-------------------------------------------
  //Setting the Port Settings
  //www.cmrr.umn.edu/~strupp/serial.html#2_3_1
  struct termios SerialPortSettings;

  tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port 
  cfsetispeed(&SerialPortSettings,B57600); // Set Read  Speed                      
  cfsetospeed(&SerialPortSettings,B57600); // Set Write Speed                            


  if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
      ROS_INFO("\n  ERROR ! in Setting attributes");
  else
      ROS_INFO("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none"); 
  //------------------------------------Termios-------------------------------------------

  unsigned int count = 0;
  ros::Rate loop_rate(50);

  int mosi = 0; //master out slave in time (millisec) 0.0001
  int somi = 0; //slave out master in time (millisec) 0.0001
  int nsample = 1; //number of samples anemometer averages together (anemometer setting)
  int tmeas = nsample * 2; //time to take 1 wind measurement (ms)
  int trig2 = 0; // time to wait for serial data from a second trigger (ms) 0.0001

  //convert to microseconds for usleep()
  mosi = mosi * 1000;
  somi = somi * 1000;
  tmeas = tmeas * 1000;
  trig2 = trig2 * 1000;

  while (ros::ok())
  {
    //ROS_INFO("loop start");
    
    ros::Time sonic_start_time = ros::Time::now();
    wind::WindStamped msg;

      /*------------------------------- Write data to serial port -----------------------------*/
    char write_buffer[2] = {'*'};  // Buffer containing characters to write into port 
    int  bytes_written = 0;    /* Value for storing the number of bytes written to the port */

    // ROS_INFO("before write");
    bytes_written = write(fd,write_buffer,sizeof(write_buffer)); //use write() to send data to port
    //usleep(mosi+tmeas+somi);
    ros::Duration(0.0001).sleep();
    // ROS_INFO("after write");

    // usleep(1000);
    if(bytes_written == -1)
    { 
      ROS_INFO("\n  Error Writing to Serial Port!\n");          
    }
    int b = 0;
    while (bytes_written == -1)
    {
      //use write() to send data to port
      ROS_INFO("Trying # %d attempt at writing to port\n",b);
      // ROS_INFO("checking write");
      bytes_written = write(fd,write_buffer,sizeof(write_buffer));
      ros::Duration(0.0001).sleep();
      // ROS_INFO("checked write");
      b++;
    }

    /*------------------------------- Read data from serial port -----------------------------*/

    char read_buffer[20]; //22 -> 21 -> 20
    int  bytes_read = 0;    // Number of bytes read by the read() system call 
    int i = 0;
  
    int bytes_avail = 0;
    
    //ROS_INFO("Checking line");
    // usleep(100);
    ioctl(fd, FIONREAD, &bytes_avail);
    // usleep(somi);
    ros::Duration(0.0001).sleep();
    //ROS_INFO("Checked line");

    // printf("\n  bytes on input line is: %d\n", bytes_avail);

    // Error Checking the Reading Line 1
    int k = 0;
    while(bytes_avail < 22) //21 -> 20 (22 is a MAGICAL number..? why? Who knows!)
    { 
      // ROS_INFO("Trigger # %d",k);  
      // if(bytes_avail == 0) //Nothing on line send another trigger
      // {
        bytes_written = write(fd,write_buffer,sizeof(write_buffer)); //use write() to send data to port
        // usleep(mosi+tmeas+somi);
        ros::Duration(0.0001).sleep();
        // ROS_INFO("Trigger %d's, second trigger sent",k);
      // }
      ioctl(fd, FIONREAD, &bytes_avail); //wait for line to fill up to 22 bytes
      // usleep(somi);
      ros::Duration(0.0001).sleep();
      // ROS_INFO("Line Check # %d bytes available: %d",k,bytes_avail);
      // printf("\n # %d check, bytes on input line: %d\n",k,bytes_avail); 
      k++;      
    }

    // Reading Line
    //ROS_INFO("Before Read\n");
    bytes_read = read(fd,read_buffer,sizeof(read_buffer)); // Read the data 
    ros::Duration(0.0001).sleep();
    // usleep(somi);
    //ROS_INFO("After Read\n");
    tcflush(fd, TCIFLUSH); //Always flush after reading

    // Error Checking the Reading Line 2
    if(bytes_read == -1)
    {    
      ROS_INFO("\n  Error Reading the Serial Port!\n");
    }

    //Print on one line C++ -------------------------------------------------------
        // for(i=0;i<bytes_read;i++)
        // printf("%c",read_buffer[i]);
    //Print on one line ROS--------------------------------------------------------
    //ROS_INFO("read_buffer is:");  
    //ROS_INFO("%s",read_buffer);  

    /*------------------------- Make and Publish ROS Message --------------------------*/
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
    // printf("U is %s\n",U);  
    // printf("u is %f\n",u);

    msg.header.seq = count;
    msg.header.stamp = sonic_start_time;
    msg.header.frame_id = '0';

    // Matrix3d RBA, R2B, R2A;
    Matrix3d RBA, REB, REA;

    //Rotate 45 about x in DSW {B} frame
    //comes from rotx(45) in MATLAB
    RBA << 1,0,0,
           0,0.7071,-0.7071,
           0,0.7071,0.7071;
    // RBA << 1,0,0,
    //        0,1,0,
    //        0,0,1;
     
    // R2B << 0,-1,0,
    //        0,0,1,
    //       -1,0,0;
    // R2A = R2B*RBA; 

    REB << 0,-1,0,
           0,0,-1,
           1,0,0;
    REA = REB*RBA;        

    Vector3d DAwind(u,v,w);
    // Vector3d D2wind = R2A * DAwind;
    Vector3d DEwind = REA * DAwind;

    // msg.u = D2wind(0); //u
    // msg.v = D2wind(1); //v
    // msg.w = D2wind(2); //w
    msg.u = DEwind(0); //u
    msg.v = DEwind(1); //v
    msg.w = DEwind(2); //w
    msg.temp = t;
    //ROS_INFO("U is %s",U);
    //ROS_INFO("u is %f",msg.u);

    wind_pub.publish(msg);
    ros::spinOnce();    
    loop_rate.sleep();
    ++count;
    ROS_INFO("U: %f, V: %f, W: %f, T: %f",u,v,w,t);
    //ROS_INFO("loop end");
  }
  close(fd);
  printf("Port Closed\n"); 
  return 0;
}