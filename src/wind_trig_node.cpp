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
#include <cmath> 

//da.c_str()

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wind_trig_node");
  ros::NodeHandle n;

  ros::Publisher wind_pub = n.advertise<wind::WindStamped>("wind_data",50);

  string dda = "/dev/ttyUSB0"; //default laptop anemometer address
  string da;
  n.param("device_address",da,dda); //load rpi anemometer address from launch file
  int fd = open(da.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
  // int fd = open("/dev/ttyUSB1",O_RDWR | O_NOCTTY | O_NONBLOCK);
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
  int sr;
  n.param("sample_rate",sr,50); //load rpi sample sr, default laptop sample rate 50 hz
  ros::Rate loop_rate(sr);
  // ros::Rate loop_rate(50);

  double u_old = 0;
  double v_old = 0;
  double w_old = 0;
  double t_old = 0;

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
    ros::Duration(0.001).sleep();
    // ROS_INFO("after write");

    if(bytes_written == -1)
    { 
      ROS_INFO("\n  Error Writing to Serial Port!\n");    
      return 1;      
    }
    int b = 0;
    /*while (bytes_written == -1)
    {
      //use write() to send data to port
      ROS_INFO("Trying # %d attempt at writing to port\n",b);
      ROS_INFO("checking write");
      bytes_written = write(fd,write_buffer,sizeof(write_buffer));
      ros::Duration(0.0001).sleep();
      ROS_INFO("checked write");
      b++;
    }*/

    /*------------------------------- Read data from serial port -----------------------------*/

    char read_buffer[20]; //22 -> 21 -> 20
    int  bytes_read = 0;    // Number of bytes read by the read() system call 
    int i = 0;
  
    int bytes_avail = 0;
    
    //ROS_INFO("Checking line");
    ioctl(fd, FIONREAD, &bytes_avail);
    ros::Duration(0.0001).sleep();
    //ROS_INFO("Checked line");

    //ROS_INFO("bytes on input line is: %d", bytes_avail);

    // Error Checking the Reading Line 1
    int k = 0;
    while(bytes_avail < 22) //21 -> 20 (22 is a MAGICAL number..? why? Who knows!)
    { 
        bytes_written = write(fd,write_buffer,sizeof(write_buffer)); //use write() to send data to port
        ros::Duration(0.001).sleep();
        ioctl(fd, FIONREAD, &bytes_avail); //wait for line to fill up to 22 bytes
        ros::Duration(0.001).sleep();
        k++;  
       // ROS_INFO("retry #: %i",k); commented out for terminal to run faster
   
    }

    // Reading Line
    //ROS_INFO("Before Read");
    bytes_read = read(fd,read_buffer,sizeof(read_buffer)); // Read the data 
    ros::Duration(0.0001).sleep();
    //ROS_INFO("After Read");
    tcflush(fd, TCIFLUSH); //Always flush after reading

    // Error Checking the Reading Line 2
    if(bytes_read == -1)
    {    
      ROS_INFO("\n  Error Reading the Serial Port!\n");
      return 1;
    }

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

    //-----Outlier correction 

    double u_delta = abs(u-u_old);
    double v_delta = abs(v-v_old);
    double w_delta = abs(w-w_old);
    double t_delta = abs(t-t_old);

    unsigned int tol = 5; // 5 m/s or 5 deg C would be too much change over 30 hz

    if(count > 0 && (u_delta > tol || v_delta > tol || w_delta > tol || t_delta > tol)) 
    {
      //When any threshold reached the old reasonable value is passed into the new ones place
      // ROS_INFO("NOT-OK U old is: %f",u_old);
      // ROS_INFO("NOT-OK U new is: %f",u);
      u = u_old;
      v = v_old;
      w = w_old;
      t = t_old;
      ROS_INFO("Held old Value");
      // return 1;
    }

    u_old = u;
    v_old = v;
    w_old = w;
    t_old = t;

    // ROS_INFO("OK U old is: %f",u_old);
    // ROS_INFO("OK U new is: %f",u);

    msg.header.seq = count;
    msg.header.stamp = sonic_start_time;
    msg.header.frame_id = '0';

    Matrix3d RBA, R2B, R2A;

    //Rotate 45 about x in DSW {B} frame
    //comes from rotx(45) in MATLAB
   /* RBA << 1,0,0,
           0,0.7071,-0.7071,
           0,0.7071,0.7071;*/
    RBA << 1,0,0,//NO 45 deg clock for 2x4 testing
           0,1,0,
	   0,0,1;
    R2B << 0,-1,0,
           0,0,-1,
           1,0,0;
    R2A = R2B*RBA;        

    Vector3d DAwind(u,v,w);
    Vector3d D2wind = R2A * DAwind;

	double mag = sqrt(u*u + v*v + w*w);	

    msg.u = D2wind(0); //u
    msg.v = D2wind(1); //v
    msg.w = D2wind(2); //w
    msg.temp = t;
	msg.mag = mag;

    wind_pub.publish(msg);
    ros::spinOnce();    
    loop_rate.sleep();
    ++count;
    ROS_INFO("U: % f, V: % f, W: % f, T: % f, MAG: % f",u,v,w,t,mag);
    //ROS_INFO("loop end");
  }
  close(fd);
  printf("Port Closed\n"); 
  return 0;
}
