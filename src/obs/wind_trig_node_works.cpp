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
  ros::init(argc, argv, "wind_trig_node");
  ros::NodeHandle n;

  ros::Publisher wind_pub = n.advertise<wind::WindStamped>("wind_data",500);

  int fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK);
  // int fd = open("/dev/ttyUSB3",O_RDWR);

    if(fd == -1)            // Error Checking 
           printf("\n  Error! in Opening ttyUSB0  ");
    else
           printf("\n  ttyUSB0 Opened Successfully ");

  //------------------------------------Termios-------------------------------------------
  //Setting the Port Settings
  //www.cmrr.umn.edu/~strupp/serial.html#2_3_1
  struct termios SerialPortSettings;

  tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port 
  cfsetispeed(&SerialPortSettings,B57600); // Set Read  Speed                      
  cfsetospeed(&SerialPortSettings,B57600); // Set Write Speed                            


  if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
      printf("\n  ERROR ! in Setting attributes");
  else
      printf("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none"); 
  //------------------------------------Termios-------------------------------------------

  unsigned int count = 0;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    ROS_INFO("loop");
    
      ros::Time sonic_start_time = ros::Time::now();
      wind::WindStamped msg;

      /*------------------------------- Write data to serial port -----------------------------*/
    char write_buffer[2] = {'*'};  // Buffer containing characters to write into port 
 
      int  bytes_written = 0;    /* Value for storing the number of bytes written to the port */ 
      ROS_INFO("before write");
      bytes_written = write(fd,write_buffer,sizeof(write_buffer)); //use write() to send data to port
      ROS_INFO("after write");

      // usleep(1000);
      if(bytes_written == -1)
        { 
          printf("\n  Error Writing to Serial Port!\n");          
        }
        int b = 0;
        while (bytes_written == -1)
        {
          //use write() to send data to port
          printf("Trying # %d attempt at writing to port\n",b);
          // tcflush(fd, TCIFLUSH);
          ROS_INFO("checking write");
          bytes_written = write(fd,write_buffer,sizeof(write_buffer));
          ROS_INFO("checked write");

          b++;
        }


    /*------------------------------- Read data from serial port -----------------------------*/

      char read_buffer[22];
      int  bytes_read = 0;    // Number of bytes read by the read() system call 
      int i = 0;
  
      int bytes_avail = 0;
      
      ROS_INFO("Checking line");
      ioctl(fd, FIONREAD, &bytes_avail);
      ROS_INFO("Checked line");

      // printf("\n  bytes on input line is: %d\n", bytes_avail);

      // Error Checking the Reading Line 1
      int k = 0;
      while(bytes_avail < 21) 
      { 
      ROS_INFO("Trigger # %d",k);  
      bytes_written = write(fd,write_buffer,sizeof(write_buffer)); //use write() to send data to port
      usleep(1000);
      ioctl(fd, FIONREAD, &bytes_avail);
      ROS_INFO("Line Check # %d",k);
      // printf("\n # %d check, bytes on input line: %d\n",k,bytes_avail); 
      k++;      
      }

      // Reading Line
      ROS_INFO("Before Read\n");
      bytes_read = read(fd,read_buffer,sizeof(read_buffer)); // Read the data 
      ROS_INFO("After Read\n");
      tcflush(fd, TCIFLUSH); //Always flush after reading

      // Error Checking the Reading Line 2
      if(bytes_read == -1)
      {    
        printf("\n  Error Reading the Serial Port!\n");
      }

      //Print on one line--------------------------------------------------------
      for(i=0;i<bytes_read;i++)
          printf("%c",read_buffer[i]);

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
      // printf("U is %s\n",U);  
      double u = atof(U);
      // printf("u is %f\n",u);
      double v = atof(V);
      double w = atof(W);
      double t = atof(T);

      msg.seq = count;
      msg.stamp = sonic_start_time;
      msg.frame_id = '0';
      // msg.ss = 0;
      msg.u = u; //u
      msg.v = v; //v
      msg.w = w; //w
      msg.temp = t;
      ROS_INFO("U is %s",U);
      ROS_INFO("u is %f",msg.u);

      // loop_rate.sleep();
      wind_pub.publish(msg);
      ros::spinOnce();    
      loop_rate.sleep();
      ++count;
  } 
  return 0;
}