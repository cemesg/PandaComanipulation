#define GLM_ENABLE_EXPERIMENTAL
#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <iostream>
#define MAX 1024
#define PORT 12341
#define SA struct sockaddr
#include <HD/hd.h>
#include <time.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include<glm/glm.hpp>
#include<glm/gtc/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include<glm/common.hpp>
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
#endif
/* Holds data retrieved from HDAPI. */
typedef struct 
{
    HDboolean m_buttonState;       /* Has the device button has been pressed. */
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
    HDErrorInfo m_error;
} DeviceData;


float x=0.0;
float y =0.0;
float z = 0.0;


static DeviceData gServoDeviceData;
static HDdouble encoders[6];
static HDdouble transform[16];
glm::quat myQuat= glm::quat();

/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData)
{   
    int nButtons = 0;
    const HDdouble sphereRadius = 20.0;
    const hduVector3Dd spherePosition((x-0.5)*200,(y-1.2)*200,-z*200);
    
    // Stiffness, i.e. k value, of the sphere.  Higher stiffness results
    // in a harder surface.
    const double sphereStiffness = .12;

    hdBeginFrame(hdGetCurrentDevice());

    /* Retrieve the current button(s). */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    
    /* In order to get the specific button 1 state, we use a bitmask to
       test for the HD_DEVICE_BUTTON_1 bit. */
    gServoDeviceData.m_buttonState = 
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    //std::cout << " Button State " << bool(gServoDeviceData.m_buttonState) << std::endl;
        
    /* Get the current location of the device (HD_GET_CURRENT_POSITION)
       We declare a vector of three doubles since hdGetDoublev returns 
       the information in a vector of size 3. */
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, encoders);
    hdGetDoublev(HD_CURRENT_TRANSFORM,transform);

    glm::mat4 MyMatrix=glm::mat4();


    for(int i =0; i< 4; i++){

        for(int j =0; j<4; j++){
            MyMatrix[i][j]  = transform[i*4+j];
        }
    }
    //MyMatrix = glm::gtx::transform::rotate(MyMatrix, glm::radians(180.0), glm::vec3(0,1,0))
    myQuat = glm::quat_cast(MyMatrix);
    std::cout << std::endl;

    /* Also check the error state of HDAPI. */
    gServoDeviceData.m_error = hdGetError();
     hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);
    

    hduVector3Dd f(x,y,z);
    //std::cout << f << std::endl;
    hdSetDoublev(HD_CURRENT_FORCE, f);

    /* Copy the position into our device_data tructure. */
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;    
}



/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{    
    DeviceData *pDeviceData = (DeviceData *) pUserData;

    memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}  

void printHelp(void)
{
    static const char help[] = {"\
Press and release the stylus button to print out the current device location.\n\
Press and hold the stylus button to exit the application\n"};

    fprintf(stdout, "%s\n", help);
}


// Function designed for chat between client and server.
void func()
{

    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    char *hello = "Hello from server";
       
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
       
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                  &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
       
    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address, 
                                 sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
                       (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    static const int kTerminateCount = 1000;
    int buttonHoldCount = 0;

    /* Instantiate the structure used to capture data from the device. */
    DeviceData currentData;
    DeviceData prevData;

    /* Perform a synchronous call to copy the most current device state. */
    hdScheduleSynchronous(copyDeviceDataCallback, 
        &currentData, HD_MIN_SCHEDULER_PRIORITY);

    memcpy(&prevData, &currentData, sizeof(DeviceData));    

    printHelp();
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 100000000;
    int res;



    char buff2[MAX];
    char buff[MAX];
    int n;
    // infinite loop for chat
    for (;;) {

        hdScheduleSynchronous(copyDeviceDataCallback,
                              &currentData,
                              HD_MIN_SCHEDULER_PRIORITY);



                //printf("Sphere Position  %f %f %f \n",(x-0.5)*200,(y-1.2)*200,-z*200 );
                
        bzero(buff, MAX);
        bzero(buff2, MAX);
  
        
        
       sprintf(buff, "(%g, %g, %g, %d, %g, %g, %g,%g)", 
                currentData.m_devicePosition[0], 
                currentData.m_devicePosition[1], 
                currentData.m_devicePosition[2],
                bool(gServoDeviceData.m_buttonState),myQuat.x, myQuat.y, myQuat.z, myQuat.w);
        
  
        // and send that buffer to client
        send(new_socket , buff , strlen(buff) , 0 );
        
  
        memcpy(&prevData, &currentData, sizeof(DeviceData)); 
        valread = read( new_socket , buff, 1024);
        //printf(buff);
        char newString[10][110];
        int i, j ,ctr;
        j=0; ctr=0;
    for(i=0;i<=(strlen(buff));i++)
    {
        // if space or NULL found, assign NULL into newString[ctr]
        if(buff[i]==' '||buff[i]=='\0')
        {
            newString[ctr][j]='\0';
            ctr++;  //for next word
            j=0;    //for next word, init index to 0
        }
        else
        {
            newString[ctr][j]=buff[i];
            j++;
        }
    }   
        //std::cout << buff << std::endl;
        x = atof(newString[0]);
        y = atof(newString[2]);
        z = atof(newString[1]);
       // printf("Sphere Position  %f %f %f \n",x,y,z);
                                                 /*fprintf(stdout,"Controller Position: (%g, %g, %g)\n", 
                currentData.m_devicePosition[0], 
                currentData.m_devicePosition[1], 
                currentData.m_devicePosition[2]);
                */
        //printf("End Effector Position: %f , %f, %f\n",x,y,z);
            
        
        
        



        //res = nanosleep(&ts, &ts);
    }
}
  
// Driver function
int main()
{
   


    HDSchedulerHandle hUpdateHandle = 0;
    HDErrorInfo error;

    /* Initialize the device, must be done before attempting to call any hd 
       functions. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }

    /* Schedule the main scheduler callback that updates the device state. */
    hUpdateHandle = hdScheduleAsynchronous(
        updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    hdEnable(HD_FORCE_OUTPUT);
    /* Start the servo loop scheduler. */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }
     struct timespec ts;
    ts.tv_sec = 1;
    ts.tv_nsec = 0;
    int res;
    res = nanosleep(&ts, &ts);
    // Function for chatting between client and server
    func();
  
    // After chatting close the socket

}