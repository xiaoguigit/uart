#include <stdio.h>   
#include <string.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include "uart.h" 


    
//*************************Test*********************************   
int main(int argc, char *argv[])  
{  
    int fdcom, i, SendLen, RecvLen;  
    struct termios termios_cur;  
    char RecvBuf[1024];  
    portinfo_t portinfo ={  
        '0',                            // print prompt after receiving   
        115200,                         // baudrate: 115200   
        '8',                            // databit: 8   
        '0',                            // debug: off   
        '0',                            // echo: off   
        '2',                            // flow control: software   
        '1',                            // default tty: COM1   
        '0',                            // parity: none   
        '1',                            // stopbit: 1   
         0                          // reserved   
    };  

    if(argc != 2){  
        printf("Usage:\n0 for send \n1 for receive\n");  
        printf("   eg:");  
        printf("        %s 0\n", argv[0]);  
        exit(-1);  
    }  

    fdcom = PortOpen(&portinfo);  
    if(fdcom<0){  
        printf("Error: open serial port error.\n");  
        exit(1);  
    }  
    
    PortSet(fdcom, &portinfo);  
    
    if(atoi(argv[1]) == 0){  
        //send data   
        for(i=0; i<100; i++){  
            SendLen = PortSend(fdcom, "1234567890", 10);  
            if(SendLen>0){  
                printf("No %d send %d data 1234567890.\n", i, SendLen);  
            }  
            else{  
                printf("Error: send failed.\n");  
            }  
            //sleep(1);  
        }  
        PortClose(fdcom);  
    }  
    else{  
        for(;;){  
            RecvLen = PortRecv(fdcom, RecvBuf, 1024, portinfo.baudrate);  
            if(RecvLen>0){  
                printf("%s\n", RecvBuf);  
            }  
            else{  
                printf("Error: receive error.\n");  
            }  
        }  
    }  
    return 0;  
} 
