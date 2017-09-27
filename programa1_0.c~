#include <stdio.h>		// Standard input/output definitions
#include <stdlib.h>
#include <unistd.h>		// UNIX standard function definitions
#include <errno.h>		// Error number definitions
#include <fcntl.h>		// File control definitions 
#include <termios.h>	// POSIX terminal control definitions 
#include <string.h> 	// para memset
#include <time.h>
#include <stdint.h>		//Para mapa de bits

//////Definicion de funciones///////
int iniciar_puerto(char **);
int serialport_flush(int);
int serialport_writebyte(int, uint8_t);
////////////////////////////////////

int main(int argc,char *argv[]){
		int fd = -1;
		int rc, i;
		if( fd!=-1 )   			//Si el puerto queda abierto, lo cierra
                close(fd);
		fd = iniciar_puerto(argv);
		if( fd==-1 ) perror("No se pudo abrir el puerto");
		serialport_flush(fd);  	//Para limpiar buffer
		
		time_t tiempo = time(0);
		struct tm *tlocal = localtime(&tiempo);

		int min=tlocal->tm_min;
		for(i=0;i<=15;i++){
			if(min==60) min=0;
			rc= serialport_writebyte(fd, (uint8_t)min);		//Variable de chequeo
			if(rc==-1) perror("Error de escritura");
			sleep(60);
			if (min<60)	min++;
			
		}
		
        close(fd);
}


int iniciar_puerto(char *argv[]){
	struct termios toptions;
    int tty_fd;
    tty_fd = open(argv[1], O_RDWR | O_NONBLOCK );

	if (tty_fd == -1){
    	perror("No se pudo abrir el puerto");
    	return -1;
	}	

	// Configuramos velocidad a 9600 bauds
	cfsetispeed(&toptions, B9600);
    cfsetospeed(&toptions, B9600);

	// 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // Sin flow control
    toptions.c_cflag &= ~CRTSCTS;
	
	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;

	tcsetattr(tty_fd, TCSANOW, &toptions);
    if( tcsetattr(tty_fd, TCSAFLUSH, &toptions) < 0) {
        perror("No se pudieron configurar los atributos termios");
        return -1;
    }

    return tty_fd;
}


int serialport_flush(int tty_fd)
{
    sleep(2); 		//Requerido para que funcione el flush
    return tcflush(tty_fd, TCIOFLUSH);
}

int serialport_writebyte( int tty_fd, uint8_t b)
{
    int n = write(tty_fd,&b,1);
    if( n!=1)					//Variable de chequeo
        return -1;
    return 0;
}










