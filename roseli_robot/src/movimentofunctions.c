#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#define ADDRESS1 0x58 //definição do endereço de i2c da placa MD25

//declaração das variaveis que serão utilizadas na comunicação via i2c

int fd;		// file descriptor
char *fileName = "/dev/i2c-1"; 	//protocolo i2c, porta 1 
unsigned char buf[10]; //buffer de dados

//-----------------------------------------------------------------------------------------------------------
//------------------------------FUNÇÕES DE MOVIMENTO---------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------


//Função criada para resetar a contagem dos encoders

void resetencoders(void) {
	// escreve o comando 32 no registrador 16 da placa MD25
	buf[0] = 16;
	buf[1] = 32;
	// abre a porta i2c
	if((fd = open(fileName, O_RDWR))<0){
		printf("resetEncoders: Failed to open i2c port\n");
		exit(1);
	}
	//define que  a placa MD25 é um escravo
	ioctl(fd, I2C_SLAVE, ADDRESS1);
	//comunica com escravo
	if ((write(fd, buf, 2)) != 2){
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}

//Função criada para ler os encoders, linearmente

float readencoders(int numero){
	float encoder;
	buf[0] = 2;

	//abre a porta i2c
	if((fd = open(fileName, O_RDWR)) < 0){
		printf("readencoders: Failed to open i2c port\n");
		exit(1);
	}
	//define que a placa MD25 é um escravo
	ioctl(fd, I2C_SLAVE, ADDRESS1);
	//verifica qual dos dois encoders deve ler
	//os registradores do primeiro encoder são de 2 a 5 e os do segundo de 5 a 8
	if( numero  == 1){
		if((write(fd,buf,1)) != 1){
			printf("Error writing to i2c slave\n");
			exit(1);
		}
		if (read(fd,buf,8)!= 8){
			printf("Unable to read from slave\n");
			exit(1);
		}
		else {
			encoder = ((buf[0]<<24) + (buf[1]<<16) + (buf[2]<<8) + buf[3]);
		}
	}
	if (numero  == 2){
		if((write(fd,buf,1)) !=1){
			printf("Error writing to i2c slave\n");
			exit(1);
		}
		if (read(fd,buf,8) != 8){
			printf("Unable to read from slave\n");
			exit(1);
		}
		else {
			encoder = ((buf[4]<<24) + (buf[5]<<16) +  (buf[6]<<8) + buf[7]);
		}
	}
	return ((encoder*10*3.14159)/360); //retorna o valor em centimetros

}

// Função criada para ler os encoders angularmente

float readencodersang(int numero){
	float encoder;
	//Usa a função readencoders() e retorna seu valor para hexa
	encoder = (360*readencoders(numero))/(10*3.14159);
	return((encoder*50/140)); //retorna o valor em graus
}

//Função criada para movimentar o robo linearmente

void drivemotors(int speed1, int speed2){

	//abre a porta i2c
	if((fd = open(fileName, O_RDWR))<0){
		printf("drivemotor: Failed to open i2c port\n");
		exit(1);
	}
	//define a placa MD25 como slave
	ioctl(fd, I2C_SLAVE, ADDRESS1);

		buf[0]=16;
                buf[1]=51;
        if((write(fd,buf,2))!=2){
                        printf("drivemotor: Error writing to i2c slave\n");
                        exit(1);
                }

	//entra em dois casos, quando as velocidades são iguais utiliza o modo 2 e quando são diferentes utiliza o modo 1

	/*if(speed1 == speed2){
		//escreve 2 no registrador 15
		buf[0] = 15;
		buf[1] = 2;
		//escreve dois valores no escravo
		if((write(fd, buf, 2)!=2)){
			printf("drivemotor: Error writing to i2c slave\n");
			exit(1);
		}
		//escreve a velocidade desejada no registrador 0
		buf[0] = 0;
		buf[1] = speed1+128;
		//escreve dois valores no escravo
		if((write(fd, buf, 2)) !=2){
			printf("drivemotor: Error writing to i2c slave\n");
			exit(1);
		}
	}
	else{*/
		//escreve 1 no registrador 15
		buf[0] = 15;
		buf[1] = 1;
		//escreve dois valores no escravo
		if((write(fd,buf,2))!=2){
			printf("drivemotor: Error writing to i2c slave\n");
			exit(1);
		}
		//escreve speed1 no registrador 0
		buf[0] = 0;
		buf[1] = speed1;
		//escreve dois valores no escravo
		if((write(fd,buf,2))!=2){
			printf("drivemotor: Error writign to i2c slave\n");
			exit(1);
		}
		//escreve speed2 no registrador 1
		buf[0] = 1;
		buf[1] = speed2;
		//escreve dois valores no escravo
		if((write(fd,buf,2))!=2){
			printf("drivemotor: Error writing to i2c slave\n");
			exit(1);
		}
	//}
}

void stopmotors(void){
	//abre a porta i2c
	if((fd = open(fileName,O_RDWR))<0){
		printf("stopmotors: Failed to open i2c port\n");
		exit(1);
	}
	//define a placa MD25 como escravo
	ioctl(fd,I2C_SLAVE,ADDRESS1);
	//escreve o valor 0 no registrador 15
	buf[0] = 15;
	buf[1] = 0;
	//escree dois valores no escravo
	if((write(fd,buf,2))!=2){
		printf("stopmotors: Failed to write to i2c slave\n");
		exit(1);
	}
	//escreve o valor 128 no registrador 0
	buf[0] = 0;
	buf[1] = 128;
	//escreve dois valores no escravo
	if((write(fd,buf,2))!=2){
		printf("stopmotors: Error writing to i2c slave\n");
		exit(1);
	}
	//escreve o valor 128 no registrador 1
	buf[0] = 1;
	buf[1] = 128;
	//escreve dois valores no escravo
	if((write(fd,buf,2))!=2){
		printf("stopmotors: Error writing to i2c slave\n");
		exit(1);
	}
}


//Função criada para rotacionar o robô em torno do proprio eixo
void rotate(int speed){
	//abre a porta i2c
	//if ((fd = open(fileName, O_RDWR))<0){
	//	printf("rotate: Failed to open i2c port\n");
		//exit(1);
	//}
	//define a placa MD25 como escravo
	ioctl(fd,I2C_SLAVE,ADDRESS1);

	//escreve o valor 1 no registrador 15

                buf[0]=16;
                buf[1]=51;

        if((write(fd,buf,2))!=2){
                        printf("drivemotor: Error writing to i2c slave\n");
                        exit(1);
                }

	buf[0] = 15;
	buf[1] = 1;
	//escreve dois valores no escravo
	if((write(fd,buf,2))!=2){
		printf("rotate: Error writing to i2c slave\n");
		exit(1);
	}
	//escreve speed no registrador 0
	buf[0] = 0;
	buf[1] = speed;
	//escreve dois valores no escravo
	if((write(fd,buf,2))!=2){
		printf("rotate: Error writing to i2c slave\n");
		exit(1);

	}
	//escreve -speed no registrador 1
	buf[0] = 1;
	buf[1] = -speed;
	//escreve dois valores no escravo
	if((write(fd,buf,2))!= 2){
		printf("rotate: Error writing to i2c slave\n");
		exit(1);
	}

}
