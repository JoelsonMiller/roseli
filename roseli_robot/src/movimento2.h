#ifndef MOVIMENTO2_INCLUDED
#define MOVIMENTO2_INCLUDED

int open_i2c_bus(void);
void resetencoders(int fd);
float readencoders(int numero, int fd);
float readencodersang(int numero, int fd);
void drivemotors(int speed1, int speed2, int fd);
void stopmotors(int fd);
void rotate(int speed, int fd);

#endif
