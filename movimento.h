#ifndef MOVIMENTO_INCLUDED
#define MOVIMENTO_INCLUDED

void resetencoders(void);
float readencoders(int numero);
float readencodersang(int numero);
void drivemotors(int speed1, int speed2);
void stopmotors(void);
void rotate(int speed);

#endif
