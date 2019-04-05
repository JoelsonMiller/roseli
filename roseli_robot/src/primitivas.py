#ACENTOS OCASIONAM  ERRO NO PROGRAMA!!
#primitivas referentes ao led, buzzer, botoes, lcd, teclado, potenciometro
import sys, traceback
#import Adafruit_GPIO.SPI as SPI
#import Adafruit_MCP3008 as ADC

from smbus import SMBus #importa biblioteca que contem I2C
import time #importando biblioteca de tempo
bus = SMBus(1) #configura o canal do i2c para o canal 1 e renomeia para 'bus'
lcd05 = 0x63 #endereco do lcd

PCF = [0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27] #lista com todos os PCFs

bus = SMBus(1) #configura o canal do i2c para o canal 1

#funcao para ligar buzzer
def buzzer(estado):
        if (estado == True):
                bus.write_byte(PCF[7],0)
        else:
                bus.write_byte(PCF[7],255)

#Funcao para acionar botao
  #Entradas: i [0 - 7] -> indica o botao habilitado
def button(i):
        #ler = bus.read_byte(PCF[1]) #leitura do respectivo pcf do botao
        mask = 2**i
        if(i >= 0 and i <= 7):
                if (mask & bus.read_byte(PCF[1])) == 0:
                        return False
                else:
                        return True
        else:
                print "Algum argumento do botao esta invalido!!!"
                sys.exit(0)
#Funcao para acender, apagar, ou comutar leds
  #Entradas: write  = 0 e led  [0 - 7] -> acende led
  #Entradas: write  = 1 e led  [0 - 7] -> apaga led
  #Entradas: write  = 2 e led  [0 - 7] -> comuta led
  #Entradas: write [0-2] e led > 8 -> acao afeta todos leds

def led(write, led):
        if(led >= 0 and led <= 8 and write >= 0 and write <= 2):
                if(led >= 0 and led <= 7):
                        mask = 2**led
                if(led == 8):
                        mask = 255
                #Acende led
                if (write == 0):
                        dado = bus.read_byte(PCF[0]) & ~(mask)
			bus.write_byte(PCF[0],dado)
                #Apaga led
                if(write == 1):
                        dado = bus.read_byte(PCF[0]) | mask
                        bus.write_byte(PCF[0],dado)
                #Comuta led
                if(write == 2):
                        dado = bus.read_byte(PCF[0]) ^ mask
                        bus.write_byte(PCF[0],dado)

        else:
                print "Algum argumento do led esta invalido!!!"
                sys.exit(0)

#Funcao para iniciar o lcd
def lcd_init():
        #0 - sem operacao
        #19 - liga backlight
        #12 - limpa tela e coloca cursor no inicio
        #6 - pisca cursor
        buf=[0,19,12,6]
        bus.write_i2c_block_data(lcd05,0,buf)

def lcd_comando(comando):
        if(type(comando) == int):
                bus.write_byte_data(lcd05,0,comando) #manda um comando para o lcd
        else:
                bus.write_i2c_block_data(lcd05,0,comando)


#Funcao para escrever uma string no lcd
def lcd_escrever(palavra):
        if(type(palavra) == str):
                lista = palavra.split('\n')
                for i in range(len(lista)):
                        for j in range(len(lista[i])):
                                palavra1 = lista[i]
                                a = ord(palavra1[j]) #transforma de char para ascII
                                bus.write_byte_data(lcd05,0,a) #escreve no lcd
                        if i<len(lista)-1:
                                bus.write_byte_data(lcd05,0,13)
        else:
                palavra1 = str(palavra) #transforma para string
                tamanho = len(palavra1) #le tamanho da string
                for i in range(tamanho):
                        a = ord(palavra1[i]) #transforma de char para ascII
                        bus.write_byte_data(lcd05,0,a) #escreve no lcd

#funcao para ler teclado
def teclado():
        lowb = bus.read_byte_data(lcd05,1) #le o lowbyte 
        highb = bus.read_byte_data(lcd05,2) #le o high byte
        tec = highb<<8+ lowb
        print(highb,lowb,tec)
        #Mapeamento Teclado
        digito = 0
        if (lowb == 1):
                digito = '1'
                escrever(digito)
        if (lowb == 2):
                digito = '2'
                escrever(digito)
        if (lowb ==4):
                digito = '3'
                escrever(digito)
        if (lowb == 8):
                digito = '4'
                escrever(digito)
        if (lowb == 16):
                digito = '5'
                escrever(digito)
        if (lowb == 32):
                digito = '6'
                escrever(digito)
        if (lowb == 64):
                digito = '7'
                escrever(digito)
        if (lowb == 128):
                digito = '8'
                escrever(digito)
        if (tec == 256):
                digito = '9'
                escrever(digito)
        if (tec == 512):
                comando(32)
        if (tec == 1024):
                digito = 0
                escrever(digito)
        if (tec == 2048):
                comando(8)
        time.sleep(0.5)

def configuraSPI(porta, select):
        if((porta == 0) or (select == 0) or (select == 1)):
                #definicao da porta e select do SPI
                SPI_PORT = porta
                SPI_DEVICE = select
        else:
                #sai do programa se for informado um valor invalido
                sys.exit()
        #Configuracao do canal SPI
        return(ADC.MCP3008(spi = SPI.SpiDev(SPI_PORT, SPI_DEVICE)))

def leADC(MCP,canal):
        if canal == 0:
                #leitura do canal 0 do AD
                value = (MCP.read_adc(0))*100/1023
        elif canal == 1:
                #leitura do canal 1 do AD
                value = (MCP.read_adc(1))*100/1023

        else:
                #sai do programa se for informado um valor invalido
                sys.exit()
        #retorna o valor convertido da leitura do AD pra porcentagem, de 0 a 100
        return(value)
