#include <MIDI.h>
#include <math.h>
#include "stdint.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "wiring_private.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
MIDI_CREATE_DEFAULT_INSTANCE();
           
// pinos de entrada
        int s_teremim = 36;             //teremim    --> PC_5   
        int botao=25;                   //pino botao  -->PD 2
        int FSR_PIN = 28;               //pino FSR   --> PE_2
        int kmodo = 33;                //chave modo  --> PD6
        int koitava = 32;              //chave oitava -->PD7
//variaveis e constantes
        int velocity;
        int nota_old = 48;
        int nota_new = 48;
        double t1=1;
        double t0=1;
        int x=1;
        int y=0;
        float freq_[5];
        float frequencia_med;
//parametros da curva de distancia em funcao da frequencia (dist)
        int   a=8811;
        float b=-0.9483;
        float dist=0;
//variaveis do botao
        int   flagb=0;                             
//variaveis/constantes do sensor FSR
		int   readfsr;
		float afsr = 2.557*pow(10,-6);
		float bfsr = 0.02405;
		float cfsr = 20.37;
		float fsr;
//variaveis display
        uint8_t Start =0;
        uint8_t End = 5;
        uint32_t LEDnum = 12; //number of LED's
        uint32_t Green = 0x00;
        uint32_t Red = 0x55;
        uint32_t Blue = 0X00;
        uint32_t BG_Green = 0x00;
        uint32_t BG_Red = 0x00;
        uint32_t BG_Blue = 0x00;
//variavel de modos de operacao      
        int modo=1;    //define modos// modo 0 teremim, modo 1 botao
        
        

void setup(){
   
  Serial.begin(31250);                        //definindo serial para comunicacao MIDI 
  pinMode(FSR_PIN, INPUT);                    //setando entrada do sensor
  pinMode(s_teremim, INPUT_PULLUP);           //setando entrada do teremim com Pullup
  pinMode(botao, INPUT);                      //sentando entrada do botao
  pinMode(kmodo, INPUT_PULLUP);               //definindo chave de selecao de modos 
  pinMode(koitava, INPUT_PULLUP);             //definindo chave de seleção de oitavas 
  attachInterrupt(s_teremim,subida,RISING);   //interrupcao de borda de subida no sinal do teremim
  attachInterrupt(botao,botaoapertado,RISING);//interrupcao de borda de subida no botao
  configureTimer1A();                         //configuracao do timer para o display 
  configureDisplay();                         //configuracao da comunicacao do display 
}

//funcao linearizacao  
int numeromidilin(float dist){
  float n;
  if(digitalRead(koitava)==1){
  n = -0.48*dist+79.2;  //eq para 2 oitavas de 48 até72 com as distancias de 65 a 15cm
  }
  if(digitalRead(koitava)==0){
  n=-0.24*dist+63.6;  //eq para 1 oitava de 48 até 60 com as distancias de 65 a 15cm
  }
  n=round(n);
  return n;
}


//funcao mediana
float mediana(float vetor[5]){
  float tmp;
  int i;
  int j;
  int n = 5;
  
       for(i=0;i<n;i++)
     {
           for(j=0;j<n-i;j++)
           {
                 if(vetor[j]>vetor[j+1])
                 {
                       tmp=vetor[j];
                       vetor[j]=vetor[j+1];
                       vetor[j+1]=tmp;
                 }
           }
     }

    return vetor[3];
  }

//interrupcao para calcular freq
void subida(){
             
             t1 = micros()-t0;
             freq_[x] = 1000000/t1;
             t0 = micros();
             x++;
             if(x == 6){

                frequencia_med = mediana(freq_);
                x = 1;
              }
           
}

//interrupcao setando flag do botao a cada borda de subida
void botaoapertado(){
  flagb = 1;
}

//funcao para calcular velocity em função da força aplicada ao FSR
void leitura_FSR(){

  readfsr = analogRead(FSR_PIN);
  fsr = afsr*(pow(readfsr,2)) + bfsr*readfsr + cfsr;
  fsr = round(fsr);
  velocity = fsr;
  
}

//config comunicação do display 
void configureDisplay(){
  SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI0 );
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SSIDisable( SSI0_BASE );
  GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  GPIOPinConfigure(GPIO_PA3_SSI0FSS);
  GPIOPinConfigure(GPIO_PA5_SSI0TX);
  GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2);
  SSIConfigSetExpClk( SSI0_BASE,
                                SysCtlClockGet(),
                                SSI_FRF_MOTO_MODE_1,
                          SSI_MODE_MASTER,
                                2150000,
                                8 );
  SSIEnable( SSI0_BASE );
}

//config interrup timer para display 
void configureTimer1A(){   
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // habilitando Timer 1 Clock
  ROM_IntMasterEnable(); // habilitando interrupcoes
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configurando operacao do timer como periodica

  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A,8000000); //80MHz/8MHz --> 10Hz --> 100ms 

  TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer1IntHandler); //registra a interrupçao para o endereço da funcao Timer1IntHandler
  ROM_IntEnable(INT_TIMER1A);  // Habilitando interrupcao do Timer 1A 
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Timer 1A Interrupcao quando Timeout
  ROM_TimerEnable(TIMER1_BASE, TIMER_A); // inicia Timer 1A
 
}

//funcao de ajuste de taxa de comunicacao do display
uint32_t Byte2Baud(uint8_t input){
  int8_t rgbBitIdx;
  uint32_t BaseBaud = 9586980; 
  for( rgbBitIdx=7; rgbBitIdx>=0; rgbBitIdx-- ) {
    if (input & (0x1<<rgbBitIdx)){
      BaseBaud = BaseBaud | 0x1<<((rgbBitIdx*3)+1);
    }
  }
  return BaseBaud;
}

//funcao display para ascender leds definidos
void LED_RING_Percent(uint32_t LEDnum, uint8_t Start, uint8_t End, uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t BG_Red, uint32_t BG_Green, uint32_t BG_Blue){
  int8_t LEDIdx;
  uint8_t LEDtxIdx;
  int8_t buffer[144]; // = (LEDnum*9)
  uint32_t RedBaud = Byte2Baud(Red);
  uint32_t GreenBaud = Byte2Baud(Green);
  uint32_t BlueBaud = Byte2Baud(Blue);
  uint32_t BG_RedBaud = Byte2Baud(BG_Red);
  uint32_t BG_GreenBaud = Byte2Baud(BG_Green);
  uint32_t BG_BlueBaud = Byte2Baud(BG_Blue);
  uint8_t StartLED = Start;
  uint8_t EndLED = End;
  
  for( LEDIdx=0; LEDIdx < (StartLED-1); LEDIdx++ ) {
    buffer[(LEDIdx*9)]     = (BG_GreenBaud>>16);
    buffer[((LEDIdx*9)+1)] = (BG_GreenBaud>>8);
    buffer[((LEDIdx*9)+2)] = (BG_GreenBaud);
    buffer[((LEDIdx*9)+3)] = (BG_RedBaud>>16);
    buffer[((LEDIdx*9)+4)] = (BG_RedBaud>>8);
    buffer[((LEDIdx*9)+5)] = (BG_RedBaud);
    buffer[((LEDIdx*9)+6)] = (BG_BlueBaud>>16);
    buffer[((LEDIdx*9)+7)] = (BG_BlueBaud>>8);
    buffer[((LEDIdx*9)+8)] = (BG_BlueBaud);
  }
  
  for( LEDIdx=StartLED; LEDIdx < (EndLED+1); LEDIdx++ ) {
    buffer[(LEDIdx*9)]     = (GreenBaud>>16);
    buffer[((LEDIdx*9)+1)] = (GreenBaud>>8);
    buffer[((LEDIdx*9)+2)] = (GreenBaud);
    buffer[((LEDIdx*9)+3)] = (RedBaud>>16);
    buffer[((LEDIdx*9)+4)] = (RedBaud>>8);
    buffer[((LEDIdx*9)+5)] = (RedBaud);
    buffer[((LEDIdx*9)+6)] = (BlueBaud>>16);
    buffer[((LEDIdx*9)+7)] = (BlueBaud>>8);
    buffer[((LEDIdx*9)+8)] = (BlueBaud);
  }
  
  for( LEDIdx=(EndLED+1); LEDIdx < (LEDnum+1); LEDIdx++ ) {
    buffer[(LEDIdx*9)]     = (BG_GreenBaud>>16);
    buffer[((LEDIdx*9)+1)] = (BG_GreenBaud>>8);
    buffer[((LEDIdx*9)+2)] = (BG_GreenBaud);
    buffer[((LEDIdx*9)+3)] = (BG_RedBaud>>16);
    buffer[((LEDIdx*9)+4)] = (BG_RedBaud>>8);
    buffer[((LEDIdx*9)+5)] = (BG_RedBaud);
    buffer[((LEDIdx*9)+6)] = (BG_BlueBaud>>16);
    buffer[((LEDIdx*9)+7)] = (BG_BlueBaud>>8);
    buffer[((LEDIdx*9)+8)] = (BG_BlueBaud);
  }

  while(SSIBusy( SSI0_BASE ));

  for(LEDtxIdx = 0; LEDtxIdx < (LEDnum*9); LEDtxIdx++) {
    uint8_t data = buffer[LEDtxIdx];
    SSIDataPut( SSI0_BASE, data);
  }
}

//funcao para enviar cores no display
void enviacor(int n){
    int r, g, b, Start, metade;
    Start = 0;

    LED_RING_Percent(LEDnum, Start, 11, 0, 0, 0, BG_Red, BG_Green, BG_Blue); 
    delay(1);
    switch(n%12){
      case 0:   // do verm
        r = 0x30;
        g = 0x00;
        b = 0x00;
        metade = 11;
        break;

      case 1:  // do sust verm pela met
        r = 0x30;
        g = 0x00;
        b = 0x00;
        metade = 5;
        break;
       
      case 2:    // re amarelo
        r = 0x30;
        g = 0x30;
        b = 0x00;
        metade = 11;
        break;

      case 3:    // re sust amarelo metade
        r = 0x15;
        g = 0x15;
        b = 0x00;
        metade = 5;
        break;

          case 4:  // mi branco
        r = 0x15;
        g = 0x15;
        b = 0x15;
        metade = 11;
        break;
       
      case 5:    // fa verde
        r = 0x00;
        g = 0x30;
        b = 0x00;
        metade = 11;
        break;

      case 6:    // fa verde metade
        r = 0x00;
        g = 0x30;
        b = 0x00;
        metade = 5;
        break;

        case 7:    // sol azul
        r = 0x00;
        g = 0x00;
        b = 0x30;
        metade = 11;
        break;

      case 8:    // sol azul metade
        r = 0x00;
        g = 0x00;
        b = 0x30;
        metade = 5;
        break;

        case 9:    // la magenta
        r = 0x30;
        g = 0x00;
        b = 0x30;
        metade = 11;
        break;

      case 10:    // la sust magenta metade
        r = 0x30;
        g = 0x00;
        b = 0x30;
        metade = 5;
        break;

        case 11:    // si ciano
        r = 0x00;
        g = 0x30;
        b = 0x30;
        metade = 11;
        break;
      
    }

  if(n<48)LED_RING_Percent(LEDnum, Start, 11, 0, 0, 0, BG_Red, BG_Green, BG_Blue); 
 
    else LED_RING_Percent(LEDnum, Start, metade, r, g, b, BG_Red, BG_Green, BG_Blue); 
    
}

//interrupcao timer do visor a cada 100ms 
void Timer1IntHandler(void){
   
  ROM_TimerIntClear(TIMER1_BASE, TIMER_A);
  dist = a*(pow(frequencia_med,b));
  nota_new = numeromidilin(dist);
  enviacor(nota_new); 
  
}



void loop(){
dist = a*(pow(frequencia_med,b));     // realiza o cálculo da distancia baseado na frequencia lida do teremim
nota_new = numeromidilin(dist);       // retorna o numero midi em funcao da frequencia lida do teremim linearizada através da distancia 
                  
if(digitalRead(kmodo)==1){                                                                    // verifica se está no modo botao
             if(flagb == 1){                                                                  // verifica se o botao foi pressionado 
                              if(nota_new>=48){
                        leitura_FSR();                   									  // leitura da pressao aplicada no FSR 
                                                if (velocity <= 65){ velocity = 65;}          // ignorando valores menores que 75
                                                if (velocity >= 127){ velocity = 127;}        // ignorando valores maiores que 127
                                                MIDI.sendNoteOn(nota_new,velocity,1);         // enviando nota MIDI 
                                                int nota_aux = nota_new;                      // variavel auxiliar para controle de duracao da nota 
                            while(flagb == 1)           									  // loop para verificar se o botao ainda está pressionado 
                            {
                                delayMicroseconds(10);         
                              if(digitalRead(botao)==LOW)            
                              {
                              MIDI.sendNoteOff(nota_aux,0,1); 								  // desligando a nota apos o botao ser solto
                              flagb=0;                         							      // zerando flag do botao 
                              }
                                                        }
                                              }  
                           }
                                    delay(20);                        						 // delay para estabilidade e debouncing
                        }
      
if(digitalRead(kmodo)==0){                                          						 // verifica se está no modo teremim
             if(nota_new>=48){
                if (nota_new != nota_old){                         						     // verifica se a nota foi alterada em funcao da frequencia
                              MIDI.sendNoteOff(nota_old,5,1);   							 // para de enviar a nota antiga
                              MIDI.sendNoteOn(nota_new,60,1);   							 // envia a nota nova
                              nota_old = nota_new;              							 // atualiza a ultima nota tocada
                                                        }
                            delay(20);                    									 // delay em funcao da frequencia 
                        leitura_FSR();                                						 // leitura da pressao aplicada no FSR 
                        if (velocity <= 25){ velocity = 25;}          						 // ignorando valores menores que 25
                        if (velocity >= 127){ velocity = 127;}        						 // ignorando valores maiores que 127
                        MIDI.sendAfterTouch(velocity,1);              						 // enviando after touch
                             } 
                        }
}

