#include <Dynamixel_Servo.h>
#include <Thread.h>
#include <ThreadController.h>

//GERAL

//#define MODO_SIMULACAO
#define MODO_DATALOGGER

#ifdef MODO_DATALOGGER
#include <SPI.h>
#include <SD.h>
#define CS_SPI                    4

#define PINO_LED_1                53  // pinos usado no LED bicolor
#define PINO_LED_2                49  // (LED de status)
#define PINO_LED_GND              51  // GND

String  dataString = "";
String  nome_arquivo = "";

bool flag_grava_dados = false;

File dataFile;

unsigned char dado_eeprom = 0;

#endif

#define LED_DEBUG                 13

//VARIAVEIS E CONSTANTES DE CONTROLE DAS THREADS

//ThreadController
ThreadController controle = ThreadController();

//Threads
Thread Thread1 = Thread();
Thread Thread2 = Thread();
Thread Thread3 = Thread();
bool  flag_debug = false;

//VARIAVEIS E CONSTANTES DE CONTROLE DOS SERVOS

#define SERVO_BAUD_RATE           400000
#define HALF_DUPLEX_DIRECTION_PIN 12
#define TIMEOUT                   1  //em ms
#define VELOCIDADE_SERVO_POR      5   //<-- MUDA A VELOCIDADE ANGULAR DOS SERVOS (ORIGINAL 10)
#define NUM_PASSO                 (sizeof(pos_cn)/sizeof(float))

#define NUM_SERVOS                12

#define SERVO_M1                  0
#define SERVO_M2                  1
#define SERVO_M3                  2
#define SERVO_M4                  3
#define SERVO_M5                  4
#define SERVO_M6                  5
#define SERVO_M7                  6
#define SERVO_M8                  7
#define SERVO_M9                  8
#define SERVO_M10                 9
#define SERVO_M11                 10
#define SERVO_M12                 11

#define F_TS                      -140
#define F_TI                      -178
#define F_FS                      -150
#define F_FI                      -160

float pos_zero[12] = {  3.34,   //<--M1
                        2.32,   //<--M2
                        3.92,   //<--M3
                        3.14,   //<--M4
                        3.72,   //<--M5
                        3.71,   //<--M6
                        3.11,   //<--M7
                        3.15,   //<--M8
                        3.37,   //<--M9
                        3.07,   //<--M10 
                        2.93,   //<--M11
                        3.14 }; //<--M12

//superior
float pos_un[] = { 2.18, 2.22, 2.26, 2.30, 2.34, 2.38, 2.42, 2.46, 2.50, 2.55, 2.59, 2.63, 2.67, 2.71, 2.46, 2.20, 1.95, 1.76, 1.69, 1.80 }; 

//inferior
float pos_cn[] = { 2.86, 2.87, 2.89, 2.90, 2.91, 2.92, 2.94, 2.95, 2.96, 2.97, 2.99, 3.00, 3.01, 3.02, 2.58, 2.21, 2.03, 2.06, 2.28, 2.60 };                     

                    
//DEBUG

unsigned char qdiant_direito = 0;
unsigned char qdiant_esquerdo = 0;
unsigned char qtras_direito = 0;
unsigned char qtras_esquerdo = 0;

unsigned char rqdiant_direito = 90;
unsigned char rqdiant_esquerdo = 90;
unsigned char rqtras_direito = 90;
unsigned char rqtras_esquerdo = 90;

//





unsigned char ids[NUM_SERVOS] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }; //<- LISTAGEM DOS IDS GRAVADOS EM CADA SERVO

float angulos[NUM_SERVOS];
float dados_servo[NUM_SERVOS];

servo_error_t error;

enum {
  SM_RESET = 0,
  SM_ZERO,
  SM_PARA,
  SM_ANDA,
  SM_TRANS,
  SM_REV
} sm_robo = SM_RESET;

struct {
  unsigned char ids[12];
  unsigned char indice;
  bool  flag_verifica_servo;
} lista_servos;

#define servo_a_ser_verificado(x)   if(!lista_servos.flag_verifica_servo) { lista_servos.ids[lista_servos.indice++] = x+1, lista_servos.ids[lista_servos.indice] = 0; }
#define verifica_servo()            lista_servos.flag_verifica_servo = true
#define reset_lista_servos()        lista_servos.indice = 0, lista_servos.ids[0] = 0, lista_servos.flag_verifica_servo = false
#define retorna_ID(x)               lista_servos.ids[x]
#define retorna_numero_de_IDs()     lista_servos.indice


//VARIAVEIS E CONSTANTES DE CONTROLE DE USO GERAL
bool  flag_dir = false;
float valor;
String buf = "";

/*----------------------------------------------------*/
//                    FUNCOES
/*----------------------------------------------------*/

float velocidade_servo(float porcentagem, float maximo)
{
  return((porcentagem*maximo)/100);
}

void le_dados_servos(unsigned char ids[], unsigned char comando, unsigned char num_servos, float dados[])
{
  unsigned char i;
  float valor;
  for(i=0; i<num_servos;i++)
  {
    servo_get(ids[i], (servo_register_t)comando, &valor, TIMEOUT);
    dados[i] = valor;    
  }
}

void escreve_dados_servos_constante(unsigned char ids[], unsigned char comando, unsigned char num_servos, float valor)
{
  unsigned char i;
  for(i=0; i<num_servos;i++)
  {
    servo_set(ids[i], (servo_register_t)comando, valor, TIMEOUT);
  }
}

void aguarda_servos(void)
{ 
  unsigned char i = 0;
  bool flag_init_servos = false;
  
  while(!flag_init_servos)
  {
    le_dados_servos(ids, SERVO_REGISTER_ID, NUM_SERVOS, dados_servo);

    flag_init_servos = true;
    for(i=0;i<NUM_SERVOS;i++)
    {
      if(ids[i] != (unsigned char)dados_servo[i])
      {
       flag_init_servos = false;
      } 
    }  
  }
}

bool verifica_servos(void)
{ 
  static unsigned char i = 0;
  static bool flag_servo_status = true;
  static bool flag_servo_falha = false;
  float dado;

  if(!flag_servo_falha)
  {
    servo_get(ids[i], SERVO_REGISTER_ID, &dado, TIMEOUT);  
  }

  if(ids[i] != (unsigned char)dado)
  {
    flag_servo_falha = true;
  }
  
  i++;
  if(i >= NUM_SERVOS)
  {
    i = 0;
    
    if(flag_servo_falha)
    {
      flag_servo_status = false;
    }
    else
    {
      flag_servo_status = true;
    }
    
    flag_servo_falha = false;
  }

  return flag_servo_status; 
}

bool verifica_servos_em_movimento(void)
{
  unsigned char i = 0;
  bool flag_le_servos = false;
  float dado;

  flag_le_servos = false;
  
  for(i=0;i<retorna_numero_de_IDs();i++)
  {
    servo_get(retorna_ID(i), SERVO_REGISTER_IS_MOVING, &dado, TIMEOUT);
  
    if((unsigned char)dado == 1)
    {
      flag_le_servos = true;
      break;
    }
  } 
  return flag_le_servos;     
}

//bool flag_teste = 1;


void SM_robo(void)
{
  static bool flag_contador_arquivo = true;       
  switch(sm_robo)
  {
    case SM_ZERO:
      #ifndef MODO_SIMULACAO

    //Posição Pata/Casco para Teste
      //if(flag_teste) {

         //flag_teste =0;
         //pos_zero[ 0 ]  = pos_zero[ 0 ] + converte_graus_para_radianos(90);
         //pos_zero[ 3 ]  = pos_zero[ 3 ] + converte_graus_para_radianos(-90);
         //pos_zero[ 6 ]  = pos_zero[ 6 ] + converte_graus_para_radianos(-90);
         //pos_zero[ 9 ]  = pos_zero[ 9 ] + converte_graus_para_radianos(90);
      //}
      
      servo_set_multiple(ids, SERVO_REGISTER_GOAL_ANGLE, pos_zero, NUM_SERVOS, 1);
      //DEBUG
      qdiant_direito = 0;
      qdiant_esquerdo = 0;
      qtras_direito = 0;
      qtras_esquerdo = 0;
      //
      #else
      Serial.println("ZERO OK!");
      #endif

      #ifdef MODO_DATALOGGER
      flag_grava_dados = 1;
      #endif
      
      reset_lista_servos();
      break;
      
    case SM_PARA:
      #ifdef MODO_SIMULACAO
      Serial.println("ZERO OK!");
      #endif
      
      #ifdef MODO_DATALOGGER
      flag_grava_dados = 1;
      #endif
      
      break;
      
    case SM_ANDA:
      #ifdef MODO_SIMULACAO
      Serial.println("ANDA OK!");
      #endif

      #ifdef MODO_DATALOGGER
      if(flag_grava_dados)
      {
        flag_grava_dados = 0;
        nome_arquivo = "";
        nome_arquivo = "dados_";
        nome_arquivo += (String)dado_eeprom;
        nome_arquivo += ".csv";

        Serial.print("Arquivo: ");
        Serial.println(nome_arquivo);

        dado_eeprom++;
        grava_arquivo_eeprom();      
      }
      #endif
           
      anda_robo();
      break;


    case SM_TRANS:
      #ifdef MODO_SIMULACAO
      Serial.println("TRANSIÇÃO OK!");
      #endif

      #ifdef MODO_DATALOGGER
      //if(flag_grava_dados)
      //{
        //flag_grava_dados = 0;
        //nome_arquivo = "";
        //nome_arquivo = "dados_";
        //nome_arquivo += (String)dado_eeprom;
        //nome_arquivo += ".csv";

        //Serial.print("Arquivo: ");
        //Serial.println(nome_arquivo);

        //dado_eeprom++;
        //grava_arquivo_eeprom();      
      //}
      #endif
           
      transi_robo();
      break;


  case SM_REV:
      #ifdef MODO_SIMULACAO
      Serial.println("REVERSO OK!");
      #endif

      #ifdef MODO_DATALOGGER
      //if(flag_grava_dados)
      //{
        //flag_grava_dados = 0;
        //nome_arquivo = "";
        //nome_arquivo = "dados_";
        //nome_arquivo += (String)dado_eeprom;
        //nome_arquivo += ".csv";

        //Serial.print("Arquivo: ");
        //Serial.println(nome_arquivo);

        //dado_eeprom++;
        //grava_arquivo_eeprom();      
      //}
      #endif
           
      rev_trans_robo();
      break;
      
    case SM_RESET:
      #ifndef MODO_SIMULACAO
      escreve_dados_servos_constante(ids, SERVO_REGISTER_MOVING_SPEED, NUM_SERVOS, velocidade_servo(VELOCIDADE_SERVO_POR, SERVO_MAXIMUM_MOVING_SPEED));
      escreve_dados_servos_constante(ids, SERVO_REGISTER_MAX_TORQUE, NUM_SERVOS, 1022);
      escreve_dados_servos_constante(ids, SERVO_REGISTER_TORQUE_LIMIT, NUM_SERVOS, 1022);

      escreve_dados_servos_constante(ids, SERVO_REGISTER_TORQUE_ENABLE, NUM_SERVOS, 1);
      escreve_dados_servos_constante(ids, SERVO_REGISTER_RETURN_DELAY_TIME, NUM_SERVOS, 1);    
      #else
      Serial.println("RESET OK!");
      #endif
      sm_robo = SM_ZERO;
      break;
         
    default:
      sm_robo = SM_ZERO; 
      break;
  }
}

float converte_graus_para_radianos(float graus) 
{
  return (graus * (6.28318531 / 360.0));
}

void anda_robo(void)
{
  static unsigned char passo = 0;
  static unsigned char anterior_direito = 0;
  static unsigned char anterior_esquerdo = 0;
  static unsigned char posterior_direito = 0;
  static unsigned char posterior_esquerdo = 0;

  static float adj_TS;
  static float adj_TI;
  static float adj_FS;
  static float adj_FI;

  static float  porcentagem_mov = 1.0;

  unsigned char i;

  static bool flag_inicializacao = 0;
 
  if(!flag_inicializacao)
  {   
    for(i=0;i<NUM_SERVOS;i++)
    {
      angulos[i] = pos_zero[i];
    }

    adj_TS = converte_graus_para_radianos(F_TS);
    adj_TI = converte_graus_para_radianos(F_TI);
    adj_FS = converte_graus_para_radianos(F_FS);
    adj_FI = converte_graus_para_radianos(F_FI);
    
    flag_inicializacao = 1;
  }
 
  anterior_direito = passo;

  
  posterior_direito = passo + 15;
  if(posterior_direito >= NUM_PASSO)
  {
    posterior_direito -= NUM_PASSO;
    
  }
  

  anterior_esquerdo = passo + 10;
  if(anterior_esquerdo >= NUM_PASSO)
  {
    anterior_esquerdo -= NUM_PASSO;
  }

  posterior_esquerdo = passo + 5;
  if(posterior_esquerdo >= NUM_PASSO)
  {
    posterior_esquerdo -= NUM_PASSO;
  }

  angulos[SERVO_M2] = -((pos_cn[posterior_direito] + adj_FI)*porcentagem_mov) + pos_zero[SERVO_M2];
  angulos[SERVO_M3] = -((pos_un[posterior_direito] + adj_FS)*porcentagem_mov) + pos_zero[SERVO_M3];

  angulos[SERVO_M5] = ((pos_cn[posterior_esquerdo] + adj_FI)*porcentagem_mov) + pos_zero[SERVO_M5];
  angulos[SERVO_M6] = ((pos_un[posterior_esquerdo] + adj_FS)*porcentagem_mov) + pos_zero[SERVO_M6];

  angulos[SERVO_M8] = ((pos_cn[anterior_direito] + adj_TI)*porcentagem_mov) + pos_zero[SERVO_M8];
  angulos[SERVO_M9] = -((pos_un[anterior_direito] + adj_TS)*porcentagem_mov) + pos_zero[SERVO_M9];

  angulos[SERVO_M11] = -((pos_cn[anterior_esquerdo] + adj_TI)*porcentagem_mov) + pos_zero[SERVO_M11];
  angulos[SERVO_M12] = ((pos_un[anterior_esquerdo] + adj_TS)*porcentagem_mov) + pos_zero[SERVO_M12];

  angulos[SERVO_M1] = pos_zero[SERVO_M1];
  angulos[SERVO_M4] = pos_zero[SERVO_M4];

  angulos[SERVO_M7] =  pos_zero[SERVO_M7];
  angulos[SERVO_M10] = pos_zero[SERVO_M10];

  
  servo_a_ser_verificado(SERVO_M2);
  servo_a_ser_verificado(SERVO_M3);
  servo_a_ser_verificado(SERVO_M5);
  servo_a_ser_verificado(SERVO_M6);
  servo_a_ser_verificado(SERVO_M8);
  servo_a_ser_verificado(SERVO_M9);
  servo_a_ser_verificado(SERVO_M11);
  servo_a_ser_verificado(SERVO_M12);
  verifica_servo();

  #ifdef MODO_SIMULACAO
  Serial.println("PASSADA = " + (String)passo);
  Serial.println("Servo 2 = " + (String)angulos[SERVO_M2]);
  Serial.println("Servo 3 = " + (String)angulos[SERVO_M3]);
  Serial.println("Servo 5 = " + (String)angulos[SERVO_M5]);
  Serial.println("Servo 6 = " + (String)angulos[SERVO_M6]);
  Serial.println("Servo 8 = " + (String)angulos[SERVO_M8]);
  Serial.println("Servo 9 = " + (String)angulos[SERVO_M9]);
  Serial.println("Servo 11 = " + (String)angulos[SERVO_M11]);
  Serial.println("Servo 12 = " + (String)angulos[SERVO_M12]);
  #endif

  #ifndef MODO_SIMULACAO
  //if(!verifica_servos_em_movimento())
  #endif
  {
    //debug
    #ifdef MODO_DATALOGGER

    File localdataFile = SD.open(nome_arquivo, FILE_WRITE);
    
    // se o arquivo estiver presente, escreve:    
    unsigned char i;
    if(localdataFile)
    {
      dataString = "";
            
      le_dados_servos(ids, SERVO_REGISTER_PRESENT_ANGLE, NUM_SERVOS, dados_servo);
      
      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      le_dados_servos(ids, SERVO_REGISTER_PRESENT_TORQUE, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }

      le_dados_servos(ids, SERVO_REGISTER_PRESENT_VOLTAGE, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      le_dados_servos(ids, SERVO_REGISTER_CURRENT_CONSUMPTION, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      dataString.replace('.',',');
      
      localdataFile.println(dataString);
      localdataFile.close();
    }
    #endif

    #ifndef MODO_SIMULACAO
    servo_set_multiple(ids, SERVO_REGISTER_GOAL_ANGLE, angulos, NUM_SERVOS, 1);
    #endif
    passo++;
    Serial.println("PASSADA = " + (String)passo);
  
    if(passo >= NUM_PASSO)
    {
      reset_lista_servos();
      passo = 0;
    }
  }
}

void transi_robo(void)
{
  static unsigned char passo = 0;
  static unsigned char anterior_direito = 0;
  static unsigned char anterior_esquerdo = 0;
  static unsigned char posterior_direito = 0;
  static unsigned char posterior_esquerdo = 0;

  static float adj_TS;
  static float adj_TI;
  static float adj_FS;
  static float adj_FI;

  static float  porcentagem_mov = 1.0;

  unsigned char i;

  static bool flag_inicializacao = 0;
 
  if(!flag_inicializacao)
  {   
    for(i=0;i<NUM_SERVOS;i++)
    {
      angulos[i] = pos_zero[i];
    }

    adj_TS = converte_graus_para_radianos(F_TS);
    adj_TI = converte_graus_para_radianos(F_TI);
    adj_FS = converte_graus_para_radianos(F_FS);
    adj_FI = converte_graus_para_radianos(F_FI);
    
    flag_inicializacao = 1;
  }
 
  anterior_direito = passo;

  if(passo == 5){

    if(qdiant_direito<90){
    
    qdiant_direito+= 15;
    }

    Serial.println("Servo 1 = " + (String)angulos[SERVO_M1]);
  }
  
  posterior_direito = passo + 15;
  if(posterior_direito >= NUM_PASSO)
  {
    posterior_direito -= NUM_PASSO;
  }

  if(passo == 19){

    if(qtras_direito<90){
    
    qtras_direito += 15;
    }
    Serial.println("Servo 4 = " + (String)angulos[SERVO_M4]);
  }


  anterior_esquerdo = passo + 10;
  if(anterior_esquerdo >= NUM_PASSO)
  {
    anterior_esquerdo -= NUM_PASSO;
  }
  
  if(passo == 15){

    if(qdiant_esquerdo<90){
    
    qdiant_esquerdo += 15;
    }
    Serial.println("Servo 7 = " + (String)angulos[SERVO_M7]);
  }

  posterior_esquerdo = passo + 5;
  if(posterior_esquerdo >= NUM_PASSO)
  {
    posterior_esquerdo -= NUM_PASSO;
  }

  if(passo == 9){

    if(qtras_esquerdo<90){
    
    qtras_esquerdo += 15;
    }

    Serial.println("Servo 10 = " + (String)angulos[SERVO_M10]);
  }

  angulos[SERVO_M2] = -((pos_cn[posterior_direito] + adj_FI)*porcentagem_mov) + pos_zero[SERVO_M2];
  angulos[SERVO_M3] = -((pos_un[posterior_direito] + adj_FS)*porcentagem_mov) + pos_zero[SERVO_M3];

  angulos[SERVO_M5] = ((pos_cn[posterior_esquerdo] + adj_FI)*porcentagem_mov) + pos_zero[SERVO_M5];
  angulos[SERVO_M6] = ((pos_un[posterior_esquerdo] + adj_FS)*porcentagem_mov) + pos_zero[SERVO_M6];

  angulos[SERVO_M8] = ((pos_cn[anterior_direito] + adj_TI)*porcentagem_mov) + pos_zero[SERVO_M8];
  angulos[SERVO_M9] = -((pos_un[anterior_direito] + adj_TS)*porcentagem_mov) + pos_zero[SERVO_M9];

  angulos[SERVO_M11] = -((pos_cn[anterior_esquerdo] + adj_TI)*porcentagem_mov) + pos_zero[SERVO_M11];
  angulos[SERVO_M12] = ((pos_un[anterior_esquerdo] + adj_TS)*porcentagem_mov) + pos_zero[SERVO_M12];

  angulos[SERVO_M1] =   converte_graus_para_radianos(qdiant_direito) + pos_zero[SERVO_M1];
  angulos[SERVO_M4] =  -converte_graus_para_radianos(qdiant_esquerdo) + pos_zero[SERVO_M4];

  angulos[SERVO_M7] =  -converte_graus_para_radianos(qtras_direito) + pos_zero[SERVO_M7];
  angulos[SERVO_M10] = converte_graus_para_radianos(qtras_esquerdo) + pos_zero[SERVO_M10];

  servo_a_ser_verificado(SERVO_M1);
  servo_a_ser_verificado(SERVO_M2);
  servo_a_ser_verificado(SERVO_M3);
  servo_a_ser_verificado(SERVO_M4);
  servo_a_ser_verificado(SERVO_M5);
  servo_a_ser_verificado(SERVO_M6);
  servo_a_ser_verificado(SERVO_M7);
  servo_a_ser_verificado(SERVO_M8);
  servo_a_ser_verificado(SERVO_M9);
  servo_a_ser_verificado(SERVO_M10);
  servo_a_ser_verificado(SERVO_M11);
  servo_a_ser_verificado(SERVO_M12);
  verifica_servo();

  #ifdef MODO_SIMULACAO
  Serial.println("PASSADA = " + (String)passo);
  Serial.println("Servo 1 = " + (String)angulos[SERVO_M1]);
  Serial.println("Servo 2 = " + (String)angulos[SERVO_M2]);
  Serial.println("Servo 3 = " + (String)angulos[SERVO_M3]);
  Serial.println("Servo 4 = " + (String)angulos[SERVO_M4]);
  Serial.println("Servo 5 = " + (String)angulos[SERVO_M5]);
  Serial.println("Servo 6 = " + (String)angulos[SERVO_M6]);
  Serial.println("Servo 7 = " + (String)angulos[SERVO_M7]);
  Serial.println("Servo 8 = " + (String)angulos[SERVO_M8]);
  Serial.println("Servo 9 = " + (String)angulos[SERVO_M9]);
  Serial.println("Servo 10 = " + (String)angulos[SERVO_M10]);
  Serial.println("Servo 11 = " + (String)angulos[SERVO_M11]);
  Serial.println("Servo 12 = " + (String)angulos[SERVO_M12]);
  #endif

  #ifndef MODO_SIMULACAO
  //if(!verifica_servos_em_movimento())
  #endif
  {
    //debug
    #ifdef MODO_DATALOGGER

    File localdataFile = SD.open(nome_arquivo, FILE_WRITE);
    
    // se o arquivo estiver presente, escreve:    
    unsigned char i;
    if(localdataFile)
    {
      dataString = "";
            
      le_dados_servos(ids, SERVO_REGISTER_PRESENT_ANGLE, NUM_SERVOS, dados_servo);
      
      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      le_dados_servos(ids, SERVO_REGISTER_PRESENT_TORQUE, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }

      le_dados_servos(ids, SERVO_REGISTER_PRESENT_VOLTAGE, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      le_dados_servos(ids, SERVO_REGISTER_CURRENT_CONSUMPTION, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      dataString.replace('.',',');
      
      localdataFile.println(dataString);
      localdataFile.close();
    }
    #endif

    #ifndef MODO_SIMULACAO
    servo_set_multiple(ids, SERVO_REGISTER_GOAL_ANGLE, angulos, NUM_SERVOS, 1);
    #endif
    passo++;
  
    if(passo >= NUM_PASSO)
    {
      reset_lista_servos();
      passo = 0;
    }
  }
}

void rev_trans_robo(void)
{
  static unsigned char passo = 0;
  static unsigned char anterior_direito = 0;
  static unsigned char anterior_esquerdo = 0;
  static unsigned char posterior_direito = 0;
  static unsigned char posterior_esquerdo = 0;

  static float adj_TS;
  static float adj_TI;
  static float adj_FS;
  static float adj_FI;

  static float  porcentagem_mov = 1.0;

  unsigned char i;

  static bool flag_inicializacao = 0;
 
  if(!flag_inicializacao)
  {   
    for(i=0;i<NUM_SERVOS;i++)
    {
      angulos[i] = pos_zero[i];
    }

    adj_TS = converte_graus_para_radianos(F_TS);
    adj_TI = converte_graus_para_radianos(F_TI);
    adj_FS = converte_graus_para_radianos(F_FS);
    adj_FI = converte_graus_para_radianos(F_FI);
    
    flag_inicializacao = 1;
  }
 
  anterior_direito = passo;

  if(passo == 5){

    if(rqdiant_direito>0){
    
    rqdiant_direito+= -15;
    }

    Serial.println("Servo 1 = " + (String)angulos[SERVO_M1]);
  }
  
  posterior_direito = passo + 15;
  if(posterior_direito >= NUM_PASSO)
  {
    posterior_direito -= NUM_PASSO;
  }

  if(passo == 19){

    if(rqtras_direito>0){
    
    rqtras_direito += -15;
    }
    Serial.println("Servo 4 = " + (String)angulos[SERVO_M4]);
  }


  anterior_esquerdo = passo + 10;
  if(anterior_esquerdo >= NUM_PASSO)
  {
    anterior_esquerdo -= NUM_PASSO;
  }
  
  if(passo == 15){

    if(rqdiant_esquerdo>0){
    
    rqdiant_esquerdo += -15;
    }
    Serial.println("Servo 7 = " + (String)angulos[SERVO_M7]);
  }

  posterior_esquerdo = passo + 5;
  if(posterior_esquerdo >= NUM_PASSO)
  {
    posterior_esquerdo -= NUM_PASSO;
  }

  if(passo == 9){

    if(rqtras_esquerdo>0){
    
    rqtras_esquerdo += -15;
    }

    Serial.println("Servo 10 = " + (String)angulos[SERVO_M10]);
  }

  angulos[SERVO_M2] = -((pos_cn[posterior_direito] + adj_FI)*porcentagem_mov) + pos_zero[SERVO_M2];
  angulos[SERVO_M3] = -((pos_un[posterior_direito] + adj_FS)*porcentagem_mov) + pos_zero[SERVO_M3];

  angulos[SERVO_M5] = ((pos_cn[posterior_esquerdo] + adj_FI)*porcentagem_mov) + pos_zero[SERVO_M5];
  angulos[SERVO_M6] = ((pos_un[posterior_esquerdo] + adj_FS)*porcentagem_mov) + pos_zero[SERVO_M6];

  angulos[SERVO_M8] = ((pos_cn[anterior_direito] + adj_TI)*porcentagem_mov) + pos_zero[SERVO_M8];
  angulos[SERVO_M9] = -((pos_un[anterior_direito] + adj_TS)*porcentagem_mov) + pos_zero[SERVO_M9];

  angulos[SERVO_M11] = -((pos_cn[anterior_esquerdo] + adj_TI)*porcentagem_mov) + pos_zero[SERVO_M11];
  angulos[SERVO_M12] = ((pos_un[anterior_esquerdo] + adj_TS)*porcentagem_mov) + pos_zero[SERVO_M12];

  angulos[SERVO_M1] =   converte_graus_para_radianos(rqdiant_direito) + pos_zero[SERVO_M1];
  angulos[SERVO_M4] =  -converte_graus_para_radianos(rqdiant_esquerdo) + pos_zero[SERVO_M4];

  angulos[SERVO_M7] =  -converte_graus_para_radianos(rqtras_direito) + pos_zero[SERVO_M7];
  angulos[SERVO_M10] = converte_graus_para_radianos(rqtras_esquerdo) + pos_zero[SERVO_M10];

  servo_a_ser_verificado(SERVO_M1);
  servo_a_ser_verificado(SERVO_M2);
  servo_a_ser_verificado(SERVO_M3);
  servo_a_ser_verificado(SERVO_M4);
  servo_a_ser_verificado(SERVO_M5);
  servo_a_ser_verificado(SERVO_M6);
  servo_a_ser_verificado(SERVO_M7);
  servo_a_ser_verificado(SERVO_M8);
  servo_a_ser_verificado(SERVO_M9);
  servo_a_ser_verificado(SERVO_M10);
  servo_a_ser_verificado(SERVO_M11);
  servo_a_ser_verificado(SERVO_M12);
  verifica_servo();

  #ifdef MODO_SIMULACAO
  Serial.println("PASSADA = " + (String)passo);
  Serial.println("Servo 1 = " + (String)angulos[SERVO_M1]);
  Serial.println("Servo 2 = " + (String)angulos[SERVO_M2]);
  Serial.println("Servo 3 = " + (String)angulos[SERVO_M3]);
  Serial.println("Servo 4 = " + (String)angulos[SERVO_M4]);
  Serial.println("Servo 5 = " + (String)angulos[SERVO_M5]);
  Serial.println("Servo 6 = " + (String)angulos[SERVO_M6]);
  Serial.println("Servo 7 = " + (String)angulos[SERVO_M7]);
  Serial.println("Servo 8 = " + (String)angulos[SERVO_M8]);
  Serial.println("Servo 9 = " + (String)angulos[SERVO_M9]);
  Serial.println("Servo 10 = " + (String)angulos[SERVO_M10]);
  Serial.println("Servo 11 = " + (String)angulos[SERVO_M11]);
  Serial.println("Servo 12 = " + (String)angulos[SERVO_M12]);
  #endif

  #ifndef MODO_SIMULACAO
  //if(!verifica_servos_em_movimento())
  #endif
  {
    //debug
    #ifdef MODO_DATALOGGER

    File localdataFile = SD.open(nome_arquivo, FILE_WRITE);
    
    // se o arquivo estiver presente, escreve:    
    unsigned char i;
    if(localdataFile)
    {
      dataString = "";
            
      le_dados_servos(ids, SERVO_REGISTER_PRESENT_ANGLE, NUM_SERVOS, dados_servo);
      
      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      le_dados_servos(ids, SERVO_REGISTER_PRESENT_TORQUE, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }

      le_dados_servos(ids, SERVO_REGISTER_PRESENT_VOLTAGE, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      le_dados_servos(ids, SERVO_REGISTER_CURRENT_CONSUMPTION, NUM_SERVOS, dados_servo);

      for(i=0; i<NUM_SERVOS; i++)
      {
        dataString += (String)dados_servo[i];
        dataString += ';'; 
      }
      
      dataString.replace('.',',');
      
      localdataFile.println(dataString);
      localdataFile.close();
    }
    #endif

    #ifndef MODO_SIMULACAO
    servo_set_multiple(ids, SERVO_REGISTER_GOAL_ANGLE, angulos, NUM_SERVOS, 1);
    #endif
    passo++;
  
    if(passo >= NUM_PASSO)
    {
      reset_lista_servos();
      passo = 0;
    }
  }
}






#ifdef MODO_DATALOGGER
void grava_arquivo_eeprom(void)
{
    dataFile = SD.open("eeprom.txt", FILE_WRITE);
    if(dataFile)
    {
      dataFile.write(dado_eeprom);
      dataFile.close();
    }       
}

void le_arquivo_eeprom(void)
{
  dataFile = SD.open("eeprom.txt");
  while(dataFile.available())
  {    
    dado_eeprom = dataFile.read();
  }
  dataFile.close();
}
#endif

void debug_thread(){
  flag_debug = !flag_debug;

  digitalWrite(LED_DEBUG, flag_debug);

//  Serial.print("Rodando em: ");
//  Serial.println(millis());
}

void servo_thread()
{
  SM_robo();
}

void protecao_thread()
{
  #ifndef MODO_SIMULACAO 
  if(!verifica_servos())
  {
    sm_robo = SM_PARA;
  }
  #endif
}

void setup(void)
{ 
  Serial.begin(9600);
  Serial.setTimeout(10);

  #ifndef MODO_SIMULACAO  
  servo_init(&Serial1, HALF_DUPLEX_DIRECTION_PIN, SERVO_BAUD_RATE);
  #endif
     
  pinMode(LED_DEBUG, OUTPUT);
  digitalWrite(LED_DEBUG, LOW);
  
  Thread1.onRun(debug_thread);
  Thread1.setInterval(500);

  Thread2.onRun(servo_thread);

  #ifdef MODO_SIMULACAO
  Thread2.setInterval(1000);
  #else
  Thread2.setInterval(250);   //<-- MUDA A VELOCIDADE DA MÁQUINA DE ESTADO DOS SERVOS (ORIGINAL 250)
  #endif 

  Thread3.onRun(protecao_thread);
  Thread3.setInterval(50);

  controle.add(&Thread1);
  controle.add(&Thread2);
  controle.add(&Thread3);

  #ifndef MODO_SIMULACAO
  aguarda_servos();
  #else
  Serial.println("---- MODO DE SIMULACAO! ----");
  #endif 

  #ifdef MODO_DATALOGGER

  Serial.println("---- MODO DATALOGGER! ----");
  pinMode(PINO_LED_1, OUTPUT);
  pinMode(PINO_LED_2, OUTPUT);
  pinMode(PINO_LED_GND, OUTPUT);

  digitalWrite(PINO_LED_1, LOW); 
  digitalWrite(PINO_LED_2, HIGH); 
  digitalWrite(PINO_LED_GND, LOW);
  
  if(!SD.begin(CS_SPI))
  {
    // NAO CONECTADO
    digitalWrite(PINO_LED_1, LOW); 
    digitalWrite(PINO_LED_2, HIGH); 
    digitalWrite(PINO_LED_GND, LOW);
    return;
  }
  else
  {
    // CONECTADO
    digitalWrite(PINO_LED_1, HIGH); 
    digitalWrite(PINO_LED_2, LOW); 
    digitalWrite(PINO_LED_GND, LOW);

    if(SD.exists("eeprom.txt"))
    {      
      le_arquivo_eeprom();  
    }
    else 
    {
      grava_arquivo_eeprom();
    }
  }
  #endif
}

/*----------------------------------------------------*/
//                    PRINCIPAL
/*----------------------------------------------------*/

void loop(void)
{
  controle.run();

  if(Serial.available() > 0)
  {
    buf = Serial.readString();
    
    while(buf == "ZERO")
    {
      sm_robo = SM_ZERO;
      Serial.println("OK");
         
    }

    if(buf == "ANDA")
    {
      sm_robo = SM_ANDA;
      Serial.println("OK");    
    }

    if(buf == "TRANS")
    {
      sm_robo = SM_TRANS;
      Serial.println("OK");    
    }

    if(buf == "PARA")
    {
      sm_robo = SM_PARA;
      Serial.println("OK");    
    }

    if(buf == "RESET")
    {
      sm_robo = SM_RESET;
      Serial.println("OK");    
    }

     if(buf == "REV")
    {
      sm_robo = SM_REV;
      Serial.println("OK");    
    } 
  }
}
