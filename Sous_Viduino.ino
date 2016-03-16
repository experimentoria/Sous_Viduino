
//-------------------------------------------------------------------
// Controlador Sous Vide
// Originalmente criado por Bill Earl - para Adafruit Industries
//
// Traducão para português, adaptacão para LCD comum de 16x2, botões extras e cronômetro de tempo
// por Guz Forster, Leonel Braz e Gustavo J. M. Forster - experimentoria.com.br
// 
// Usando as bibliotecas de PID e Autotune PID para Arduino por Brett Beauregard
//------------------------------------------------------------------

// Bibliotecas PID
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Biblioteca do LCD
#include <LiquidCrystal.h>

// Bibliotecas para o sensor de temperatura a prova d'água DS18B20 (ou compatível)
#include <OneWire.h>
#include <DallasTemperature.h>

// Biblioteca EEPROM para que possamos guardar as últimas configuracões do controlador no Arduino
#include <EEPROM.h>

// ************************************************
// Definicões dos Pinos
// ************************************************

// Relé
#define RelayPin 8

// Pino do sensor de temperatura
#define PIN_SENSOR_TEMP 9


// Define todos os pinos de botões
#define PIN_BTN_PARACIMA  A0
#define PIN_BTN_PARABAIXO A1
#define PIN_BTN_ESQUERDA  A2
#define PIN_BTN_DIREITA   A3
#define PIN_BTN_SHIFT     A4

// matriz com todos os pinos dos botões do sistema
byte button_pins[] = {PIN_BTN_PARACIMA,PIN_BTN_PARABAIXO,PIN_BTN_ESQUERDA,PIN_BTN_DIREITA,PIN_BTN_SHIFT};

byte old_estadoBotao = 0;
byte estadoBotao = 0;

byte pinoBotaoPressionado=0;

// inicializa o LCD com os números dos pinos
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// ************************************************
// Variáveis e constantes PID
// ************************************************

// Definindo variáveis que iremos conectar
double  Setpoint,	// variável que usuário irá definir o valor da temperatura desejada
		    Input, 		// variávei que recebe o valor de temperatura do sensor
		    Output;		// Guz: verificar como essa variável se conecta com a onTime

volatile long onTime = 0; // Guz: verificar para que serve essa variável

// Variáveis de ajustes do PID
double  Kp,
		    Ki,
		    Kd;

// Enderecos da EEPROM para guardarmos os dados;
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;


// Cria o objeto PID com aa referências de Input, Output e Setpoint (usando bitwise AND - &) além dos parâmetros iniciais (Kp, Ki, Kd)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// Definindo variáveis para tempo proporcional de 10 segundos
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Variáveis e constantes do Auto Tune
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

bool tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// Mostrar variáveis e constantes
// ************************************************

//#define PIN_BTN_SHIFT 1

unsigned long lastInput = 0; // último botão pressionado

const int logInterval = 10000; // log a cada 10 segundos (para saída na Serial)
long lastLogTime = 0;

// ************************************************
// Estados para as telas
// ************************************************
enum operatingState
{
	OFF = 0,
	RUN,
  SETP,
	TUNE_P,
	TUNE_I,
	TUNE_D,
	AUTO
};

byte arrOperatingState[]=
{
  OFF,
  RUN,
  SETP,
  TUNE_P,
  TUNE_I,
  TUNE_D,
  AUTO
};

byte stateIndex = 0;

// ************************************************
// Variáveis e constantes do sensor
// Fio de dados do sensor é ligado no pino 2 do Arduino (definido acima)

// Cria uma instância do objeto OneWire para comunicar com qualquer dispositivo OneWire (e não apenas CIs de temperatura da Maxim/Dallas)
OneWire oneWire(PIN_SENSOR_TEMP);

// Passa a referência oneWire criada para o objeto Dallas Temperature.
DallasTemperature sensors(&oneWire);

// A array que guarda o endereco do sensor (definida na biblioteca Dallas Termperature)
DeviceAddress tempSensor;

byte buttonState_DIR = digitalRead(PIN_BTN_DIREITA);
byte buttonState_ESQ = digitalRead(PIN_BTN_ESQUERDA);
byte buttonState_CIM = digitalRead(PIN_BTN_PARACIMA);
byte buttonState_BAI = digitalRead(PIN_BTN_PARABAIXO);
byte buttonState_SHI = digitalRead(PIN_BTN_SHIFT);

byte botoes[]= {buttonState_DIR, buttonState_ESQ, buttonState_CIM, buttonState_BAI, buttonState_SHI};

// ************************************************
// Setup e exibe a tela principal
// ************************************************
void setup()
{
   Serial.begin(9600);
   // Serial.println("teste");
   // Modos de pino para os botões:
   for (byte i=0; i < 5; i++)
   {
      pinMode(button_pins[i], INPUT);
   }
   
   // Inicializa o controle do relé:

   pinMode(RelayPin, OUTPUT);    // Modo de output (saída) para o relé
   digitalWrite(RelayPin, LOW);  // Para ter certeza que está desligado ao iniciar

   // Inicializa o LCD

   lcd.begin(16, 2);
   
   lcd.print(F(" Experimentoria"));
   lcd.setCursor(0, 1);
   lcd.print(F("   Maker Chef"));

   // Inicializa o sensor de temperatura

   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) // Verifica se o sensor existe / se está dando leitura
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Erro no sensor!"));
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);

   delay(3000);  // Tela de abertura

   // Inicializa o PID e variáveis relacionadas
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Roda o interruptor do Timer 2 do Arduino a cada 15ms
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  // Habilita o overflow do interruptor do Timer 2 do Arduino
  TIMSK2 |= 1<<TOIE2;
}

// ************************************************
// Funcão que lida com o interrutor do timer
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (arrOperatingState[stateIndex] == OFF)
  {
    digitalWrite(RelayPin, LOW);  // ter a certeza que o relé está desligado
  }
  else
  {
    DriveOutput(); // Se o estado/tela não for a OFF, executa saídas do relé
  }
}

// ************************************************
// Loop principal do controle
//
// Todas as mudancas de tela passam por aqui
// ************************************************
void loop()
{   
      //faca algo somente quando soltar o botao
      while(checaBotoes()!=0) {};

      lcd.clear(); //limpa o LCD

      //atualiza
      switch (arrOperatingState[stateIndex])
       {
       case OFF:
          Off();
          break;
       case SETP:
          Tune_Sp();
          break;
        case RUN:
          Run();
          break;
       case TUNE_P:
          TuneP();
          break;
       case TUNE_I:
          TuneI();
          break;
       case TUNE_D:
          TuneD();
          break;
       } 

}

byte checaBotoes(){
  for (byte i=0;i<5;i++)
  {
    estadoBotao = digitalRead(button_pins[i]);
    pinoBotaoPressionado = button_pins[i];
    if(estadoBotao==1) break;
  }

  return estadoBotao;
}

void gotoNext(){
  stateIndex = (stateIndex==5) ? 0 : stateIndex+1;
}

void gotoPrev(){
  stateIndex = (stateIndex==0) ? 5 : stateIndex-1;
}

// ************************************************
// Estado/tela inicial - pressionando para a DIREITA
// entra na configuracão da temperatura desejada
// ************************************************
void Off()
{
  myPID.SetMode(MANUAL);
  digitalWrite(RelayPin, LOW);  // ter a certeza de que o relé está desligado
  lcd.print(F(" Experimentoria"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Maker Chef"));
  old_estadoBotao = 0;

  if (digitalRead(PIN_BTN_DIREITA) == HIGH)
  {
       // Preparando para entrar no modo RUN (operacão)
       sensors.requestTemperatures(); // Inicia a leitura da temperatura

       //liga o PID
       myPID.SetMode(AUTOMATIC);
       windowStartTime = millis();

       gotoNext();
   }
}

// ************************************************
// Estado / tela do Setpoint
// PRA CIMA/PRA BAIXO muda os valores do setpoint
// PRA DIREITA muda para tela/estado de ajustar parâmetros PID
// PRA ESQUERDA muda para tela/estado de OFF (desliga o PID)
// BOTÃO SHIFT junto com PRA CIMA ou PRA BAIXO para o ajuste ser multiplicado por 10
// ************************************************
void Tune_Sp()
{  
   lcd.print(F("Temp. desejada:"));
   old_estadoBotao = 0;
   while(true)
   {

      float increment = 0.1;
      if (digitalRead(PIN_BTN_SHIFT) == HIGH)
      {
        increment *= 10;
      }
      if (digitalRead(PIN_BTN_ESQUERDA) == HIGH)
      {
         gotoPrev();
         return;
      }
      if (digitalRead(PIN_BTN_DIREITA) == HIGH)
      {
         gotoNext();
         return;
      }
      if (digitalRead(PIN_BTN_PARACIMA) == HIGH)
      {
         Setpoint += increment;
         delay(200);
      }
      if (digitalRead(PIN_BTN_PARABAIXO) == HIGH)
      {
         Setpoint -= increment;
         delay(200);
      }
    
      // if ((millis() - lastInput) > 3000)  // volta para tela de execucão depois de 3 segundos sem atividade
      // {
      // arrOperatingState[stateIndex] = RUN;
      //    return;
      // }
      lcd.setCursor(0,1);
      lcd.print(Setpoint);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Tela / estado de ajuste do Kp - P(id)
// PRA CIMA/PRA BAIXO mudam os valores
// PRA DIREITA vai para a tela de ajuste do Ki - p(I)d
// PRA ESQUERDA vai para a tela / estado de ajuste do setPoin (temperatura)
// BOTÃO SHIFT junto com PRA CIMA ou PRA BAIXO para o ajuste ser multiplicado por 10
// ************************************************
void TuneP()
{
   lcd.print(F("Ajuste o Kp:"));

   old_estadoBotao = 0;
   while(true)
   {

      float increment = 1.0;
      if (digitalRead(PIN_BTN_SHIFT) == HIGH)
      {
        increment *= 10;
      }
      if (digitalRead(PIN_BTN_ESQUERDA) == HIGH)
      {
         gotoPrev();
         return;
      }
      if (digitalRead(PIN_BTN_DIREITA) == HIGH)
      {
         Serial.println(" ========= DIR ============ ");
         gotoNext();
         return;
      }
      if (digitalRead(PIN_BTN_PARACIMA) == HIGH)
      {
         Kp += increment;
         delay(200);
      }
      if (digitalRead(PIN_BTN_PARABAIXO) == HIGH)
      {
         Kp -= increment;
         delay(200);
      }
      // if ((millis() - lastInput) > 3000)  // volta para tela de execucão depois de 3 segundos sem atividade
      // {
      // stateIndex = 1;
      //    return;
      // }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Tela / estado de ajuste do Ki p(I)d
// PRA CIMA/PRA BAIXO mudam os valores
// PRA DIREITA vai para a tela de ajuste do Kd - pi(D)
// PRA ESQUERDA vai para a tela de ajuste do Kp - (P)id
// BOTÃO SHIFT junto com PRA CIMA ou PRA BAIXO para o ajuste ser multiplicado por 10
// ************************************************
void TuneI()
{
   lcd.print(F("Ajuste o Ki"));

   old_estadoBotao = 0;
   while(true)
   {

      float increment = 0.01;
      if (digitalRead(PIN_BTN_SHIFT) == HIGH)
      {
        increment *= 10;
      }
      if (digitalRead(PIN_BTN_ESQUERDA) == HIGH)
      {
         gotoPrev();
         return;
      }
      if (digitalRead(PIN_BTN_DIREITA) == HIGH)
      {
         Serial.println(" ========= DIR ============ ");
         gotoNext();
         return;
      }
      if (digitalRead(PIN_BTN_PARACIMA) == HIGH)
      {
         Ki += increment;
         delay(200);
      }
      if (digitalRead(PIN_BTN_PARABAIXO) == HIGH)
      {
         Ki -= increment;
         delay(200);
      }
      // if ((millis() - lastInput) > 3000)  // volta para tela de execucão depois de 3 segundos sem atividade
      // {
      //    stateIndex = 1;
      //    return;
      // }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Tela / estado de ajuste do Kd - pi(D)
// PRA CIMA/PRA BAIXO mudam os valores
// PRA DIREITA vai para a tela de ajuste do setPoint (temperatura)
// PRA ESQUERDA vai para a tela de ajuste do Ki - p(I)d
// BOTÃO SHIFT junto com PRA CIMA ou PRA BAIXO para o ajuste ser multiplicado por 10
// ************************************************
void TuneD()
{
   lcd.print(F("Ajuste o Kd:"));

   old_estadoBotao = 0;
   while(true)
   {
      float increment = 0.01;
      if (digitalRead(PIN_BTN_SHIFT) == HIGH)
      {
        increment *= 10;
      }
      if (digitalRead(PIN_BTN_ESQUERDA) == HIGH)
      {
         gotoPrev();
         return;
      }
      if (digitalRead(PIN_BTN_DIREITA) == HIGH)
      {
         Serial.println(" ========= DIR ============ ");
         gotoNext();
         return;
      }
      if (digitalRead(PIN_BTN_PARACIMA) == HIGH)
      {
         Kd += increment;
         delay(200);
      }
      if (digitalRead(PIN_BTN_PARABAIXO) == HIGH)
      {
         Kd -= increment;
         delay(200);
      }
      // if ((millis() - lastInput) > 3000)  // volta para tela de execucão depois de 3 segundos sem atividade
      // {
      //    stateIndex = 1;
      //    return;
      // }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Tela de controle do PID
// BOTÃO SHIFT junto com DIREITA para acionar o Autotune
// PRA DIREITA - ajuste do Setpoint (temperatura)
// PRA ESQUERDA - tela/estado de OFF (desliga o PID)
// ************************************************
void Run()
{   
   lcd.print(F("Sp: "));
   lcd.print(Setpoint);
   lcd.write(223); // imprime o caractere "º" (graus)
   lcd.print(F("C"));

   SaveParameters(); // funcão de salvar os parâmetros atuais na EEPROM
   myPID.SetTunings(Kp,Ki,Kd); // estabelece os valores iniciais do PID

   old_estadoBotao = 0;
   while(true)
   {
      if ((digitalRead(PIN_BTN_SHIFT) == HIGH) 
         && (digitalRead(PIN_BTN_DIREITA) == HIGH) 
         && (abs(Input - Setpoint) < 0.5))  // só inicia o Autotune se a temperatura estiver estável dentro do valor estabelecido pelo usuário
      {
         StartAutoTune();
      }
      else if (digitalRead(PIN_BTN_DIREITA) == HIGH)
      {
        gotoNext();
        return;
      }
      else if (digitalRead(PIN_BTN_ESQUERDA) == HIGH)
      {
        gotoPrev();
        return;
      }
      
      DoControl();
      
      lcd.setCursor(0,1);
      lcd.print(Input);
      lcd.write(223);
      lcd.print(F("C "));
      
      float pct = map(Output, 0, WindowSize, 0, 1000);
      lcd.setCursor(9,1);
      lcd.print(pct/10);
      //lcd.print(Output);
      lcd.print("%");

      lcd.setCursor(15,0);
      if (tuning)
      {
        lcd.print("T");
      }
      else
      {
        lcd.print(" ");
      }
      
      // periodicamente imprime valores pela porta serial em formato CSV
      if (millis() - lastLogTime > logInterval)  
      {
         Serial.print(Input);
         Serial.print(",");
         Serial.println(Output);
      }

      delay(100);
   }
}

// ************************************************
// Executa o loop de controle do PID
// ************************************************
void DoControl()
{
  // Ler o input:
  if (sensors.isConversionAvailable(0)) // verifica se recebeu alguma mudanca de temperatura no sensor
  {
    Input = sensors.getTempC(tempSensor); // adquire a temperatura em ºC
    sensors.requestTemperatures(); // inicializa a funcão para converter o valor digital em temperatura
  }
  
  if (tuning) // roda o auto-tuner
  {
     if (aTune.Runtime()) // retorna "true" quando finalizado
     {
        FinishAutoTune(); // finaliza o auto-tune
     }
  }
  else // Executa o algoritmo de controle PID
  {
     myPID.Compute();
  }
  
  // O estado proporcional de tempo do relé é atualizado regularmente pelo interruptor do Timer 2
  onTime = Output; 
}

// ************************************************
// Chamado pelo ISR a cada 15ms para ativar a saída
// no relé
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Determina a saída
  // "on time" é proporcional à saída do PID
  if(now - windowStartTime>WindowSize)
  { //hora de mudar o estado do relé
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Inicia o ciclo do Auto-Tune
// ************************************************

void StartAutoTune()
{
   // Lembra o modo anterior
   ATuneModeRemember = myPID.GetMode();

   // ajusta os parâmetros do auto-tune
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Retorna para o controle normal
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extrai os parâmetros calculados do auto-tune
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Ajusta novamente os valores do PID e retorna para o controle normal
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Salva qualquer parâmetro modificado na EEPROM
   SaveParameters();
}

// ************************************************
// Verifica botões pressionaos e marca o tempo desde
// o útlimo botão pressionado
// ************************************************


int botaoApertado()
{
  int reply;

  for (uint8_t i=0; i<5; i++) {
    reply = digitalRead(button_pins[i]);
  }

  if (reply != 0)
  {
    lastInput = millis();
  }

  return reply;
}


// ************************************************
// Salva quaisquer parâmetros modificados na EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Carrega parâmetros da EEPROM
// ************************************************
void LoadParameters()
{
  // carrega da EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Usa esses valores padrões caso os valores da EEPROM sejam inválidos/inexistentes
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


// ************************************************
// Escreve valores de ponto fluante na EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Lê valores de ponto fluante da EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
