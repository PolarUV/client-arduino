#include <SPI.h>
#include <UIPEthernet.h>
#include <USB.h>
#include <PS5USB.h>

//#define DEBUG

// ############################## Ethernet ##############################

const byte Mac[] = {0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
const IPAddress RobotIP(192, 168, 1, 1);
const IPAddress RemoteIP(192, 168, 1, 2);
constexpr uint16_t Port = 8888;

EthernetUDP UDP;

// ############################## USB ##############################

uint16_t LastMessageCounter = -1;

USB USBPort{};
PS5USB PS5(&USBPort);

// ############################## Стабилизация ##############################

struct RequestPacketStruct
{
  // Режим работы робота
  uint8_t Mode = 0;
  // Координаты стиков
  uint8_t LeftStickX = 127;
  uint8_t LeftStickY = 127;
  uint8_t RightStickX = 127;
  uint8_t RightStickY = 127;
  // Состояния кнопок
  uint8_t ButtonL1 = 0;
  uint8_t ButtonL2 = 0;
  uint8_t ButtonR1 = 0;
  uint8_t ButtonR2 = 0;
};

struct ResponsePacketStruct
{
  uint8_t state;
};

enum Mode
{
  RemoteControl,
  Stabilization,
  Parking
};
uint8_t CurrentMode = Mode::RemoteControl;

// ############################## Setup ##############################

void setup()
{
  // Выбор номера вывода CS
  Ethernet.init(8);

  // Запуск Ethernet
  Ethernet.begin(Mac, RemoteIP);

  // Open serial communications and wait for port to open:
  #ifdef DEBUG
  Serial.begin(9600);
  while (!Serial);
  #endif

  // Check for Ethernet hardware status
  #ifdef DEBUG
  Serial.print("Ethernet hardware status: ");
  #endif
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
  #ifdef DEBUG
    Serial.println("no hardware");
    Serial.println("ERROR. Stopping program...");
  #endif
    while(1);
  }
  else
  {
  #ifdef DEBUG
    Serial.println("ethernet controller found");
  #endif
  }

  #ifdef DEBUG
  // Check for ethernet cable status
  Serial.print("Ethernet cable status: ");
  EthernetLinkStatus linkStatus = Ethernet.linkStatus();
  switch (linkStatus)
  {
    case LinkON:
      Serial.println("cable found");
      break;
    case LinkOFF:
      Serial.println("no cable");
      break;
    case Unknown:
      Serial.println("unknown");
      break;
  }
  #endif

  // Инициализация UDP
  UDP.begin(Port);

  // Инициализация USB
  if (USBPort.Init() == -1)
  {
  #ifdef DEBUG
    Serial.println("ERROR. Can't initialize USB");
  #endif
    while (1);
  }

  #ifdef DEBUG
  Serial.println("Setup completed");
  #endif
}

// ############################## Loop ##############################

void loop()
{
  USBPort.Task();

  if (PS5.connected() && LastMessageCounter != PS5.getMessageCounter())
  {
    LastMessageCounter = PS5.getMessageCounter();

    // Создание пакета
    RequestPacketStruct requestPacket;

    // Считывание состояния геймпада
    uint8_t leftStickX = PS5.getAnalogHat(AnalogHatEnum::LeftHatX);
    uint8_t leftStickY = PS5.getAnalogHat(AnalogHatEnum::LeftHatY);
    uint8_t rightStickX = PS5.getAnalogHat(AnalogHatEnum::RightHatX);
    uint8_t rightStickY = PS5.getAnalogHat(AnalogHatEnum::RightHatY);
    uint8_t buttonL2 = PS5.getAnalogButton(ButtonEnum::L2);
    uint8_t buttonR2 = PS5.getAnalogButton(ButtonEnum::R2);

    // Заполнение пакета
    if (PS5.getButtonClick(ButtonEnum::TRIANGLE))
    {
      if (CurrentMode != Mode::Stabilization) CurrentMode = Mode::Stabilization;
    }
    else if (PS5.getButtonClick(ButtonEnum::SQUARE))
    {
      if (CurrentMode != Mode::Parking) CurrentMode = Mode::Parking;
    }
    else if (PS5.getButtonClick(ButtonEnum::CROSS))
    {
      if (CurrentMode != Mode::RemoteControl) CurrentMode = Mode::RemoteControl;
    }
    
    requestPacket.Mode = CurrentMode;
    requestPacket.LeftStickX = (117 < leftStickX && leftStickX < 137) ? 127 : leftStickX;
    requestPacket.LeftStickY = (117 < leftStickY && leftStickY < 137) ? 127 : leftStickY;
    requestPacket.RightStickX = (117 < rightStickX && rightStickX < 137) ? 127 : rightStickX;
    requestPacket.RightStickY = (117 < rightStickY && rightStickY < 137) ? 127 : rightStickY;
    requestPacket.ButtonL1 = PS5.getButtonPress(L1);
    requestPacket.ButtonL2 = (117 < buttonL2 && buttonL2 < 137) ? 127 : buttonL2;
    requestPacket.ButtonR1 = PS5.getButtonPress(R1);
    requestPacket.ButtonR2 = (117 < buttonR2 && buttonR2 < 137) ? 127 : buttonR2;

    // Упаковка пакета
    char buffer[sizeof(RequestPacketStruct)];
    memcpy(buffer, &requestPacket, sizeof(RequestPacketStruct));

    // Отправка пакета
    UDP.beginPacket(RobotIP, Port);
    UDP.write(buffer,sizeof(buffer));
    UDP.endPacket();

    #ifdef DEBUG
    Serial.println("Packet sent");
    #endif

    // Прием ответа от робота
    char replyBuffer;
    int packetSize = UDP.parsePacket();
    if (packetSize)
    {
      UDP.read(&replyBuffer, 1);
    }

    // Расшифровка ответа от робота
    switch (replyBuffer)
    {
      case 'R':
      {
        PS5.setLed(ColorsEnum::Green);
        break;
      }
      case 'S':
      {
        PS5.setLed(ColorsEnum::Blue);
        break;
      }
      case 'P':
      {
        PS5.setLed(ColorsEnum::White);
        break;
      }
      default:
      {
        PS5.setLed(ColorsEnum::Red);
        break;
      }
    }
  }

  // Задержка перед отправкой следующего пакета
  delay(50);
}
