#include <SPI.h>
#include <UIPEthernet.h>
#include <USB.h>
#include <PS5USB.h>

//#define DEBUG

// ############################## Ethernet ##############################

constexpr byte Mac[] = {0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
const IPAddress RobotIP(192, 168, 1, 1);
const IPAddress RemoteIP(192, 168, 1, 2);
constexpr uint16_t Port = 8888;

EthernetUDP UDP;

// ############################## Двигатели ##############################

constexpr int16_t MotorCoefficients[6][6] =
    {
        // Fx    Fy    Fz   Mx   My  Mz
        {0, -100, 100, 100, 100, 0},  // Двигатель 1
        {100, 0, 0, 0, 0, -100},      // Двигатель 2
        {0, 100, -100, 100, 100, 0},  // Двигатель 3
        {0, 100, 100, -100, 100, 0},  // Двигатель 4
        {-100, 0, 0, 0, 0, -100},     // Двигатель 5
        {0, -100, -100, -100, 100, 0} // Двигатель 6
};

// ############################## USB ##############################

uint16_t LastMessageCounter = -1;

USB USBPort{};
PS5USB PS5(&USBPort);

// ############################## Стабилизация ##############################

struct RequestPacketStruct
{
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
uint8_t RobotState{};

// ############################## Setup ##############################

void setup()
{
  // Выбор номера вывода CS
  Ethernet.init(8);

  // Запуск Ethernet
  Ethernet.begin(Mac, RemoteIP);

// Open serial communications and wait for port to open:
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial)
    ;
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
    while (1)
      ;
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
    while (1)
      ;
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

    if (RobotState != 'S' && RobotState != 'W')
    {
      // Отправка пакета
      const auto* buffer = reinterpret_cast<const char*>(MotorCoefficients);
      constexpr static auto bufferSize = sizeof(MotorCoefficients);
      UDP.beginPacket(RobotIP, Port);
      UDP.write(buffer, bufferSize);
      UDP.endPacket();

#ifdef DEBUG
      Serial.println("Coefficients packet sent");
#endif
    }
    else
    {
      // Считывание состояния геймпада
      const uint8_t leftStickX = PS5.getAnalogHat(AnalogHatEnum::LeftHatX);
      const uint8_t leftStickY = PS5.getAnalogHat(AnalogHatEnum::LeftHatY);
      const uint8_t rightStickX = PS5.getAnalogHat(AnalogHatEnum::RightHatX);
      const uint8_t rightStickY = PS5.getAnalogHat(AnalogHatEnum::RightHatY);
      const uint8_t buttonL2 = PS5.getAnalogButton(ButtonEnum::L2);
      const uint8_t buttonR2 = PS5.getAnalogButton(ButtonEnum::R2);

      // Создание пакета
      RequestPacketStruct requestPacket;
      requestPacket.LeftStickX = (117 < leftStickX && leftStickX < 137) ? 127 : leftStickX;
      requestPacket.LeftStickY = (117 < leftStickY && leftStickY < 137) ? 127 : leftStickY;
      requestPacket.RightStickX = (117 < rightStickX && rightStickX < 137) ? 127 : rightStickX;
      requestPacket.RightStickY = (117 < rightStickY && rightStickY < 137) ? 127 : rightStickY;
      requestPacket.ButtonL1 = PS5.getButtonPress(L1);
      requestPacket.ButtonL2 = (117 < buttonL2 && buttonL2 < 137) ? 127 : buttonL2;
      requestPacket.ButtonR1 = PS5.getButtonPress(R1);
      requestPacket.ButtonR2 = (117 < buttonR2 && buttonR2 < 137) ? 127 : buttonR2;

      // Упаковка пакета
      auto* buffer = reinterpret_cast<char*>( &requestPacket);
      static constexpr auto buffer_size = sizeof(requestPacket);
      // Отправка пакета
      UDP.beginPacket(RobotIP, Port);
      UDP.write(buffer, buffer_size);
      UDP.endPacket();

#ifdef DEBUG
      Serial.println("Commands packet sent");
#endif
    }

    RobotState = '0';
    // Прием ответа от робота
    if (UDP.parsePacket())
    {
      UDP.read(&RobotState, 1);
    }

    // Расшифровка ответа от робота
    switch (RobotState)
    {
    case 'W':
    {
      PS5.setLed(ColorsEnum::Green);
      break;
    }
    case 'S':
    {
      PS5.setLed(ColorsEnum::Blue);
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
