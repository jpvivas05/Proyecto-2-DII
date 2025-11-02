# Proyecto-2-DII
Proyecto 2 Electrónica Digital II. Sistema de comunicación mediante módulo I2C, SPI y UART.
Al utilizar el programa, la núcleo STM será la encargada de tratar con el usuario, solicitando mediante un menú con el módulo UART el tipo de comunicación que se desee utilizar (I2C o SPI). En el ESP32, con el módulo SPI, se solicitará al usuario escoger una led y un tiempo en ms a encender. Con el módulo I2C, se obtendrá el valor de un potenciómetro. Estos valores serán indicados por medio del monitor serial y una pantalla LCD.
Materiales requeridos:
- Led RGB ánodo común
- Pantalla LCD (sin módulo)
- 2 potenciómetros
- 4 resistencias 330 Ohms
- Jumpers
- ESP32
- Núcleo STM

En los documentos, será necesario activar los módulos de interrupción UART, configuración I2C y SPI en el software CubeIDE para el núcleo. Así como añadir las librerías de LCD, arduino y SPI en el software VSC para el ESP32.
