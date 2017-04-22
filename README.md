## PROYECTO 1 - GISE-Proyecto-CCS
El principal objetivo del proyecto es el desarrollo de una aplicación de control de salidas y entradas digitales, y adquisición (muestreo, almacenamiento y representación) de señales analógicas y digitales, controlado desde un PC por interfaz USB. El sistema se basará en la placa de bajo coste EK‐TM4C123GXL. 

El sistema implementado tendrá la función de un dispositivo de medida y control (monitorización de entradas analógicas y digitales, datalogger, tarjeta de adquisición de datos y salidas para control de actuadores), así como algunas funcionalidades adicionales.

<p align="center">
    <img src="https://i.gyazo.com/67de20daa1245bf042484976c04ad8b8.png" width="640">
</p>

### Especificaciones funcionales básicas
Se exponen aqui las funcionalidades básicas del proyecto.
#### Control PWM del led tricolor
Desde la aplicación se debe poder controlar el ciclo de trabajo de las señales PWM conectadas a los pines PF1, PF2 y PF3 
(la de los LEDS verde, rojo y azul de la placa TIVA) de forma individual, de manera que sea posible establecer el color RGB del LED tricolor desde el interfaz de usuario (utilizando un widget que permita seleccionar el color deseado). A la vez debe haber un control de brillo general que afecte a todos los LEDs.
#### Cambio de modo
Desde la aplicación se debe poder controlar el modo de los pines de salida que están conectados los LEDs entre modo salida PWM y modo salida GPIO (similar al ejercicio FreeRTOS). Además, desde la aplicación se debe poder controlar el valor de los tres pines GPIO de salida (alto o bajo) mediante los “checkboxes” que tenía la aplicación de partida.
#### Control de entradas digitales.
La aplicación del PC debe poder consultar el valor que presentan las entradas digitales PF0 y PF4, que corresponden a los pulsadores SW1 y SW2 de la placa TIVA. Para ello se añadirá al interfaz gráfico QT, un botón que permita leer el estado de dichos pines (por sondeo) y mostrarlo en el interfaz de usuario (para mostrar el valor utilice un control de tipo LED (de la biblioteca analogwidgets) para cada pulsador.
#### Envío asíncrono de eventos de los pulsadores.
Desde el PC se debe poder activar o desactivar este comportamiento, que consiste en que el microcontrolador avise al PC cuando hay un cambio en el estado de los pulsadores SW1 o SW2 de la TIVA, sin que el PC tenga que consultarle. Al recibir los mensajes de notificación, el PC debe mostrar el valor de los pines en el interfaz de usuario. El envío de este evento se debe poder activar y desactivar desde el PC. 
#### Visualización en tiempo real de:
1. Visualizar las tareas del sistema y su estado [Implementado en el ejemplo]
2. Visualizar la memoria libre de FreeRTOS [Implementado en el ejemplo]
3. Visualizar el uso de CPU [Implementado en el ejemplo]
4. Visualizar las estadísticas de las tareas [Implementado en el ejemplo]

### Protocolo de comunicación
Para las comunicaciones entre la aplicación del PC y la del microcontrolador se utilizará el perfil CDC (Communication Device Class) del interfaz USB, que emula un puerto serie.
<p align="center">
    <img src="https://i.gyazo.com/db8f47a92f01cdd8c4d51a73551017ab.png" width="480">
</p>
Cada trama va precedida por un byte especial de comienzo de trama (con valor 0xFC) y finaliza con un byte de fin de trama (valor 0xFD). Entre el carácter de inicio de trama y el carácter de fin de trama podrá ir un número variable de octetos (con unos límites), que se organizan en campos con longitud de uno o más bytes. Estos campos son un byte que indica el tipo de comando, un campo de datos cuya longitud depende del tipo de comando y un campo de dos bytes con un CRC para control de errores. 

Cada trama enviada por el puerto serie se detecta gracias a los octetos especiales de inicio y de fin, lo que obliga a que ningún otro byte pueda tener los valores 0xCF o 0xDF, ya que de lo contrario se detectaría de forma incorrecta un inicio o un fin de trama. Para ello, a los octetos intermedios, antes de ser transmitidos, se les aplica lo que se conoce como Byte Stuffing.

#### Lista de comandos BÁSICOS
El envío de los parámetros se hará en formato Little‐Endian, es decir, primero el byte menos significativo y luego el más. Nótese que este orden es el mismo en el que se almacenan los datos tanto en la memoria del PC como del TM4C123GH6PM.

<p align="center">
    <img src="https://i.gyazo.com/25eeb3c4f0b3b910a76783c283929a4b.png" width="640">
</p>

El microcontrolador puede también enviar mensajes de respuesta hacia el PC.

<p align="center">
    <img src="https://i.gyazo.com/d7611bdf3e145ec295af5c7d53cf25d8.png" width="640">
</p>
