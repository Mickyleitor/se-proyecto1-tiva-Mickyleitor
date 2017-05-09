//*****************************************************************************
//
// Codigo de partida Practica 1.
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//*****************************************************************************

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"

#include "drivers/rgb.h"
#include "usb_dev_serial.h"
#include "protocol.h"
#include "remote.h"

#include"configADC.h"

#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128
#define TAREA1TASKPRIO 1
#define TAREA1STACKSIZE 128
#define ADC_COUNTER 8

//Globales

uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
TaskHandle_t handle = NULL;
xQueueHandle QUEUE_GPIO;

// Vamos a crear este semaforo para controlar y prevenir colisiones de envio de tramas
// ¿Porque? Porque hay varias tareas que podrían enviar de forma simultánea
// ciertos comandos y mientras se crea la trama a enviar, no se pueden interrumpir.

// En esta version no he podido, me da error de linkado.
// TODO en el futuro!!!!!!!
// xSemaphoreHandle UART_SEMAFORO;

extern void vUARTTask( void *pvParameters );
static uint8_t frame[MAX_FRAME_SIZE];

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static unsigned char count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

// El codigo de esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos del interprete a traves
// del terminal serie (puTTY)
//Aqui solo la declaramos para poderla referenciar en la funcion main
extern void vUARTTask( void *pvParameters );



// Codigo de tarea de ejemplo: eliminar para la aplicacion final
static portTASK_FUNCTION(ProcessTask,pvParameters)
{
    uint8_t Received;
    int32_t number_bytes;

    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        if (xQueueReceive(QUEUE_GPIO,&Received, portMAX_DELAY)==pdTRUE){

            PARAM_COMANDO_REQUEST parametro;
            if(Received & GPIO_PIN_4){
                parametro.sw1=0;
            }else parametro.sw1=1;
            if(Received & GPIO_PIN_0){
                parametro.sw2=0;
            }else parametro.sw2=1;

            // xSemaphoreTake(UART_SEMAFORO,portMAX_DELAY);
            number_bytes=create_frame(frame,COMANDO_REQUEST,&parametro,sizeof(parametro),MAX_FRAME_SIZE);

            if (number_bytes>=0)
            {
                send_frame(frame,number_bytes);
            }
            // xSemaphoreGive(UART_SEMAFORO);
        }
    }
}

// SEMANA2: Tarea que envia los datos del MICRO al PC (podria hacer algo mas)
static portTASK_FUNCTION(ADCTask,pvParameters)
{

    MuestrasADC muestras;
    int contador=0;
    MuestrasADC array[ADC_COUNTER];
    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {

        configADC_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)
        array[contador]=muestras;
        if(contador==(ADC_COUNTER-1)){ // Si hemos tomado todas las muestras las enviamos.
            RemoteSendCommand(COMANDO_ADC,(void *)array,sizeof(array));
            contador=0;
        }else contador++;
        //Aprovechamos que las estructuras MuestrasADC y PARAM_COMANDO_ADC son iguales para mandarlas directamente;
    }
}

//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


	// Get the system clock speed.
	g_ulSystemClock = SysCtlClockGet();


	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	ROM_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);

	//
	// Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
	//se usa para mandar y recibir mensajes y comandos por el puerto serie
	// Mediante un programa terminal como gtkterm, putty, cutecom, etc...
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0,115200,SysCtlClockGet());

	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);	//La UART tiene que seguir funcionando aunque el micro este dormido
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);	//La UART tiene que seguir funcionando aunque el micro este dormido


	//Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1 (eliminar si no se usa finalmente)
	RGBInit(1);
	SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);	//Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo

	// Configuramos pulsadores
    //Necesario para poder utilizar los pulsadores
    ButtonsInit();
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    //Habilitamos interrupciones
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    IntEnable(INT_GPIOF);
    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0,GPIO_BOTH_EDGES);

    // Configuramos Timer2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
    // Configura el Timer2 para cuenta periodica de 32 bits (no lo separa en TIMER2A y TIMER2B)
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    // Carga timer (0.1 segundos)
    TimerLoadSet(TIMER2_BASE, TIMER_A, (SysCtlClockGet()/10));

    configADC_IniciaADC();  //SEMANA 2: Inicia el ADC

    /*
    // Creación semoforo MUTEX
    UART_SEMAFORO=xSemaphoreCreateMutex();
    if(UART_SEMAFORO == NULL) while(1);
    */

	//
	// Mensaje de bienvenida inicial.
	//
	UARTprintf("\n\nBienvenido aL sistema de instrumentación y control con conexión USB. (curso 2016/17)!\n");
	UARTprintf("\nAutores: Michele La Malva Moreno ");

	/**                                              Creacion de tareas 												**/

	// Crea la tarea que gestiona los comandos UART (definida en el fichero commands.c)
	//
	if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 256,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

    if((xTaskCreate(ProcessTask,(portCHAR *)"Task REQUEST",TAREA1STACKSIZE,NULL,tskIDLE_PRIORITY +1,&handle)!= pdTRUE))
    {
        while(1);
    }
    //Semana 2
    if((xTaskCreate(ADCTask, (portCHAR *)"ADC", 256,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
    {
            while(1);
    }


    UsbSerialInit(32,32);   //Inicializo el  sistema USB
    RemoteInit(); //Inicializo la aplicacion de comunicacion con el PC (Remote)

    uint8_t x=5; // Tamaño de cola
    QUEUE_GPIO = xQueueCreate(x,sizeof(x));
    if (NULL == QUEUE_GPIO) while(1);   //Si hay problemas al crear la cola, se queda aqui!!
	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

void ButtonsISR(void){
    uint8_t ui8Buttons, ui8Changed;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    uint8_t status = ButtonsPoll(&ui8Changed,&ui8Buttons);
    xQueueSendFromISR(QUEUE_GPIO,&ui8Buttons,&xHigherPriorityTaskWoken);
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
