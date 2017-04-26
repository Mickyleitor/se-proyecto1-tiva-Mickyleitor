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

#define LED1TASKPRIO 1
#define LED2TASKPRIO 1
#define LED1TASKSTACKSIZE 128
#define LED2TASKSTACKSIZE 128


//Globales

uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
xQueueHandle QUEUE_GPIO;
TaskHandle_t handle = NULL;
extern void vUARTTask( void *pvParameters );


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

static portTASK_FUNCTION(LEDTask,pvParameters)
{

    int32_t i32Estado_led=0;

    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        i32Estado_led=!i32Estado_led;

        if (i32Estado_led)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 , GPIO_PIN_1);
            vTaskDelay(0.1*configTICK_RATE_HZ);        //Espera del RTOS (eficiente, no gasta CPU)
                                                     //Esta espera es de unos 100ms aproximadamente.
        }
        else
        {
            GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1,0);
            vTaskDelay(2*configTICK_RATE_HZ);        //Espera del RTOS (eficiente, no gasta CPU)
                                                   //Esta espera es de unos 2s aproximadamente.
        }
    }
}

// MLMM: Tarea que interpreta primer elemento de la FIFO y realiza tantas
// Iteraciones como se hayan pasado por parámetros en funcion de los
// Segundos y la frecuencia. Lo mismo se aplica a la frecuencia.
void LEDTask2(void *pvParameters)
{
    uint32_t Recibido[2];
    uint32_t Contador,SemiPeriodo;
    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        if (xQueueReceive(QUEUE_GPIO,&Recibido,0)==pdTRUE){
                // Suspendemos tarea del LED1TASK (la volveremos a llamar cuando haya un comando que lo active de nuevo)
                vTaskSuspend(handle);
                // Numero de veces que se enciende un LED sera
                // Segundos * Frecuencia
                Contador = Recibido[0] * Recibido[1];
                // Tiempo de Semiperiodo = 1 / (f(hz) * 2 )
                // NOTA: Se multiplica por 1000 y despues por 0.001 porque siendo uint32_t
                // NO se pueden almacenar decimales y no se podría imprimir los milisegundos.
                SemiPeriodo = 1000/(Recibido[1]*2);
                UARTprintf("Activada alarma de %d segundos a %d Hz\n",Recibido[0],Recibido[1]);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 , 0);

                while(Contador>0){
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 , GPIO_PIN_3);
                    vTaskDelay(SemiPeriodo*0.001*configTICK_RATE_HZ);

                    GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3,0);
                    vTaskDelay(SemiPeriodo*0.001*configTICK_RATE_HZ);
                    Contador--;
                }
                // Aqui se supone que se acaban de terminar los segundos
                // Y se activa la alarma
                UARTprintf("ALARMA\n");
                // Aqui volvemos a desactivar tarea, ya que las especificaciones dice que una vez
                // La alarma está activada el led rojo permanecerá encendido (y hemos podido "resumir"
                // la tarea de parpadeo gpio rojo mientras estaba en cuenta atras la alarma)
                vTaskSuspend(handle);
                // Encedemos LED rojo por la fuerza
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 , GPIO_PIN_1);
        }
        // Esto es necesario ya que si no, el uP está todo el rato esperando (bucle infinito)
        // A un caracter, y como esta espera es eficiente, pues resuelve nuestro problema.
        vTaskDelay(configTICK_RATE_HZ); /// Esto equivale a un segundo.
    }
}

// El codigo de esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos del interprete a traves
// del terminal serie (puTTY)
//Aqui solo la declaramos para poderla referenciar en la funcion main
extern void vUARTTask( void *pvParameters );


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

	//
	// Mensaje de bienvenida inicial.
	//
	UARTprintf("\n\nBienvenido a la aplicacion XXXXX (curso 2016/16)!\n");
	UARTprintf("\nAutores: XXXXX y XXXXX ");

	/**                                              Creacion de tareas 												**/


    //
    // Crea la tarea que parpadea el LED ROJO.
    //

    if ((xTaskCreate(LEDTask2, (signed portCHAR *)"Led2",LED2TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + 1,NULL)!= pdTRUE))
        {
            while(1)
            {
            }
        }
    //
    // Crea la tarea que parpadea el LED ROJO.
    //

    if((xTaskCreate(LEDTask, (signed portCHAR *)"Led1", LED1TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + 1, &handle) != pdTRUE))
        {
            while(1)
            {
            }
        }

    //
    // Create la tarea que gestiona los comandos (definida en el fichero commands.c)
    //
    if((xTaskCreate(vUARTTask, (signed portCHAR *)"Uart", 256,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
        {
            while(1)
            {
            }
        }

    uint32_t auxiliar[2];
    QUEUE_GPIO = xQueueCreate(10,sizeof(auxiliar));
     if (NULL == QUEUE_GPIO)
        while(1);   // Si la cola no es creada, se queda "pillado" en este bucle.

    //
    // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
    //
    vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

    //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
    while(1)
    {
        //Si llego aqui es que algo raro ha pasado
    }
}

