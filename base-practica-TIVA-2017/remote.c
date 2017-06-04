/*
 * remote.c
 *
 *  Created on: 1/4/2016
 *      Author: jcgar
 */

#include"remote.h"

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
#include "drivers/rgb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "configADC.h"

static uint8_t frame[MAX_FRAME_SIZE];	//Usar una global permite ahorrar pila en la tarea, pero hay que tener cuidado!!!!
static uint32_t gRemoteProtocolErrors=0;

// Manejador para tarea del puerto
extern TaskHandle_t handle;
// Semaforo para el UART
extern xSemaphoreHandle UART_SEMAFORO;

//Defino a un tipo que es un puntero a funcion con el prototipo que tienen que tener las funciones que definamos
typedef int32_t (*remote_fun)(uint32_t param_size, void *param);


//Funcion que se ejecuta cuando llega un paquete indicando comando rechazado
int32_t ComandoRechazadoFun(uint32_t param_size, void *param)
{
	//He recibido "Comando rechazado" desde el PC
	//TODO, por hacer: Tratar dicho error??
	gRemoteProtocolErrors++;
	return 0;
}


//Funcion que se ejecuta cuando llega un PING
int32_t ComandoPingFun(uint32_t param_size, void *param)
{
	int32_t numdatos;

	xSemaphoreTake(UART_SEMAFORO,portMAX_DELAY);
	numdatos=create_frame(frame,COMANDO_PING,0,0,MAX_FRAME_SIZE);
	if (numdatos>=0)
	{
		send_frame(frame,numdatos);
	}
	xSemaphoreGive(UART_SEMAFORO);

	return numdatos;
}


//Funcion que se ejecuta cuando llega el comando que configura los LEDS
int32_t ComandoLedsFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_LEDS parametro;
	uint32_t ulColors[3] = {0x0000, 0x0000, 0x0000 };

	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
	{
		//Ahora mismo se hace usando el PWM --> TODO: Cambiar a GPIO para cumplir las especificaciones
		ulColors[0]= parametro.leds.red ? 0x8000 : 0x0000;
		ulColors[1]=parametro.leds.green ? 0x8000 : 0x0000;
		ulColors[2]= parametro.leds.blue ? 0x8000 : 0x0000;

		RGBColorSet(ulColors);

		return 0;	//Devuelve Ok (valor mayor no negativo)
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

//Funcion que se ejecuta cuando recibimos un comando que no tenemos aun implementado
int32_t ComandoNoImplementadoFun(uint32_t param_size, void *param)
{
	return PROT_ERROR_UNIMPLEMENTED_COMMAND; /* Devuelve un error para que lo procese la tarea que recibe los comandos */
}

//Funcion que se ejecuta cuando llega el comando que configura los LEDS
int32_t ComandoBrilloLedsFun(uint32_t param_size, void *param)
{
    PARAM_COMANDO_BRILLO parametro;


    if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
    {


        RGBIntensitySet(parametro.rIntensity);

        return 0;   //Devuelve Ok (valor mayor no negativo)
    }
    else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
    }
}

int32_t ComandoColorFun(uint32_t param_size,void *param)
{
    PARAM_COMANDO_COLOR parametro;
    uint32_t array[3]={0x0000, 0x0000, 0x0000};
    if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
    {

        array[0]=parametro.r <<8;
        array[1]=parametro.g <<8;
        array[2]=parametro.b <<8;
        RGBColorSet(array);
        return 0;
    }
    else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
    }

}

int32_t ComandoModeFun(uint32_t param_size,void *param){
    // Funcion auxiliar para debug para habilitar (1) o deshabilitar (0) interrupciones.
    PARAM_COMANDO_MODO parametro;
    if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
    {
        if (parametro.x == 0){
            RGBDisable();
            ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
        }else if (parametro.x == 1){
            RGBEnable();
        }
        return 0;
    }
    else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE;
    }
}

int32_t ComandoRequestFun(uint32_t param_size,void *param){
    int32_t numdatos;
    PARAM_COMANDO_REQUEST parametro;
    parametro.sw1 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
    parametro.sw2 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);

    xSemaphoreTake(UART_SEMAFORO,portMAX_DELAY);
    numdatos=create_frame(frame,COMANDO_REQUEST,&parametro,sizeof(parametro),MAX_FRAME_SIZE);

    if (numdatos>=0)
    {
        send_frame(frame,numdatos);
    }
    xSemaphoreGive(UART_SEMAFORO);

    return numdatos;
}

int32_t ComandoInterruptFun(uint32_t param_size,void *param){
    PARAM_COMANDO_INTERRUPTS parametro;
    if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
    {
        if (parametro.x == 0){ //Si es 0 desactivamos las interrupciones
             vTaskSuspend(handle);
             IntDisable(INT_GPIOF);

        }else if (parametro.x == 1){ //Si es 1 activamos las interrupciones
            vTaskResume(handle);
            IntEnable(INT_GPIOF);
        }
        return 0;
    }
    else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE;
    }
}

//SEMANA2: Funcion que procesa el comando ADC
int32_t ComandoADCFun(uint32_t param_size, void *param)
{
    if (param_size!=0) return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error si viene con datos (no los esperamos)

    configADC_DisparaADC(); //Dispara la conversion (por software)

    return 0;
}

int32_t ComandoFreqFun(uint32_t param_size, void *param)
{
    PARAM_COMANDO_FREQ parametro;
    uint32_t ui32Periodo=0;
    double valor =0;
    if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
    {
        //Cargamos nueva cuenta en el timer
        valor = parametro.frequency;
        valor=valor*1000;
        ui32Periodo =((SysCtlClockGet()/valor));
        TimerLoadSet(TIMER2_BASE, TIMER_A,ui32Periodo-1);
        return 0;   //Devuelve Ok (valor mayor no negativo)
    }
    else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
    }
}
int32_t ComandoTimerFun(uint32_t param_size, void *param)
{
    PARAM_COMANDO_TIMER parametro;
    if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
    {
        //Si el check box est� activo, activamos timer, sino lo desactivamos
        if(parametro.Timer_On == true){
            TimerEnable(TIMER2_BASE, TIMER_A);
            ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_TIMER,0);  //Disparo por el timer2
        }else{
            TimerDisable(TIMER2_BASE, TIMER_A);
            ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_PROCESSOR,0); // Disparo por software (sondeo)
        }
        return 0;   //Devuelve Ok (valor mayor no negativo)
    }
    else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
    }
}

/* Array que contiene las funciones que se van a ejecutar en respuesta a cada comando */
static const remote_fun remote_fun_array[]={
		ComandoRechazadoFun, /* Responde al paquete comando rechazado */
		ComandoPingFun, /* Responde al comando ping */
		ComandoLedsFun, /* Responde al comando LEDS */
		ComandoBrilloLedsFun, /* Responde al comando Brillo */
		ComandoModeFun, /* Responde al comando Modo GPIO / PWM */
		ComandoRequestFun, /* Responde al comando para preguntar estado (pull) de sw1,sw2 */
		ComandoColorFun, /* Responde al comando de la rueda de color */
		ComandoInterruptFun, /* Responde al comando de habilitacion o deshabilitacion de interrupciones */
		ComandoADCFun,
		ComandoFreqFun,
		ComandoTimerFun,
		ComandoNoImplementadoFun
};

// Codigo para procesar los comandos recibidos a traves del canal USB del micro ("conector lateral")

//Esta tarea decodifica los comandos y ejecuta la funci�n que corresponda a cada uno de ellos (por posicion)
//Tambi�n gestiona posibles errores en la comunicacion
static portTASK_FUNCTION( CommandProcessingTask, pvParameters ){

	//Frame es global en este fichero, se reutiliza en las funciones que envian respuestas ---> CUIDADO!!!

	int32_t numdatos;
	uint8_t command;
	void *ptrtoparam;

	/* The parameters are not used. (elimina el warning)*/
	( void ) pvParameters;

	for(;;)
	{
		numdatos=receive_frame(frame,MAX_FRAME_SIZE);
		if (numdatos>0)
		{	//Si no hay error, proceso la trama que ha llegado.
			numdatos=destuff_and_check_checksum(frame,numdatos);
			if (numdatos<0)
			{
				//Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
				gRemoteProtocolErrors++;
				// Procesamiento del error (TODO, POR HACER!!)
			}
			else
			{
				//El paquete esta bien, luego procedo a tratarlo.
				command=decode_command_type(frame);
				numdatos=get_command_param_pointer(frame,numdatos,&ptrtoparam);

				if (command<(sizeof(remote_fun_array)/sizeof(remote_fun)))
				{
					switch(remote_fun_array[command](numdatos,ptrtoparam))
					{
						//La funcion puede devolver c�digos de error.
					    //Se procesar�an a continuaci�n
						case PROT_ERROR_NOMEM:
						{
							// Procesamiento del error NO MEMORY (TODO, por hacer)
						}
						break;
						case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
						{
							// Procesamiento del error STUFFED_FRAME_TOO_LONG (TODO, por hacer)
						}
						break;
						case PROT_ERROR_COMMAND_TOO_LONG:
						{
							// Procesamiento del error COMMAND TOO LONG (TODO, por hacer)
						}
						break;
						case PROT_ERROR_INCORRECT_PARAM_SIZE:
						{
							// Procesamiento del error INCORRECT PARAM SIZE (TODO, por hacer)
						}
						break;
						case PROT_ERROR_UNIMPLEMENTED_COMMAND:
						{
							PARAM_COMANDO_RECHAZADO parametro;

							parametro.command=command;
							//El comando esta bien pero no esta implementado
							xSemaphoreTake(UART_SEMAFORO,portMAX_DELAY);
							numdatos=create_frame(frame,COMANDO_RECHAZADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
							if (numdatos>=0)
							{
									send_frame(frame,numdatos);
							}
							xSemaphoreGive(UART_SEMAFORO);
						}
						break;
						//A�adir casos de error aqui...
						default:
							/* No hacer nada */
							break;
					}
				}
				else
				{
						/* El comando no es reconocido por el microcontrolador */
						ComandoNoImplementadoFun(numdatos,ptrtoparam);
						gRemoteProtocolErrors++;
				}
			}
		}
		else
		{ // if (numdatos >0)
				//Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
				gRemoteProtocolErrors++;
				// Procesamiento del error (TODO)
		}
	}
}


//SEMANA2: a�adida para facilitar el envio de datos a diversas tareas. IMPORTANTE!!! Leer los comentarios que hay abajo
//Ojo!! Frame es global (para ahorrar memoria de pila en las tareas) --> Se deben tomar precauciones al usar esta funci�n en varias tareas
//IDEM en lo que respecta al envio por el puerto serie desde varias tareas....
//Estas precauciones no se han tomado en este codigo de partida, pero al realizar la practica se deberian tener en cuenta....
int32_t RemoteSendCommand(uint8_t comando,void *parameter,int32_t paramsize)
{
    int32_t numdatos;
    xSemaphoreTake(UART_SEMAFORO,portMAX_DELAY);
    numdatos=create_frame(frame,comando,parameter,paramsize,MAX_FRAME_SIZE);
    if (numdatos>=0)
    {
        send_frame(frame,numdatos);
    }
    xSemaphoreGive(UART_SEMAFORO);
    return numdatos;
}


//Inicializa la tarea que recibe comandos (se debe llamar desde main())
void RemoteInit(void)
{
    //
    // Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
    //
    // Se podrian crear otras tareas e IPC que hagan falta (por ejemplo Mutex si se consideran necesarios!!!!)

    if(xTaskCreate(CommandProcessingTask, (portCHAR *)"usbser",REMOTE_TASK_STACK, NULL, REMOTE_TASK_PRIORITY, NULL) != pdTRUE)
    {
        while(1);
    }

}
