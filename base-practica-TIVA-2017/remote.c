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

static uint8_t frame[MAX_FRAME_SIZE];	//Usar una global permite ahorrar pila en la tarea, pero hay que tener cuidado!!!!
static uint32_t gRemoteProtocolErrors=0;

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

	numdatos=create_frame(frame,COMANDO_PING,0,0,MAX_FRAME_SIZE);
	if (numdatos>=0)
	{
		send_frame(frame,numdatos);
	}

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

//Funcion que se ejecuta cuando llega el comando que configura el BRILLO
int32_t ComandoBrilloLedsFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_BRILLO parametro;


	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
	{


		RGBIntensitySet(parametro.rIntensity);

		return 0;	//Devuelve Ok (valor mayor no negativo)
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
		ComandoNoImplementadoFun
};

// Codigo para procesar los comandos recibidos a traves del canal USB del micro ("conector lateral")

//Esta tarea decodifica los comandos y ejecuta la función que corresponda a cada uno de ellos (por posicion)
//También gestiona posibles errores en la comunicacion
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
						//La funcion puede devolver códigos de error.
					    //Se procesarían a continuación
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
							numdatos=create_frame(frame,COMANDO_RECHAZADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
							if (numdatos>=0)
							{
									send_frame(frame,numdatos);
							}
						}
						break;
						//AÑadir casos de error aqui...
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


//Inicializa la tarea que recibe comandos (se debe llamar desde main())
void RemoteInit(void)
{
	//
	// Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
	//
	if(xTaskCreate(CommandProcessingTask, (portCHAR *)"usbser",REMOTE_TASK_STACK, NULL, REMOTE_TASK_PRIORITY, NULL) != pdTRUE)
	{
		while(1);
	}

}
