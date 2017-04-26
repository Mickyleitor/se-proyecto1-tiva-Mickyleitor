//*****************************************************************************
//
// commands.c - FreeRTOS porting example on CCS4
//
// Este fichero contiene errores que seran explicados en clase
//
//*****************************************************************************


#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard TIVA includes */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

/* Other TIVA includes */
#include "utils/cpu_usage.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"

#include "drivers/rgb.h"

// ==============================================================================
// The CPU usage in percent, in 16.16 fixed point format.
// ==============================================================================
extern uint32_t g_ui32CPUUsage;


// MLMM: Hacemos referencia a la cola declarada en el main
extern xQueueHandle QUEUE_GPIO;

// MLMM: Tarea led rojo parpadeando
extern TaskHandle_t handle;

// ==============================================================================
// Implementa el comando cpu que muestra el uso de CPU
// ==============================================================================
int  Cmd_cpu(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("ARM Cortex-M4F %u MHz - ",SysCtlClockGet() / 1000000);
    UARTprintf("%2u%% de uso\n", (g_ui32CPUUsage+32768) >> 16);

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando free, que muestra cuanta memoria "heap" le queda al FreeRTOS
// ==============================================================================
int Cmd_free(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("%d bytes libres\n", xPortGetFreeHeapSize());

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando task. S�lo es posible si la opci�n configUSE_TRACE_FACILITY de FreeRTOS est� habilitada
// ==============================================================================
#if ( configUSE_TRACE_FACILITY == 1 )

extern char *__stack;
int Cmd_tasks(int argc, char *argv[])
{
	char*	pcBuffer;
	uint8_t*	pi8Stack;
	portBASE_TYPE	x;
	
	pcBuffer = pvPortMalloc(1024);
	vTaskList(pcBuffer);
	UARTprintf("\t\t\t\tUnused\nTaskName\tStatus\tPri\tStack\tTask ID\n");
	UARTprintf("=======================================================\n");
	UARTprintf("%s", pcBuffer);
	
	// Calculate kernel stack usage
	x = 0;
	pi8Stack = (uint8_t *) &__stack;
	while (*pi8Stack++ == 0xA5)
	{
		x++;	//Esto s�lo funciona si hemos rellenado la pila del sistema con 0xA5 en el arranque
	}
	sprintf((char *) pcBuffer, "%%%us", configMAX_TASK_NAME_LEN);
	sprintf((char *) &pcBuffer[10], (const char *) pcBuffer, "kernel");
	UARTprintf("%s\t-\t*%u\t%u\t-\n", &pcBuffer[10], configKERNEL_INTERRUPT_PRIORITY, x/sizeof(portBASE_TYPE));
	vPortFree(pcBuffer);
	return 0;
}

#endif /* configUSE_TRACE_FACILITY */

#if configGENERATE_RUN_TIME_STATS
// ==============================================================================
// Implementa el comando "Stats"
// ==============================================================================
Cmd_stats(int argc, char *argv[])
{
	char*	pBuffer;

	pBuffer = pvPortMalloc(1024);
	if (pBuffer)
	{
		vTaskGetRunTimeStats(pBuffer); //Es un poco inseguro, pero por ahora nos vale...
		UARTprintf("TaskName\tCycles\t\tPercent\r\n");
		UARTprintf("===============================================\r\n");
		UARTprintf("%s", pBuffer);
		vPortFree(pBuffer);
	}
	return 0;
}
#endif

// ==============================================================================
// Implementa el comando help
// ==============================================================================
int Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    UARTprintf("Comandos disponibles\n");
    UARTprintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}

// ==============================================================================
// Implementa el comando "LED"
// ==============================================================================
// ==============================================================================
int Cmd_led(int argc, char *argv[])
{
    uint32_t x;
    if (argc != 3)
    {
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" led [on|off] [color]\n");
    }
    else
    {
        // MLMM: Tenia una version mucho mas compacta para esto, pero un dia
        // antes de entregar el ejercicio se me perdió el proyecto :(
        // Pero funciona igualmente!!

        if ( 0 == (strncmp(argv[2],"rojo",4))) x= GPIO_PIN_1;
        if ( 0 == (strncmp(argv[2],"verde",5))) x= GPIO_PIN_3;
        if ( 0 == (strncmp(argv[2],"azul",4))) x= GPIO_PIN_2;

        if (0==strncmp( argv[1], "on",2))
        {

            UARTprintf("Enciendo el LED %s\n",argv[2]);
            GPIOPinWrite(GPIO_PORTF_BASE, x,x);
        }
        else if (0==strncmp( argv[1], "off",3))
        {
            UARTprintf("Apago el LED %s\n",argv[2]);
            GPIOPinWrite(GPIO_PORTF_BASE, x,0);
        }
        else
        {
            //Si el parametro no es correcto, muestro la ayuda
            UARTprintf(" led [on|off] [color]\n");
        }

    }
    return 0;
}


// ==============================================================================
// Implementa el comando "MODE"
// ==============================================================================
int Cmd_mode(int argc, char *argv[])
{
    if (argc != 2)
    {
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" mode [gpio|pwm]\n");
    }
    else
    {
        if (0==strncmp( argv[1], "gpio",4))
        {
            UARTprintf("cambio a modo GPIO\n");
            RGBDisable();
            // MLMM: Aqui volvemos a resumir la tarea del LED1, para que parpadee por defecto.
            vTaskResume(handle);
            ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
        }
        else if (0==strncmp( argv[1], "pwm",3))
        {
            UARTprintf("Cambio a modo PWM (rgb)\n");
            RGBEnable();
        }
        else
        {
            //Si el parametro no es correcto, muestro la ayuda
            UARTprintf(" mode [gpio|pwm]\n");
        }
    }


    return 0;
}

// ==============================================================================
// Implementa el comando "RGB"
// MLMM: Modificas las conversiones correctas del array
// ==============================================================================
int Cmd_rgb(int argc, char *argv[])
{
    uint32_t arrayRGB[3];

    if (argc != 4)
    {
        //Si los par�metros no son suficientes, muestro la ayuda
        UARTprintf(" rgb [red] [green] [blue]\n");
    }
    else
    {
        arrayRGB[0]=strtoul(argv[1], NULL, 10) <<8;
        arrayRGB[1]=strtoul(argv[2], NULL, 10) <<8;
        arrayRGB[2]=strtoul(argv[3], NULL, 10) <<8;

        if ((arrayRGB[0]>=65535)||(arrayRGB[1]>=65535)||(arrayRGB[2]>=65535))
        {

            UARTprintf(" \n");
        }
        else{
            RGBColorSet(arrayRGB);
        }

    }


    return 0;
}
// MLMM: Comando intensity que cambia la intensidad global
// dependiendo del valor de entrada normalizado
// Utilizando la funcion RGBIntensity
int Cmd_intensity(int argc, char *argv[]){
    if (argc < 2){
        UARTprintf("Intensity [valor]");
    }
    else{
        RGBIntensitySet(strtof(argv[1],NULL));
    }
    return 0;
}

// MLMM: Creamos funcion de la alarma
// Que interpreta los comando de entrada y los envia a la cola de mensajes.
int Cmd_alarma(int argc, char *argv[]){
    uint32_t auxiliar[2]={0,0};
    if (argc != 3){
        UARTprintf("alarma [segundos] [frecuencia]\n");
    }
    else{
        UARTprintf("Alarma programada para %s segundos a %s Herzios\n",argv[1],argv[2]);
        auxiliar[0]=strtoul(argv[1],NULL,10);
        auxiliar[1]=strtoul(argv[2],NULL,10);

        //Enviamos datos por la cola de mensajes
        xQueueSend(QUEUE_GPIO,&auxiliar,0);

    }
    return 0;
}




// ==============================================================================
// Tabla con los comandos y su descripcion. Si quiero anadir alguno, debo hacerlo aqui
// ==============================================================================
tCmdLineEntry g_psCmdTable[] =
{
    { "help",     Cmd_help,      "       : Lista de comandos" },
    { "?",        Cmd_help,      "          : lo mismo que help" },
    { "cpu",      Cmd_cpu,       "        : Muestra el uso de  CPU " },
    { "led",      Cmd_led,       "        : Apaga y enciende algun led" },
    { "mode",     Cmd_mode,      "       : Cambia los pines PF1, PF2 y PF3 entre modo GPIO y modo PWM (rgb)" },
    { "rgb",      Cmd_rgb,       "        : Establece el color RGB" },
    { "intensity",     Cmd_intensity,      "  : Cambia la intensidad global del RGB" },
    { "alarma",     Cmd_alarma,      "     : Crea una cuenta atras para la activacion de alarma" },
    { "free",     Cmd_free,      "       : Muestra la memoria libre" },
#if ( configUSE_TRACE_FACILITY == 1 )
    { "tasks",    Cmd_tasks,     "      : Muestra informacion de las tareas" },
#endif
#if (configGENERATE_RUN_TIME_STATS)
    { "stats",    Cmd_stats,     "      : Muestra estadisticas de las tareas" },
#endif
    { 0, 0, 0 }
};

// ==============================================================================
// Tarea UARTTask.  Espera la llegada de comandos por el puerto serie y los ejecuta al recibirlos...
// ==============================================================================
//extern xSemaphoreHandle g_xRxLineSemaphore;
void UARTStdioIntHandler(void);

void vUARTTask( void *pvParameters )
{
    char    pcCmdBuf[64];
    int32_t i32Status;
	
    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\n\nWelcome to the TIVA FreeRTOS Demo!\n");
	UARTprintf("\n\n FreeRTOS %s \n",
		tskKERNEL_VERSION_NUMBER);
	UARTprintf("\n Teclee ? para ver la ayuda \n");
	UARTprintf("> ");    

	/* Loop forever */
	while (1)
	{

		/* Read data from the UART and process the command line */
		UARTgets(pcCmdBuf, sizeof(pcCmdBuf));
		if (strlen(pcCmdBuf) == 0)
		{
			UARTprintf("> ");
			continue;
		}

		//
		// Pass the line from the user to the command processor.  It will be
		// parsed and valid commands executed.
		//
		i32Status = CmdLineProcess(pcCmdBuf);

		//
		// Handle the case of bad command.
		//
		if(i32Status == CMDLINE_BAD_CMD)
		{
			UARTprintf("Comando erroneo!\n");	//No pongo acentos adrede
		}

		//
		// Handle the case of too many arguments.
		//
		else if(i32Status == CMDLINE_TOO_MANY_ARGS)
		{
			UARTprintf("El interprete de comandos no admite tantos parametros\n");	//El maximo, CMDLINE_MAX_ARGS, esta definido en cmdline.c
		}

		//
		// Otherwise the command was executed.  Print the error code if one was
		// returned.
		//
		else if(i32Status != 0)
		{
			UARTprintf("El comando devolvio el error %d\n",i32Status);
		}

		UARTprintf("> ");

	}
}
