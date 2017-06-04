#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


 //SEMANA2: Este fichero implementa la configuracion del ADC y la ISR asociada. Las tareas pueden llamar a la funcion configADC_LeeADC (bloqueante) para leer datos del ADC
//La funcion configADC_DisparaADC(...) (no bloqueante) realiza el disparo software del ADC
//La funcion configADC_IniciaADC realiza la configuraci�n del ADC: Los Pines E0 a E3 se ponen como entradas anal�gicas (AIN3 a AIN0 respectivamente). Ademas crea la cola de mensajes necesaria para que la funcion configADC_LeeADC sea bloqueante


static QueueHandle_t cola_adc;



//Provoca el disparo de una conversion (hemos configurado el ADC con "disparo software" (Processor trigger)
void configADC_DisparaADC(void)
{
	ADCProcessorTrigger(ADC0_BASE,1);
}


void configADC_IniciaADC(void)
{
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
			    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

				//HABILITAMOS EL GPIOE
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
				// Enable pin PE3 for ADC AIN0|AIN1|AIN2|AIN3
				GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);


				//CONFIGURAR SECUENCIADOR 1
				ADCSequenceDisable(ADC0_BASE,1);

                //Configuramos la velocidad de conversion al maximo (1MS/s)
                //1 muestra por segundo y podemos dividir la velocidad en el ultimo parametro
                ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

                //Configuracion de secuencia
                //TRIGGER PROCESSOR disparar por proceso y si ponemos TRIGGER Timer disparamos por timer (NOS HARA FALTA)
                TimerControlTrigger(TIMER2_BASE,TIMER_A,true);

                ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_PROCESSOR,0); // En este momento disparo por software (en tiempo real se cambia esto para poder disparar por timer)

				ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH0);
				ADCSequenceStepConfigure(ADC0_BASE,1,1,ADC_CTL_CH1);
				ADCSequenceStepConfigure(ADC0_BASE,1,2,ADC_CTL_CH2);
				ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH3|ADC_CTL_IE |ADC_CTL_END );	//La ultima muestra provoca la interrupcion
				ADCSequenceEnable(ADC0_BASE,1); //ACTIVO LA SECUENCIA

				//Habilita las interrupciones
				ADCIntClear(ADC0_BASE,1);
				ADCIntEnable(ADC0_BASE,1);
				IntPrioritySet(INT_ADC0SS1,configMAX_SYSCALL_INTERRUPT_PRIORITY);
				IntEnable(INT_ADC0SS1);

				//Creamos una cola de mensajes para la comunicacion entre la ISR y la tara que llame a configADC_LeeADC(...)
				cola_adc=xQueueCreate(8,sizeof(MuestrasADC));
				if (cola_adc==NULL)
				{
					while(1);
				}
}


void configADC_LeeADC(MuestrasADC *datos)
{
	xQueueReceive(cola_adc,datos,portMAX_DELAY);
}

void configADC_ISR(void)
{
	portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

	MuestrasLeidasADC leidas;
	MuestrasADC finales;
	ADCIntClear(ADC0_BASE,1);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
	ADCSequenceDataGet(ADC0_BASE,1,(uint32_t *)&leidas);//COGEMOS LOS DATOS GUARDADOS

	//Pasamos de 32 bits a 16 (el conversor es de 12 bits, as� que s�lo son significativos los bits del 0 al 11)
	finales.chan1=leidas.chan1;
	finales.chan2=leidas.chan2;
	finales.chan3=leidas.chan3;
	finales.chan4=leidas.chan4;

	//Guardamos en la cola
	xQueueSendFromISR(cola_adc,&finales,&higherPriorityTaskWoken);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
