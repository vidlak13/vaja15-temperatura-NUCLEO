# vaja15-temperatura-NUCLEO
spreminjanje pwm z ntc uporom
---------------------------------------------------
ODGOVORI:
E.) e.) HAL_UART_Transmit(&huart2, (uint8_t*)TxData, sprintf(TxData, &quot;%u&quot;, ADCValue), 10);
i.) analogni vhod ADC pretvornika: pin PC0.
G.) PWM izhod: pin PA8.
H.)   if(T2int >= 45 && Alarm_OFF == 1)
		        {
		        		  Alarm_OFF=0;
		        		  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_RESET);
		        		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  		 
		      }
    
    
I.)            void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

{
	UNUSED(GPIO_Pin);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	for(uint32_t i=0; i<10000; i++);
	Alarm_OFF=1;
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

}
.) led se ugasne.
-------------------------------------------------------------------------
KOMENTAR:
S programom sva imela veliko težav, saj nama kar nekajkrat ni delovalo, nekajkrat nama ni deloval PWM izhod, nekajkrat pa tudi program, ki se ni hotel prenesti na NUCLEO ploščico. Drugače pa se PWM spreminja z višanjem temperature, led dioda se prižge ko presežemo temp 45, user tipka pa jo tudi ugasne ko pridemo pod to temperaturo.
