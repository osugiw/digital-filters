#define LPF_ALPHA_COEF	      0.98f // Alpha coefficient
#define LPF_TIME_SAMPLING     20    // In milliseconds

typedef struct {
	float alpha;	// Smoothing factor
	float y_value[2];		// Output of IIR/RC Filter
} RCFilter;

uint8_t tx_buffer[5];
uint32_t rawAdcValue, filteredAdcValue;
uint32_t lastTime = 0;
RCFilter ADC_LPF;

/**
 * @brief Initialize RC Filter (First Order) or Infinite Impulse Response
 * @param:
 * 		filter			A struct for storing the computed alpha and filtered value
 * 		alpha			Smoothing factor that affects the cut-off frequency between (0-1)
 * @retval None
**/
void LPF_init(RCFilter *filter, float alpha){
	filter->alpha = alpha;

	// Clear output buffer
	filter->y_value[0] = 0.0f;
	filter->y_value[1] = 0.0f;
}

/**
 * @brief Apply filter to the given input value
 * @param:
 * 		filter			A struct for storing the computed alpha and filtered value
 * 		x_value			Input data to filter
 * @retval Filtered value
**/
uint16_t LPF_filterData(RCFilter *filter, uint32_t x_value){
	filter->y_value[1] = filter->y_value[0];	// Shift output samples
	filter->y_value[0] = ((1 - filter->alpha) * x_value) + (filter->alpha * filter->y_value[1]);
	return (uint16_t)filter->y_value[0];
}

void setup() {
  Serial.begin(9600);
  LPF_init(&ADC_LPF, LPF_ALPHA_COEF);
}

void loop() {
  if(millis() - lastTime > LPF_TIME_SAMPLING){
    lastTime = millis();
    rawAdcValue = map(analogRead(A0), 0, 1023, 0, 4095);
    filteredAdcValue = LPF_filterData(&ADC_LPF, rawAdcValue);
    Serial.print(rawAdcValue);
    Serial.print(",");
    Serial.println(filteredAdcValue);
  }
}
