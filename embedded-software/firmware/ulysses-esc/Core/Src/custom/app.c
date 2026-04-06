#include "app.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include "commutation.h"
#include "adc.h"

//startup defines 
#define VOLTAGE_RATING 2700U
#define SUPPLY_VOLTAGE 20
#define DUTY_CYCLE 0.20 //tied to commutation.c, under PWM_DUTY_CYCLE
#define TARGET_RPM (VOLTAGE_RATING * SUPPLY_VOLTAGE * DUTY_CYCLE)
#define MOTOR_POLES_PAIRS 7
#define STARTUP_RAMP_TIME_MS 10000           // how long initalization speed takes (milliseconds)
#define STARTUP_MIN_FREQ_HZ 100             // min freq we run at 

//blanking defines 
#define BLANKING_TIME_PERCENTAGE 20

//comm step freq defines 
#define STARTUP_MAX_FREQ_HZ   (TARGET_RPM * MOTOR_POLES_PAIRS * 6) / 60  /* Electrical commutation cycles/sec — adjust  */
#define COMM_STEP_ARR   ((16000000U / (COMM_FREQ_HZ)) - 1U) /* TIM15 ARR */
#define BEMF_BUFFER_SIZE 32 /*Size of the circular buffer for back emf averaging*/

#define SYSCLOCK_HZ HAL_RCC_GetHCLKFreq()

typedef enum { 
    STATE_STARTUP,           // Using timer to force commutation 
    STATE_SENSORLESS,        // Using zero-crossing to trigger commutation 
} MotorState; 

typedef struct {
    uint16_t samples[BEMF_BUFFER_SIZE];
    uint8_t write_index;
    uint32_t sum; 
} PhaseBuffer; 

typedef struct { 
    uint32_t commutation_ticks;         // timer value when the commutation happened
    uint8_t active;                     //are we blanking> 
}BlankingWindow;

//create a blankingwindow object, all to 0 
static BlankingWindow blanking = { 0 };

//open loop start up! 
static MotorState motor_state = STATE_STARTUP;
static uint32_t startup_start_time_ms = 0; 

/* ring buffers for filtering 
whne new samples arrive, subtract the oldest at write index
write a new sample, add new to sum, advance write_index
sum is always getting updated, just divide by buffer size for the average */
static PhaseBuffer phase_A = {0};
static PhaseBuffer phase_B = {0};
static PhaseBuffer phase_C = {0};

/* communication states 
current commutation step (0-5) 
each step defineds which two motor phases are active and whcih is floating */
static uint8_t comm_step = 0; 

/** 
 * @brief start up the open loop startup sequence 
 * 
 * record the current time 
 * configure TIM15 to interrupt at startup freq 
 * enable TIM15 inturrupts 
 * set motor state to STARTUP 
 */
void startup_begin( void ) {
    startup_start_time_ms = HAL_GetTick(); 

    //configure the TIM15 for startup minimum freq 
    uint32_t ARR = (SYSCLOCK_HZ / STARTUP_MIN_FREQ_HZ) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim15, ARR); 
    __HAL_TIM_SET_COUNTER(&htim15, 0); 

    HAL_TIM_Base_Start_IT(&htim15);

    motor_state = STATE_STARTUP; 
}

/** 
 * @brief updating the startup freq ramping up from min to max
 * 
 * its linear, so 500ms in means half of the freq 
 */
void startup_update( void ){
    if(motor_state != STATE_STARTUP){
        return; //do nothing if we are not in startup anymore 
    }

    uint32_t elapsed_ms = HAL_GetTick() - startup_start_time_ms; 
    
    if(elapsed_ms >= STARTUP_RAMP_TIME_MS) { 
        motor_state = STATE_SENSORLESS; 
        return; 
    }

    uint32_t freq_range = STARTUP_MAX_FREQ_HZ - STARTUP_MIN_FREQ_HZ; 
    uint32_t current_freq = STARTUP_MIN_FREQ_HZ +
                            (freq_range * elapsed_ms) / STARTUP_RAMP_TIME_MS; 

    // update tim15 freq 
    uint32_t ARR = (SYSCLOCK_HZ / current_freq) - 1; 
    __HAL_TIM_SET_AUTORELOAD(&htim15, ARR);
} 

/** 
 * @brief start blanking window using TIM15 counter 
 */
static void blanking_start( void ){
    blanking.commutation_ticks = __HAL_TIM_GET_COUNTER(&htim15); 
    blanking.active = 1; 
}

/** 
 * @brief check if stil in blanking window 
 */
static uint8_t blanking_is_active( void ){
    if(!blanking.active) {
        return 0; 
    }

    uint32_t current_ticks = __HAL_TIM_GET_COUNTER(&htim15);
    uint32_t elapsed = current_ticks - blanking.commutation_ticks; 

    uint32_t current_arr = __HAL_TIM_GET_AUTORELOAD(&htim15); 
    uint32_t blanking_time = (current_arr * BLANKING_TIME_PERCENTAGE) / 100; 


    if(elapsed > blanking_time){
        blanking.active = 0; 
        return 0; 
    }

    return 1; 
}

//flag set by adc callback when zero crossing is detected. 
//main loop checks this flag and advances commutation 
// volatile because its set in an interrupting context
volatile uint8_t zero_crossing_detected = 0;

/**
 * @brief Updating the ring buffer with new ADC samples
 * 
 * subtracts oldest sample from the running sample 
 * writes new ssample to current index
 * adds new sample to sum 
 * increment write pointer (wrap around at buffer size) 
 * 
 * @param buf: pointer to PhaseBuffer struct 
 * @param new_sample: new 12-bit ADC value (0-4095) tehoretically but no tests yet 
 */
static void update_ring_buffer(PhaseBuffer* buf, uint16_t new_sample){
    buf->sum -= buf->samples[buf->write_index];
    buf->samples[buf->write_index] = new_sample;
    buf->sum += new_sample;
    buf->write_index = (buf->write_index + 1) % BEMF_BUFFER_SIZE; 
}

/**
 * @brief Get averaged back-emfvalue fro mthe ring buffer 
 * 
 * Returns the mean value of the last BEMF_BUFFER_SIZE samples. 
 * Just divides precalculated sum by buffer size
 * 
 * @param buf: pointer to PhaseBuffer struct
 * @return averaged adc value (0- 4095) theoretically but havent tested
 */
static uint16_t get_averaged_bemf(PhaseBuffer* buf){
    return buf->sum / BEMF_BUFFER_SIZE; 
}

//back emf reading time! 
/**
 * @brief getting the back emf voltage from the floating phase.
 * 
 * In each step we have 3 pins, hi low and floating. in each step,
 * we get a different phase. from commutation.c i ge tthat 
 * 
 *   Step 0: HinA/LinC active Phase B floats
 *   Step 1: HinB/LinC active  Phase A floats
 *   Step 2: HinB/LinA active  Phase C floats
 *   Step 3: HinC/LinA active Phase B floats
 *   Step 4: HinC/LinB active  Phase A floats
 *   Step 5: HinA/LinB active Phase C floats
 * 
 * @param step: current commutation step (0-5) 
 * @return averaged backemf votlage of floating phases. 
 */
static uint16_t get_bemf_for_step(uint8_t step){
    switch (step){
        case 0: return get_averaged_bemf(&phase_B);  /* Phase B floating */
        case 1: return get_averaged_bemf(&phase_A);  /* Phase A floating */
        case 2: return get_averaged_bemf(&phase_C);  /* Phase C floating */
        case 3: return get_averaged_bemf(&phase_B);  /* Phase B floating */
        case 4: return get_averaged_bemf(&phase_A);  /* Phase A floating */
        case 5: return get_averaged_bemf(&phase_C);  /* Phase C floating */
        default: return 0;
    };
}

/**
 * @brief Detecting zero crossing in back emf signal
 * 
 * the floating phase oscillates around 2048 mid rail for our 12bit
 * adc with 3.3V reference(at least accroidng to google it will)
 * 
 * Zero crossing is just when the voltage crosses the midpoint. 
 * this indicates that the back emf has started to reverse directions, 
 * and we need the next commutation step to occur. 
 * 
 * @param step: current commutation step we are on 
 * @return 1 if zero crossing is detected, and 0 otherwise 
 */
static uint8_t detect_zero_crossing(uint8_t step){
    if(blanking_is_active()){
        return 0; 
    }

    static uint16_t prev_bemf = 0; 
    uint16_t threshold = 2048;
    uint16_t bemf = get_bemf_for_step(step);

    if((prev_bemf < threshold && bemf >= threshold) || //rising edge
       (prev_bemf >= threshold && bemf < threshold)) //fallingedge
       {
        prev_bemf = bemf;
        return 1;
       }

    prev_bemf = bemf;
    return 0; 
}

void app(void) {

    //starting adc with dma 
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_dma_buffer, 3);

    startup_begin();
    //converted TIM15 into a watchdog safety just incase we skip a step
    // and the motor gets stuck, itll force the next phase. not too sure if 
    // this iwll work though logically it makes abit of sense to me
    /* __HAL_TIM_SET_AUTORELOAD(&htim15, COMM_STEP_ARR);
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    HAL_TIM_Base_Start_IT(&htim15); */

    uint32_t last_update_ms = HAL_GetTick(); 
    uint32_t current_ms = 0; 


    while (1) {
        
        current_ms = HAL_GetTick(); 

        if((current_ms - last_update_ms) >= 10) {
            startup_update();
            last_update_ms = current_ms; 
        }

        if(motor_state == STATE_SENSORLESS && zero_crossing_detected){
            zero_crossing_detected = 0; 
            
            comm_step = (comm_step + 1) % 6;
            Set_Commutation_Step(comm_step);
            blanking_start(); 
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM15) {
        if(motor_state == STATE_STARTUP){
            //force a step 
            comm_step = (comm_step + 1U) % 6U;
            Set_Commutation_Step(comm_step);
            blanking_start(); 
        }
    }
    //if in sensorless mode, it dont do nothing 
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

    //updating the ring buffers with latest samples from dma buffer
    if(hadc-> Instance == ADC1){
        update_ring_buffer(&phase_A, adc_dma_buffer[0]);
        update_ring_buffer(&phase_B, adc_dma_buffer[1]);
        update_ring_buffer(&phase_C, adc_dma_buffer[2]);

        //check if floating pyhase voltage crossed midpoint
        if(detect_zero_crossing(comm_step)){
            zero_crossing_detected = 1;
        }
    }
}
