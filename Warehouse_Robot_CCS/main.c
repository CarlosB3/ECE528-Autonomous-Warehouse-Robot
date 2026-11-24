
/**
 * @file main.c
 * @brief ECE 528/L warehouse robot navigation.
 *
 * Station A/B: Timed left turn, then forward movement.
 * Station C: Original Lab 3 sensor-based turn.
 *
 * Phase 7: Detect delivery T-marker using three 0xFF readings.
 * Phase 8: Stop for three seconds at the delivery station.
 *
 * The robot remains stopped after the pause until the
 * turnaround and return-home phases are implemented.
 */

#include <string.h>
#include <stdint.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"
#include "inc/BLE_UART.h"
#include "inc/Bumper_Switches.h"

#define PWM_NOMINAL              2500
#define PWM_CORRECTION           1000
#define POSITION_GAIN_DIVISOR      48

// Original Lab 3 turn settings for Station C.
#define TURN_90_MIN_MS            180
#define TURN_180_MIN_MS           550
#define TURN_TIMEOUT_MS          1800

// Verified Station A/B maneuver settings.
#define AB_TURN_MS               1200
#define AB_FORWARD_MS             200

#define CORNER_IGNORE_MS          130
#define CROSS_INTERSECTION_MS     180
#define MIN_BRANCH_MS             350
#define MIN_RETURN_MS            1500
#define DEAD_END_SAMPLES            3
#define TURN_CENTER_SAMPLES         2
#define JUNCTION_CONFIRM_SAMPLES    2
#define JUNCTION_CLEAR_SAMPLES      8
#define JUNCTION_MIN_INTERVAL_MS  300

// NEW - Phase 7 and Phase 8.
#define DELIVERY_BRANCH_CONFIRM_SAMPLES  3
#define DELIVERY_MARKER_CONFIRM_SAMPLES  3
#define DELIVERY_PAUSE_MS             3000

static uint32_t Timer_A1_ms_elapsed = 0;

typedef enum
{
    CENTER              = 0b00011000,
    SLIGHT_LEFT_CENTER  = 0b00011100,
    SLIGHT_RIGHT_CENTER = 0b00111000,
    FAR_LEFT            = 0b00000001,
    FAR_RIGHT           = 0b10000000,
    LEFT_TURN           = 0b11111000,
    RIGHT_TURN          = 0b00011111,
    T_INTERSECTION      = 0b11111111,
    DEAD_END            = 0b00000000
} Line_Position;

typedef enum
{
    EXPLORE_MAZE,
    EXPLORE_INTERSECTION,
    RETURN_TO_START,
    FINISHED
} Navigation_State;

typedef enum
{
    LEFT_BRANCH_OUT,
    LEFT_BRANCH_BACK,
    RIGHT_BRANCH_OUT,
    RIGHT_BRANCH_BACK
} Intersection_Phase;

typedef enum
{
    NO_TURN,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    TURN_LEFT_180
} Turn_State;

static Navigation_State navigation_state = EXPLORE_MAZE;
static Intersection_Phase intersection_phase = LEFT_BRANCH_OUT;
static Turn_State turn_state = NO_TURN;

static uint32_t phase_start_ms = 0;
static uint32_t turn_start_ms = 0;
static uint32_t last_turn_finish_ms = 0;
static uint8_t dead_end_count = 0;
static uint8_t turn_center_count = 0;

// Dedicated Station A/B maneuver.
typedef enum
{
    AB_IDLE,
    AB_ROTATING,
    AB_MOVING_FORWARD
} AB_Maneuver_State;

static AB_Maneuver_State warehouse_ab_state = AB_IDLE;
static uint32_t warehouse_ab_stage_start_ms = 0;

static uint8_t branch_line_seen = 0;
static uint8_t navigation_error = 0;

static volatile uint8_t warehouse_junction_count = 0;
static volatile uint8_t warehouse_sensor_data = 0;

static uint8_t junction_detect_count = 0;
static uint8_t junction_clear_count = 0;
static uint8_t junction_latched = 0;
static uint32_t last_junction_ms = 0;

static volatile uint8_t warehouse_turn_requested = 0;
static uint8_t warehouse_route_armed = 0;
static uint8_t warehouse_center_count = 0;

static volatile uint8_t ble_last_button = 0;
static volatile uint8_t ble_last_state = 0;
static volatile uint32_t ble_packet_count = 0;
static volatile uint32_t ble_text_count = 0;

static volatile uint8_t ble_selected_destination = 0;
static volatile uint8_t warehouse_active_destination = 0;
static volatile uint8_t warehouse_run_enabled = 0;
static volatile uint8_t warehouse_stop_latched = 0;
static volatile uint8_t warehouse_collision_latched = 0;

// ============================================================
// NEW - Phase 7 and Phase 8 delivery detection state.
// ============================================================

typedef enum
{
    DELIVERY_WAIT_FOR_BRANCH,
    DELIVERY_SEARCH_FOR_MARKER,
    DELIVERY_PAUSING,
    DELIVERY_PAUSE_COMPLETE
} Delivery_Phase;

static Delivery_Phase delivery_phase = DELIVERY_WAIT_FOR_BRANCH;

static uint8_t delivery_branch_count = 0;
static uint8_t delivery_marker_count = 0;

static uint32_t delivery_pause_start_ms = 0;

// ============================================================
// Existing reflectance sensor LED display.
// ============================================================

void Detect_Line_Position(uint8_t data)
{
    LED1_Output(0x00);
    LED2_Output(0x00);

    switch (data)
    {
        case CENTER:
            LED2_Output(0x02);
            break;

        case SLIGHT_LEFT_CENTER:
            LED2_Output(0x03);
            break;

        case SLIGHT_RIGHT_CENTER:
            LED2_Output(0x05);
            break;

        case FAR_LEFT:
            LED2_Output(0x07);
            break;

        case FAR_RIGHT:
            LED2_Output(0x06);
            break;

        case LEFT_TURN:
            LED1_Output(0x01);
            LED2_Output(0x07);
            break;

        case RIGHT_TURN:
            LED1_Output(0x01);
            LED2_Output(0x06);
            break;

        case T_INTERSECTION:
            LED2_Output(0x04);
            break;

        case DEAD_END:
            LED2_Output(0x01);
            break;

        default:
            break;
    }
}

static uint8_t Count_Black_Sensors(uint8_t data)
{
    uint8_t count = 0;
    uint8_t bit;

    for (bit = 0; bit < 8; bit++)
    {
        if (data & (0x01u << bit))
        {
            count++;
        }
    }

    return count;
}

static uint8_t Is_T_Intersection(uint8_t data)
{
    return (data == T_INTERSECTION) ||
           (((data & 0x81u) == 0x81u) &&
            ((data & 0x18u) != 0) &&
            (Count_Black_Sensors(data) >= 6));
}

static uint8_t Is_Left_Corner(uint8_t data)
{
    return ((data & 0xE0u) == 0xE0u) &&
           ((data & 0x18u) != 0) &&
           ((data & 0x03u) == 0);
}

static uint8_t Is_Right_Corner(uint8_t data)
{
    return ((data & 0x07u) == 0x07u) &&
           ((data & 0x18u) != 0) &&
           ((data & 0xC0u) == 0);
}

void Line_Follower_Controller(uint8_t data)
{
    int32_t position;
    int32_t correction;

    if (data == DEAD_END)
    {
        Motor_Stop();
        return;
    }

    position = Reflectance_Sensor_Position(data);

    correction = (position * PWM_CORRECTION) /
                 POSITION_GAIN_DIVISOR;

    if (correction > PWM_CORRECTION)
    {
        correction = PWM_CORRECTION;
    }
    else if (correction < -PWM_CORRECTION)
    {
        correction = -PWM_CORRECTION;
    }

    if (position >= 250)
    {
        Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
    }
    else if (position <= -250)
    {
        Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
    }
    else
    {
        Motor_Forward(
            (uint16_t)(PWM_NOMINAL + correction),
            (uint16_t)(PWM_NOMINAL - correction)
        );
    }
}

// ============================================================
// Existing original Lab 3 turn logic for Station C.
// ============================================================

static void Start_Turn(Turn_State requested_turn)
{
    turn_state = requested_turn;
    turn_start_ms = Timer_A1_ms_elapsed;
    turn_center_count = 0;
    dead_end_count = 0;

    if (turn_state == TURN_RIGHT_90)
    {
        Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
    }
    else
    {
        Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
    }
}

// ============================================================
// Existing verified Station A/B maneuver.
// ============================================================

static void Start_AB_Maneuver(void)
{
    warehouse_ab_state = AB_ROTATING;
    warehouse_ab_stage_start_ms = Timer_A1_ms_elapsed;

    turn_center_count = 0;
    dead_end_count = 0;

    Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
}

static void Execute_AB_Maneuver(uint8_t data)
{
    uint32_t elapsed =
        Timer_A1_ms_elapsed - warehouse_ab_stage_start_ms;

    if (warehouse_ab_state == AB_ROTATING)
    {
        if (elapsed < AB_TURN_MS)
        {
            Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
            return;
        }

        warehouse_ab_state = AB_MOVING_FORWARD;
        warehouse_ab_stage_start_ms = Timer_A1_ms_elapsed;

        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
        return;
    }

    if (warehouse_ab_state == AB_MOVING_FORWARD)
    {
        if (elapsed < AB_FORWARD_MS)
        {
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
            return;
        }

        warehouse_ab_state = AB_IDLE;

        last_turn_finish_ms = Timer_A1_ms_elapsed;
        phase_start_ms = Timer_A1_ms_elapsed;

        dead_end_count = 0;
        branch_line_seen = 1;

        Line_Follower_Controller(data);
    }
}

// ============================================================
// Existing sensor-based turn execution for Station C.
// ============================================================

static void Execute_Turn(uint8_t data)
{
    uint32_t elapsed =
        Timer_A1_ms_elapsed - turn_start_ms;

    uint32_t minimum =
        (turn_state == TURN_LEFT_180) ?
        TURN_180_MIN_MS : TURN_90_MIN_MS;

    if (elapsed >= TURN_TIMEOUT_MS)
    {
        navigation_error = 1;
        navigation_state = FINISHED;
        turn_state = NO_TURN;

        Motor_Stop();
        return;
    }

    if ((elapsed >= minimum) &&
        ((data & 0x18u) == 0x18u) &&
        !Is_T_Intersection(data) &&
        !Is_Left_Corner(data) &&
        !Is_Right_Corner(data))
    {
        if (turn_center_count < TURN_CENTER_SAMPLES)
        {
            turn_center_count++;
        }
    }
    else
    {
        turn_center_count = 0;
    }

    if (turn_center_count >= TURN_CENTER_SAMPLES)
    {
        turn_state = NO_TURN;

        last_turn_finish_ms = Timer_A1_ms_elapsed;
        phase_start_ms = Timer_A1_ms_elapsed;

        dead_end_count = 0;
        branch_line_seen = 1;

        Line_Follower_Controller(data);
        return;
    }

    if (turn_state == TURN_RIGHT_90)
    {
        Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
    }
    else
    {
        Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
    }
}

static uint8_t Dead_End_Confirmed(uint8_t data)
{
    if (data == DEAD_END)
    {
        if (dead_end_count < DEAD_END_SAMPLES)
        {
            dead_end_count++;
        }
    }
    else
    {
        dead_end_count = 0;
    }

    return (dead_end_count >= DEAD_END_SAMPLES);
}

static void Follow_Track(uint8_t data)
{
    if ((Timer_A1_ms_elapsed - last_turn_finish_ms)
        >= CORNER_IGNORE_MS)
    {
        if (Is_Left_Corner(data))
        {
            Start_Turn(TURN_LEFT_90);
            return;
        }

        if (Is_Right_Corner(data))
        {
            Start_Turn(TURN_RIGHT_90);
            return;
        }
    }

    if (Is_T_Intersection(data))
    {
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
    }
    else
    {
        Line_Follower_Controller(data);
    }
}

// ============================================================
// Existing warehouse junction counting - unchanged.
// ============================================================

static void Count_Warehouse_Junctions(uint8_t data)
{
    uint8_t junction_detected;

    warehouse_sensor_data = data;

    junction_detected =
        Is_Left_Corner(data) || Is_T_Intersection(data);

    if (junction_detected)
    {
        junction_clear_count = 0;

        if (junction_detect_count < JUNCTION_CONFIRM_SAMPLES)
        {
            junction_detect_count++;
        }

        if ((junction_detect_count >= JUNCTION_CONFIRM_SAMPLES) &&
            !junction_latched)
        {
            junction_latched = 1;

            if ((warehouse_junction_count == 0) ||
                ((Timer_A1_ms_elapsed - last_junction_ms) >=
                 JUNCTION_MIN_INTERVAL_MS))
            {
                if (warehouse_junction_count < 3)
                {
                    warehouse_junction_count++;
                }

                last_junction_ms = Timer_A1_ms_elapsed;
            }
        }
    }
    else
    {
        junction_detect_count = 0;

        if (junction_latched)
        {
            junction_clear_count++;

            if (junction_clear_count >= JUNCTION_CLEAR_SAMPLES)
            {
                junction_latched = 0;
                junction_clear_count = 0;
            }
        }
    }
}

// ============================================================
// Navigation controller.
//
// NEW: Delivery-marker detection and 3-second pause.
// The existing turn-selection logic remains unchanged.
// ============================================================

void Navigation_Controller(uint8_t data)
{
    if (!warehouse_run_enabled)
    {
        Motor_Stop();
        return;
    }

    uint32_t elapsed =
        Timer_A1_ms_elapsed - phase_start_ms;

    if (navigation_state == FINISHED)
    {
        Motor_Stop();
        return;
    }

    // --------------------------------------------------------
    // NEW - Phase 8: Maintain the 3-second delivery pause.
    // --------------------------------------------------------
    if (delivery_phase == DELIVERY_PAUSING)
    {
        Motor_Stop();

        if ((Timer_A1_ms_elapsed - delivery_pause_start_ms)
            >= DELIVERY_PAUSE_MS)
        {
            delivery_phase = DELIVERY_PAUSE_COMPLETE;
        }

        return;
    }

    // NEW - Keep the robot stopped until Phase 9 is added.
    // Red LED1 indicates the 3-second pause has completed.
    if (delivery_phase == DELIVERY_PAUSE_COMPLETE)
    {
        Motor_Stop();
        LED1_Output(0x01);
        return;
    }

    // --------------------------------------------------------
    // Existing Station A/B maneuver - unchanged.
    // --------------------------------------------------------
    if (warehouse_ab_state != AB_IDLE)
    {
        Execute_AB_Maneuver(data);
        return;
    }

    // --------------------------------------------------------
    // Existing Station C sensor-based turn - unchanged.
    // --------------------------------------------------------
    if (turn_state != NO_TURN)
    {
        Execute_Turn(data);
        return;
    }

    // --------------------------------------------------------
    // NEW - Phase 7: Delivery T-marker detection.
    //
    // Only examine delivery markers after the selected
    // station turn has completed.
    //
    // First confirm a normal centered branch line.
    // This prevents the original junction's 0xFF
    // pattern from triggering a premature delivery stop.
    // --------------------------------------------------------
    if ((navigation_state == EXPLORE_MAZE) &&
        (warehouse_turn_requested == 2u))
    {
        if (delivery_phase == DELIVERY_WAIT_FOR_BRANCH)
        {
            if (((data & 0x18u) != 0u) &&
                !Is_T_Intersection(data) &&
                !Is_Left_Corner(data) &&
                !Is_Right_Corner(data))
            {
                if (delivery_branch_count <
                    DELIVERY_BRANCH_CONFIRM_SAMPLES)
                {
                    delivery_branch_count++;
                }

                if (delivery_branch_count >=
                    DELIVERY_BRANCH_CONFIRM_SAMPLES)
                {
                    delivery_phase = DELIVERY_SEARCH_FOR_MARKER;
                    delivery_marker_count = 0;
                }
            }
            else
            {
                delivery_branch_count = 0;
            }
        }
        else if (delivery_phase == DELIVERY_SEARCH_FOR_MARKER)
        {
            // Require exactly 0xFF for three
            // consecutive reflectance sensor readings.
            if (data == 0xFFu)
            {
                if (delivery_marker_count <
                    DELIVERY_MARKER_CONFIRM_SAMPLES)
                {
                    delivery_marker_count++;
                }

                if (delivery_marker_count >=
                    DELIVERY_MARKER_CONFIRM_SAMPLES)
                {
                    // Delivery marker confirmed.
                    delivery_phase = DELIVERY_PAUSING;

                    // Start the 3-second pause.
                    delivery_pause_start_ms =
                        Timer_A1_ms_elapsed;

                    Motor_Stop();
                    return;
                }
            }
            else
            {
                // A non-0xFF reading breaks the sequence.
                delivery_marker_count = 0;
            }
        }
    }

    // --------------------------------------------------------
    // Existing warehouse navigation - unchanged.
    // --------------------------------------------------------
    switch (navigation_state)
    {
        case EXPLORE_MAZE:
        {
            uint8_t previous_junction_count =
                warehouse_junction_count;

            if (!warehouse_route_armed)
            {
                if (((data & 0x18u) == 0x18u) &&
                    !Is_T_Intersection(data) &&
                    !Is_Left_Corner(data) &&
                    !Is_Right_Corner(data))
                {
                    if (warehouse_center_count < 3u)
                    {
                        warehouse_center_count++;
                    }

                    if (warehouse_center_count >= 3u)
                    {
                        warehouse_route_armed = 1u;
                    }
                }
                else
                {
                    warehouse_center_count = 0;
                }

                Line_Follower_Controller(data);
                break;
            }

            if (warehouse_turn_requested == 0u)
            {
                Count_Warehouse_Junctions(data);
            }

            if ((previous_junction_count <
                 warehouse_active_destination) &&
                (warehouse_junction_count ==
                 warehouse_active_destination) &&
                (warehouse_turn_requested == 0u))
            {
                warehouse_turn_requested = 2u;

                if (warehouse_active_destination <= 2u)
                {
                    // A/B: Verified timed maneuver.
                    Start_AB_Maneuver();
                }
                else
                {
                    // C: Original Lab 3 sensor-based turn.
                    Start_Turn(TURN_LEFT_90);
                }
            }
            else
            {
                Line_Follower_Controller(data);
            }

            break;
        }

        // Legacy Lab 3 maze states remain unchanged.
        case EXPLORE_INTERSECTION:
        {
            switch (intersection_phase)
            {
                case LEFT_BRANCH_OUT:

                    if (data != DEAD_END)
                    {
                        branch_line_seen = 1;
                    }

                    if ((elapsed >= MIN_BRANCH_MS) &&
                        branch_line_seen &&
                        Dead_End_Confirmed(data))
                    {
                        intersection_phase = LEFT_BRANCH_BACK;

                        Start_Turn(TURN_LEFT_180);
                    }
                    else
                    {
                        Follow_Track(data);
                    }
                    break;

                case LEFT_BRANCH_BACK:

                    if ((elapsed >= MIN_BRANCH_MS) &&
                        (Is_T_Intersection(data) ||
                         Is_Right_Corner(data)))
                    {
                        intersection_phase = RIGHT_BRANCH_OUT;
                        phase_start_ms = Timer_A1_ms_elapsed;

                        dead_end_count = 0;
                        branch_line_seen = 0;

                        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
                    }
                    else
                    {
                        Follow_Track(data);
                    }
                    break;

                case RIGHT_BRANCH_OUT:

                    if (elapsed < CROSS_INTERSECTION_MS)
                    {
                        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
                    }
                    else
                    {
                        if (data != DEAD_END)
                        {
                            branch_line_seen = 1;
                        }

                        if ((elapsed >= MIN_BRANCH_MS) &&
                            branch_line_seen &&
                            Dead_End_Confirmed(data))
                        {
                            intersection_phase = RIGHT_BRANCH_BACK;

                            Start_Turn(TURN_LEFT_180);
                        }
                        else
                        {
                            Follow_Track(data);
                        }
                    }
                    break;

                case RIGHT_BRANCH_BACK:

                    if ((elapsed >= MIN_BRANCH_MS) &&
                        (Is_T_Intersection(data) ||
                         Is_Left_Corner(data)))
                    {
                        navigation_state = RETURN_TO_START;

                        Start_Turn(TURN_LEFT_90);
                    }
                    else
                    {
                        Follow_Track(data);
                    }
                    break;

                default:
                    navigation_error = 1;
                    navigation_state = FINISHED;

                    Motor_Stop();
                    break;
            }

            break;
        }

        case RETURN_TO_START:

            if ((elapsed >= MIN_RETURN_MS) &&
                branch_line_seen &&
                Dead_End_Confirmed(data))
            {
                navigation_state = FINISHED;
                Motor_Stop();
            }
            else
            {
                Follow_Track(data);
            }
            break;

        case FINISHED:
        default:
            Motor_Stop();
            break;
    }
}

/**
 * @brief Timer_A1 periodic task.
 *
 * The reflectance sensors are sampled every 10 ms.
 * This timing remains unchanged.
 */
void Timer_A1_Periodic_Task(void)
{
    Timer_A1_ms_elapsed++;

    if ((Timer_A1_ms_elapsed % 10u) == 0u)
    {
        Reflectance_Sensor_Start();
    }

    if (((Timer_A1_ms_elapsed % 10u) == 1u) &&
        (Timer_A1_ms_elapsed > 10u))
    {
        uint8_t data = Reflectance_Sensor_End();

        Detect_Line_Position(data);

        Navigation_Controller(data);
    }
}

/**
 * @brief Permanent bumper-triggered emergency stop.
 */
static void Bumper_Collision_Handler(uint8_t bumper_state)
{
    (void)bumper_state;

    warehouse_collision_latched = 1;
    warehouse_run_enabled = 0;
    warehouse_stop_latched = 1;

    Motor_Stop();
}

/**
 * @brief Hardware initialization and Bluetooth commands.
 */
int main(void)
{
    char BLE_UART_Buffer[BLE_UART_BUFFER_SIZE] = {0};

    DisableInterrupts();

    Clock_Init48MHz();

    LED1_Init();
    LED2_Init();

    EUSCI_A0_UART_Init_Printf();

    BLE_UART_Init();

    P1->OUT &= ~0x40u;

    Reflectance_Sensor_Init();

    Motor_Init();

    Bumper_Switches_Init(&Bumper_Collision_Handler);

    // Latch any bumper already pressed at startup.
    {
        uint8_t initial_bumper_state = Bumper_Read();

        if (initial_bumper_state != 0u)
        {
            Bumper_Collision_Handler(initial_bumper_state);
        }
    }

    // SysTick remains disabled.

    Timer_A1_Interrupt_Init(
        &Timer_A1_Periodic_Task,
        TIMER_A1_INT_CCR0_VALUE
    );

    EnableInterrupts();

    while (1)
    {
        BLE_UART_InString(
            BLE_UART_Buffer,
            BLE_UART_BUFFER_SIZE
        );

        ble_text_count++;

        char *cr = strchr(BLE_UART_Buffer, '\r');

        if (cr != NULL)
        {
            *cr = '\0';
        }

        if (strcmp(BLE_UART_Buffer, "A") == 0)
        {
            if (!warehouse_run_enabled)
            {
                ble_selected_destination = 1;
            }
        }
        else if (strcmp(BLE_UART_Buffer, "B") == 0)
        {
            if (!warehouse_run_enabled)
            {
                ble_selected_destination = 2;
            }
        }
        else if (strcmp(BLE_UART_Buffer, "C") == 0)
        {
            if (!warehouse_run_enabled)
            {
                ble_selected_destination = 3;
            }
        }
        else if (strcmp(BLE_UART_Buffer, "START") == 0)
        {
            if ((ble_selected_destination != 0) &&
                !warehouse_run_enabled &&
                !warehouse_stop_latched)
            {
                warehouse_active_destination =
                    ble_selected_destination;

                warehouse_run_enabled = 1;

                BLE_UART_OutString("STARTED\r\n");
            }
        }
        else if (strcmp(BLE_UART_Buffer, "STOP") == 0)
        {
            warehouse_run_enabled = 0;
            warehouse_stop_latched = 1;

            Motor_Stop();

            BLE_UART_OutString("STOPPED\r\n");
        }
    }
}
