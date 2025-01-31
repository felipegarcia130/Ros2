#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

#include <MCR2_MotorDriver.h>
#include <MCR2_Encoder.h>
#include <MCR2_PIDController.h>

//Define Check macro 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RMCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RMW_RET_OK)){error_loop();}}  // Enter error loop on failure
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Define Safe Execution Macro
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

#define PWMpin 4
#define pinA   15
#define pinB   18
#define motorSign -1
#define motorTimeout_ms 2000

#define encoderPinA 34
#define encoderPinB 36
#define encoderSign -1

#define MAX_VELOCITY 10    // Maximum velocity setpoint (m/s)
#define MIN_VELOCITY -10   // Minimum velocity setpoint (m/s)

//Define Variables to be used

int motR_pins[3] = {PWMpin, pinA, pinB};     //Define the Motor Pins
int motR_sign = motorSign;                 //Define the motor rotation sign

int encoderR_pins[2] = {encoderPinA, encoderPinB};
int gear_ratio_r = 34;            //Define gear ratio
int encoderR_ticks = 48;          //Number of encoder ticks
int encoderR_sign = -1;           //Sign of the encoder velocity

float setPoint = 0.0;             //Set Point
float motorSpeed = 0.0;           //Motor Speed
float controlAction = 0.0;        //Control Action
float Kp_R=0.050;                 //The Kp parameter of the pid controller
float Ti_R=0.08;                  //The Ti parameter of the pid controller
float Td_R=0.0;                   //The Td parameter of the pid controller
float uR_min=-1;                  //The lower limit of the controller output
float uR_max=1;                   //The upper limit of the controller output
float u=0;                        // Control input u

bool transitionInProgress = false; // Flag to indicate if FOH is active
float currentSetpoint = 0.0;       // Current smoothed setpoint
float steps = 0.0;
float increment = 0.0;             // Increment per step for FOH
const unsigned long transitionDuration = 80; // Transition duration in ms


unsigned long prev_time = 0.0;
unsigned long last_message_time = 0.0;
unsigned long start_time = 0.0;

//Define Timeouts to be Used in mS
const unsigned int control_timeout = 50;

//WiFI Information
micro_ros_agent_locator locator;

IPAddress local_ip = {10, 16, 1, 1};
IPAddress gateway = {10, 16, 1, 1};
IPAddress subnet = {255, 255, 255, 0};

const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";

const char* agent_ip = "10.16.1.2";
const int agent_port = 8888;



//Define Enum with Connection States
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//Instantiate the required components for the node
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t rcl_allocator;

//Create Publishers
rcl_publisher_t motorSpeed_pub, motorCtrl_pub, motorCtrlSP_pub, motorCtrlErr_pub;

//Create Subscribers
rcl_subscription_t setPoint_sub;

//Create Timers
rcl_timer_t control_timer;

//Define messages to be used
std_msgs__msg__Float32 setPoint_msg, motSpeed_msg, motCtrl_msg, ctrlSetpoint_msg, ctrlError_msg;

//Initialise Objects
MotorDriver motor_R;
Encoder encoder_R;
PIDController PID_R;

//Define Setup Functions for motors and encoders
void setupMotEnc();
void setupControl();
void setupWiFi();

//Define create and destroy entities functions
bool create_entities();
void destroy_entities();



//Define the error loop functions
void error_loop();

//Subscriber Callback
void setPoint_cb(const void * msgin)
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  
  if (!transitionInProgress){
      setPoint = constrain(msg->data, MIN_VELOCITY, MAX_VELOCITY);
  }

  last_message_time = micros();
  
  // Mark the transition as in progress
  transitionInProgress = true;
  
  // Calculate increment for FOH
  steps = transitionDuration / ((float)control_timeout);
  increment = (setPoint - currentSetpoint) / steps;
  
}

//Timer Callback
void control_cb(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    
    encoder_R.ReadSensors();                              // Read Encoders
    motorSpeed = encoder_R.Get_Speed();

    if ((micros() - last_message_time) > (motorTimeout_ms * 1000) & !transitionInProgress) {
      // If no new message received, stop the motor
      setPoint = 0.0;
      transitionInProgress = true;
      // Calculate increment for FOH
      steps = transitionDuration / ((float)control_timeout);
      increment = (setPoint - currentSetpoint) / steps;
    }

    if (transitionInProgress) {
    // Gradually update the setpoint using FOH
    if (abs(setPoint - currentSetpoint) > abs(increment)) {
      currentSetpoint += increment;
      }
    else {
      // Transition complete
      currentSetpoint = setPoint;
      transitionInProgress = false;
      }
    }

    u = PID_R.GetControl(currentSetpoint, motorSpeed);     //Get Control
    motor_R.MotorWrite(u);                                //Motor Write
     
    motSpeed_msg.data = motorSpeed;
    motCtrl_msg.data = u;
    ctrlSetpoint_msg.data = currentSetpoint;
    ctrlError_msg.data = currentSetpoint - motorSpeed;
    
    RCSOFTCHECK(rcl_publish(&motorCtrlSP_pub, &ctrlSetpoint_msg, NULL));
    RCSOFTCHECK(rcl_publish(&motorCtrlErr_pub, &ctrlError_msg, NULL));
    RCSOFTCHECK(rcl_publish(&motorSpeed_pub, &motSpeed_msg, NULL));
    RCSOFTCHECK(rcl_publish(&motorCtrl_pub, &motCtrl_msg, NULL));
    
  }
}

void setup() {

  setupWiFi();

  //set_microros_transports();  //Set communication transport (Serial is default)

  setupMotEnc();
  setupControl();

  start_time = micros();

  state = WAITING_AGENT;      //Initial State

}

void loop() {
      //State Machine waiting for connection with a host
    switch (state) {

      //Ping the host until there is answer, do it every 500 mS. If a connection is detected change state to AGENT_AVAILABLE
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;

      //If agent is available create the entities required for micro_ros, publishers and subscribers. ONce created change state to AGENT_CONNECTED
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        break;

      //Execute the "Executor" with a delay stated by 100 mS. Keep Pinging the host if there is a disconnection change state to AGENT_DISCONNECTED
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        }
        break;

      //Destroy the entities created to free memory
      case AGENT_DISCONNECTED:
        
        transitionInProgress = false;
        setPoint = 0.0;

        while (abs(motorSpeed) > 0.05){
          encoder_R.ReadSensors();                              // Read Encoders
          motorSpeed = encoder_R.Get_Speed();

          if (!transitionInProgress){
            transitionInProgress = true;
            // Calculate increment for FOH
            steps = transitionDuration / ((float)control_timeout);
            increment = (setPoint - currentSetpoint) / steps;
          }
          else {
            // Gradually update the setpoint using FOH
              if (abs(setPoint - currentSetpoint) > abs(increment)) {
                  currentSetpoint += increment;
              } else {
                  // Transition complete
                  currentSetpoint = setPoint;
              }

            u = PID_R.GetControl(currentSetpoint, motorSpeed);
            motor_R.MotorWrite(u);
          }
          delay(50);
        }

        motor_R.MotorWrite(0.0);
        setPoint_msg.data = 0.0;
        motSpeed_msg.data = 0.0;
        motCtrl_msg.data = 0.0;
        ctrlSetpoint_msg.data = 0.0;
        
        motorSpeed = 0.0;
        u = 0.0;


        destroy_entities();
        state = WAITING_AGENT;
        break;

      //Default state
      default:
        break;
    }

}

//Create micro_ros entities if connected
bool create_entities()
{
  //Create Entities
  // create memory allocator
  rcl_allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &rcl_allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor_control", "", &support));

  //create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &motorSpeed_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/speed"));

  RCCHECK(rclc_publisher_init_best_effort(
    &motorCtrl_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/ctrl/output"));

  RCCHECK(rclc_publisher_init_best_effort(
    &motorCtrlSP_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/filtered_set_point"));

  RCCHECK(rclc_publisher_init_best_effort(
    &motorCtrlErr_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/ctrl/error"));

  //create subscriber
  RCCHECK(rclc_subscription_init_default(
    &setPoint_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/set_point"));

    //create timers
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(control_timeout),
    control_cb));

  //create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &rcl_allocator));

  //add timers and subscribers to the executor in order of execution
  RCCHECK(rclc_executor_add_subscription(&executor, &setPoint_sub, &setPoint_msg, &setPoint_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  // Set executor semantics to LET
  RCCHECK(rclc_executor_set_semantics(&executor, LET));

  return true;
}

//Destroy created entities if there is a disconnection (free memory)
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);     //Get the context address (structure that manages the lifecycle, configuration, and state of the Micro-ROS system)
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);      //Sets the DDS-XRCE session spin time for destroying entities

  rcl_publisher_fini(&motorSpeed_pub, &node);      //Destroy Publishers
  rcl_publisher_fini(&motorCtrl_pub, &node);
  rcl_publisher_fini(&motorCtrlSP_pub, &node);
  rcl_publisher_fini(&motorCtrlErr_pub, &node);
  rcl_timer_fini(&control_timer);                  //Destroy Timers
  rcl_subscription_fini(&setPoint_sub, &node);     //Destroy Subscribers
  rclc_executor_fini(&executor);                   //Destroy Executor
  rcl_node_fini(&node);                            //Destroy Node
  rclc_support_fini(&support);                     //Destroy Support (Context)
}

//Error Loop Function
void error_loop(){
  while(1){
    // Toggle LED state
      printf("Failed initialisation. Aborting.\n");  // Print error message
    // Wait for 100 milliseconds before retrying
    delay(100);
  }
}

void setupMotEnc()
{
  //Setup the Motor Object
  motor_R.SetSign(motR_sign);                                            //Set up motor sign
  motor_R.DriverSetup(motR_pins[0], 0, motR_pins[1], motR_pins[2]);      //Setup the motor pins (PWMpin, Pin A, Pin B)
  motor_R.MotorWrite(0);                                                 //Write 0 velocity to the motor when initialising

  //Setup the Encoder Object
  encoder_R.SetTicksPerRev(encoderR_ticks);                              //Set the number of enconder pulses per revolution
  encoder_R.SetGearRatio(gear_ratio_r);                                  //Set the gear ratio
  encoder_R.SetSign(encoderR_sign);                                      //Set the encoder velocity sign
  encoder_R.Encoder_setup(encoderR_pins[0], encoderR_pins[1], 'R');      //Set encoder pins
}

void setupControl()
{
  //Setup the PID Control Object
  PID_R.SetParameters(Kp_R,Ti_R,Td_R);                                    //Set up the PID Parameters
  PID_R.SetControlLimits(uR_min,uR_max);                                  //Set control output Limits
}

void setupWiFi(){
  locator.address.fromString(agent_ip);
  locator.port = agent_port;

  WiFi.mode(WIFI_AP_STA);  
  WiFi.softAP(ssid,password);
  delay(1000);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  RMCHECK(rmw_uros_set_custom_transport(
    false,
    (void *) &locator,
    arduino_wifi_transport_open,
    arduino_wifi_transport_close,
    arduino_wifi_transport_write,
    arduino_wifi_transport_read
  ));
}
