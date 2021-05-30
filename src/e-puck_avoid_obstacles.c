#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/led.h>
#include <unistd.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define LEDS 10

static int get_time_step() {
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;
}

static void step() {
    if (wb_robot_step(get_time_step()) == -1) {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

static void passive_wait(double sec) {
    double start_time = wb_robot_get_time();
    do {
        step();
    } while (start_time + sec > wb_robot_get_time());
}


void fireLed(WbDeviceTag* ledsVector) {
  for (int i = 0; i < LEDS; i++){
    wb_led_set(ledsVector[i], true);
    passive_wait(0.5);
  }
}

void moveRobo( WbDeviceTag* esquerdaMotor, WbDeviceTag* direitaMotor, WbDeviceTag* proximitySensors){
  
  double esquerdaSpeed  = 0.3 * MAX_SPEED;
  double direitaSpeed = 0.3 * MAX_SPEED;
  double proximitySensorValues[8];
 
  for (int i = 0; i < 8 ; i++){
      proximitySensorValues[i] = wb_distance_sensor_get_value(proximitySensors[i]);
  }
  
  if (proximitySensorValues[5] > 120 ||
      proximitySensorValues[6] > 120 ||
      proximitySensorValues[7] > 120 ||
      proximitySensorValues[4] > 120 ) {
      esquerdaSpeed  += 0.3 * MAX_SPEED;
      direitaSpeed -= 0.3 * MAX_SPEED;
   }
  
   else if (proximitySensorValues[0] > 120 ||
      proximitySensorValues[1] > 120 ||
      proximitySensorValues[2] > 120 ||
      proximitySensorValues[3] > 120 ) {
      esquerdaSpeed  -= 0.3 * MAX_SPEED;
      direitaSpeed += 0.3 * MAX_SPEED;
   }
   
   wb_motor_set_velocity(*esquerdaMotor, esquerdaSpeed);
   wb_motor_set_velocity(*direitaMotor, direitaSpeed);

}

void verificaColisao(
  WbDeviceTag* proximitySensors, 
  WbFieldRef* boxFields, 
  double** boxInitialPositions,
  WbDeviceTag* ledsVector,
  WbNodeRef* robot){
  
  WbFieldRef robot_field = wb_supervisor_node_get_field(robot, "translation");
  const double* posicaoRobo = wb_supervisor_field_get_sf_vec3f(robot_field);
  for(int i = 0; i < 8; i++){
  
    float sensorValue = wb_distance_sensor_get_value(proximitySensors[i]);
    bool hit = false;
    if(sensorValue > 110){
        for(int i = 0; i < 9; i++){
    
          double* posicaoBox = wb_supervisor_field_get_sf_vec3f(boxFields[i]);
             
          for(int j = 0; j < 3; j++){
              printf("Detecting a collision of %f to position\n", posicaoBox[j]);
              printf("Box initial positional was %f\n", boxInitialPositions[i][j]);
              if(posicaoBox[j] != boxInitialPositions[i][j] && (posicaoBox[j] <= posicaoRobo[j]+0.001 || posicaoBox[j] >= posicaoRobo[j]-0.001)){              
                printf("Bagui leve mano\n");
                fireLed(ledsVector);
                hit = true;
                break;
              }
          }
          
          if(hit)
            break;
        }
        for(int i = 0; i < 9; i++){
          double* posicaoBox = wb_supervisor_field_get_sf_vec3f(boxFields[i]);
          for(int k = 0; k<3;k++){
             boxInitialPositions[i][k] = posicaoBox[k];  
          }  
        }
    }
  }
}

void setupSensors(WbDeviceTag* proximitySensors){

  char names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  
  for (int i = 0; i < 8 ; i++) {
    proximitySensors[i] = wb_robot_get_device(names[i]);
    wb_distance_sensor_enable(proximitySensors[i], TIME_STEP);
  }
  
}

void initializeMotor(const WbDeviceTag* esquerdaMotor, const WbDeviceTag* direitaMotor){

  wb_motor_set_position(*esquerdaMotor, INFINITY);
  wb_motor_set_position(*direitaMotor, INFINITY);
  
  wb_motor_set_velocity(*esquerdaMotor, 0.0);
  wb_motor_set_velocity(*direitaMotor, 0.0);

}

void defineCaixas(WbNodeRef* boxes, WbFieldRef* boxFields){
  
  char names[9][12] = {
    "caixinha_1", "caixinha_2", "caixinha_3", "caixinha_4",
    "caixinha_5", "caixinha_6", "caixinha_7", "caixinha_8","caixinha_9"
  };
  
  for(int i = 0; i < 9; i++){
    boxes[i] = wb_supervisor_node_get_from_def(names[i]);
    boxFields[i] = wb_supervisor_node_get_field(boxes[i], "translation");
  }
}

void defineCaixasPosicaoInicial(WbFieldRef* boxFields, double** boxInitialPositions){

  for(int i = 0; i < 9; i++){
    double* posicao_box = wb_supervisor_field_get_sf_vec3f(boxFields[i]);
    
    for(int j = 0; j < 3; j++){
       boxInitialPositions[i][j] = posicao_box[j];
 
    }
  }
}

void LogCaixasPosition(WbFieldRef* boxFields, double** boxInitialPositions){
  char names[9][12] = {
    "caixinha_1", "caixinha_2", "caixinha_3", "caixinha_4",
    "caixinha_5", "caixinha_6", "caixinha_7", "caixinha_8","caixinha_9"
  };
  for(int i = 0; i < 9; i++){
    double* posicao_box = wb_supervisor_field_get_sf_vec3f(boxFields[i]);
    printf("%s box \n",names[i]);
    for(int j = 0; j < 3; j++){
       printf("%f pos atual, pos inicial %f\n",posicao_box[j],boxInitialPositions[i][j]);
    }
  }
}

double** matrixBase(int l, int c){

  double** matrix = malloc(sizeof(double*) * l);
  
  for(int i = 0; i < l; i++){
      matrix[i] = malloc(sizeof(double) * c);
  }
  
  return matrix;
}

void setupLeds(WbDeviceTag* ledsVector){

  static const char* leds_names[LEDS] = { "led0", "led1", "led2", "led3", "led4", 
  "led5", "led6", "led7", "led8", "led9" };

  for (int i = 0; i < LEDS; i++)
    ledsVector[i] = wb_robot_get_device(leds_names[i]);
}

void defineLed(WbDeviceTag* ledsVector) {
  for (int i = 0; i < LEDS; i++)
    wb_led_set(ledsVector[i], false);
}

int main(int argc, char **argv) {
  fflush(stdout);
  wb_robot_init();
  
  WbDeviceTag esquerda_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag direita_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag proximitySensors[8];
  WbDeviceTag leds[LEDS];
   
  bool leds_values[LEDS];
  
  WbNodeRef boxes[9];
  WbFieldRef boxFields[9];
  double **boxInitialPositions = matrixBase(9,3);
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("destro_bot");
  WbFieldRef robot_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  setupSensors(proximitySensors);
  initializeMotor(&esquerda_motor, &direita_motor);
  defineCaixas(boxes, boxFields);
  defineCaixasPosicaoInicial(boxFields, boxInitialPositions);
  setupLeds(leds);
  
  while (wb_robot_step(TIME_STEP) != -1) {
    
    defineLed(leds);
    moveRobo(&esquerda_motor, &direita_motor,proximitySensors); 
    verificaColisao(proximitySensors,boxFields,boxInitialPositions,leds, robot_node);
    LogCaixasPosition(boxFields, boxInitialPositions);
  }
  
  wb_robot_cleanup();
  return 0;
}
