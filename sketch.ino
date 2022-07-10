
#include <dht.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <SD.h>

#define DHT22_PIN A0

dht DHT;

#define CS_PIN 10

File root;


const int pinB = 3;
const int pinG = 5;
const int pinR = 6;

#define UPPER_LIMIT                 255
#define NUMBER_OF_INPUT_OUTPUT      3                  // total system input/output
#define TOTAL_NUMBER_OF_MF          9                 // total of membership functions; NUMBER_OF_INPUT_OUTPUT * NUMBER_OF_MF
#define NUMBER_OF_RULE              9                 // number of rules in ruleBase
#define NUMBER_OF_INPUT             2                  // number of inputs for fuzzy logic
#define NUMBER_OF_OUTPUT            1                  // number of outputs for fuzzy logic
#define NUMBER_OF_MF                3                 // number of membership functions for each input/output
#define NUMBER_OF_IF_SIDE           2                 // number of ifSides for each rule
#define NUMBER_OF_THEN_SIDE         1                  // number of thenSides for each rule
#define TOTAL_NUMBER_OF_IF_SIDE     18                 // total number of ifSides for all rules
#define TOTAL_NUMBER_OF_THEN_SIDE   9                 // total number of thenSide for all rules

typedef struct io_type{
  char *name;
  int value;
  struct mf_type *membership_functions;
  struct io_type *next;
}io_type;

typedef struct mf_type{
  char *name;
  int value;
  int point1;
  int point2;
  int slope1;
  int slope2;
  struct mf_type *next;
}mf_type;

typedef struct rule_type{
  struct rule_element_type *if_side;
  struct rule_element_type *then_side;
  struct rule_type *next;
}rule_type;

typedef struct rule_element_type{
  int *value;
  struct rule_element_type *next;
}rule_element_type;


io_type inputOutput[NUMBER_OF_INPUT_OUTPUT];
mf_type mf[TOTAL_NUMBER_OF_MF];
rule_type ruleBase[NUMBER_OF_RULE];
rule_element_type ifSide[TOTAL_NUMBER_OF_IF_SIDE];
rule_element_type thenSide[TOTAL_NUMBER_OF_THEN_SIDE];
/* rules for fuzzy system declared here
   outer array matches with numberOfRule, innter array matches with numberOfInputOutput */

/* I suggest to change our names of values, for example "Medium" -> "ME" etc. */
char *rule[NUMBER_OF_RULE][NUMBER_OF_INPUT_OUTPUT] = {{"SM", "BI", "SM"},
                                                      {"SM", "ME", "SM"},
                                                      {"SM", "SM", "SM"},
                                                      {"ME", "BI", "ME"},
                                                      {"ME", "ME", "ME"},
                                                      {"ME", "SM", "BI"},
                                                      {"BI", "BI", "BI"},
                                                      {"BI", "ME", "ME"},
                                                      {"BI", "SM", "BI"}};
/* Name of each membership function. all inputs and outputs membership functions should be normalized
   http://www.drdobbs.com/cpp/fuzzy-logic-in-c/184408940 refer to this website to understand more about
   why name and point arrays should be normalized */
char *name[NUMBER_OF_MF] = {"SM", "ME", "BI"}; // match with numberMf
/* Four points input needed to conver to two points two slopes in initialize_system function
   Inner array always has four elements to construct
   membership functions shape such as triangle or Trapezoid
   outer array matches with numberMf
   These numbers from 2d array point are normalized to range 0-255 */
// int point[NUMBER_OF_MF][4] = { {0,    2,   2,   13}, // temperature
//                                {2,    13,  13,  26},
//                                {13,   26,  26,  40},
//                                {26,   35,  35,  40},
//                                {0,    5,   5,   30}, // humidity
//                                {5,   35,  35,  60},
//                                {35,   65,  65,  90},
//                                {65,   95,  95,  100}, 
//                                {0,    10,  10,  40}, // intensity
//                                {10,   50,  50,  70},
//                                {50,   60,  60,  80},
//                                {60,   90,  90,  95},
//                                {90,   97,  97,  100}};

int point[NUMBER_OF_MF][4];

int read_input_output_values();
void initialize_system();
void fuzzification();
void rule_evaluation();
void defuzzification();
void compute_degree_of_membership(mf_type *mf,int input);
int compute_area_of_trapezoid(mf_type *mf);
void initialize_system();
void put_system_outputs();
void get_system_inputs(int input1,int input2);
void compute_degree_of_membership(mf_type *mf,int input);


void setup() {
  Serial.begin(115200);

  pinMode(pinR, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(DHT22_PIN, INPUT);
  SD.begin(CS_PIN);
}

void loop() {
  read_input_output_values();
  DHT.read22(DHT22_PIN);
  analogWrite(pinR, LOW);
  analogWrite(pinG, LOW);
  analogWrite(pinB, LOW);
  int temperature = getTemperature();
  if (temperature < 0){
    temperature = 0;
  }
  if (temperature > 40){
    temperature = 40;
  }
  Serial.print("temp: ");
  Serial.print(temperature);
  int humidity = getHumidity() * 0.4;
  Serial.print(" hum: ");
  Serial.print(humidity);
  initialize_system();
  get_system_inputs(temperature,humidity);// this function is used for reading input for fuzzy logic, should be normalized to 0-255 range
  fuzzification();
  rule_evaluation();
  defuzzification();

  long fuzzy_output = inputOutput[2].value;
  Serial.print(" output: ");
  Serial.print(fuzzy_output);
  Serial.println(" ");
  long fuzzy_normalized = fuzzy_output;
  //float fuzzy_normalized = (float)((fuzzy_output + 40) * 250) / (float)120;
  // float fuzzy_normalized = map(fuzzy_output, -34, 75, 0, 250);
  // Serial.println(fuzzy_normalized);
  if (fuzzy_normalized <= 13) {
    analogWrite(pinB, 30);
  }

  if (fuzzy_normalized  > 13 && fuzzy_normalized  < 27) {
    analogWrite(pinG, 60);
  }

  if (fuzzy_normalized  >= 27) {
    analogWrite(pinR, 100);
  }
  delay(2000);
}


float getTemperature() {
  return DHT.temperature;
}

float getHumidity() {
  return DHT.humidity;
}

int read_input_output_values(){
  String str = "";
  int i = 0; //counter for mf
  int j = 0; //counter for element in mf
  
  // open the file
  File f = SD.open("wokwi.txt"); 
  
  // if there was an error
  if(f == NULL){
    Serial.print("Error while opening the file!");
    return(-1); 
  }
  // if there was no error
  else{ 
    while (f.available()) {
      // read characters one by one until you get a space
      char next_letter = char(f.read());
      if (next_letter == ' ') { //convert string to int and write it in the appropriate place in the
      // point array
        int number = str.toInt();
        point[i][j] = number;
        j += 1;
        str = "";
        if (j == 4){
          if(i == NUMBER_OF_MF){
            break;
          }
          j = 0;
          i += 1;
        }
      } else { //concat character with previous letters
        str += next_letter;
      }
    }
    int number = str.toInt();
    point[i][j] = number;
  }
  f.close(); // close file
  return(0);
}


void fuzzification(){
  int k = 0;
  for(int i = 0; i < NUMBER_OF_INPUT; i++)
  {
    for(int j = 0; j < NUMBER_OF_MF; j++)
    {
      compute_degree_of_membership(&mf[k],inputOutput[i].value);
      k++;
    }
  }
}


void rule_evaluation(){
  int a = 0;
  int b = 0;
  int strength;
  int nomatch=0;                   /* NEW, test some rules */
  for(int i = 0; i < NUMBER_OF_RULE; i++)
  {
    strength=UPPER_LIMIT;
    int *ptr3 = &i;
    printf("i=%d\n", i);
    for(int j = 0; j < NUMBER_OF_IF_SIDE; j++)
    {
      strength=min(strength,*(ifSide[a].value));
      a++;
      int *ptr4 = &a;

      int *ptr6 = &j;
      //printf("j=%d\n", j);

      //printf("a=%d %d %ls\n", a, *(ifSide[a].value),  ifSide[a].value);
    }
    for(int k = 0; k < NUMBER_OF_THEN_SIDE; k++)
    {
        int *ptr5 = &b;
        //printf("b=%d\n", b);
        *(thenSide[b].value)=max(strength,*(thenSide[b].value));      /* NEW */
        if(strength>0)nomatch=1;                      /* NEW */
        b++;
    }
  }
  if(nomatch==0)printf("NO MATCHING RULES FOUND!\n"); /* NEW */
}

void defuzzification(){
  long forOutputMf = TOTAL_NUMBER_OF_MF - NUMBER_OF_MF;
  long sum_of_products;
  long sum_of_areas;
  long area, centroid;
    sum_of_products=0;
    sum_of_areas=0;
    for(int i = forOutputMf; i < TOTAL_NUMBER_OF_MF; i++){
      area=compute_area_of_trapezoid(&mf[i]);
      centroid=mf[i].point1+(mf[i].point2-mf[i].point1)/2;
      sum_of_products+=area*centroid;
      sum_of_areas+=area;
    }
    
    if(sum_of_areas==0){                               /* NEW */
      printf("Sum of Areas = 0, will cause div error\n"); /* NEW */
      printf("Sum of Products= %d\n",sum_of_products);    /* NEW */
      inputOutput[2].value=0;                                        /* NEW */
      return;                                             /* NEW */
    }                                                     /* NEW */
    inputOutput[2].value=sum_of_products/sum_of_areas;
}

void compute_degree_of_membership(mf_type *mf, int input){
  int delta_1, delta_2;
  delta_1=input - mf->point1;
  delta_2=mf->point2 - input;
  if((delta_1<=0)||(delta_2<=0))mf->value=0;
  else{
    mf->value=min((mf->slope1*delta_1),(mf->slope2*delta_2));
    mf->value=min(mf->value,UPPER_LIMIT);
    //printf("testing = %d\n", mf->value);
  }
}

int compute_area_of_trapezoid(mf_type *mf){
  int run_1,run_2,area,top;
  int base;
  base=mf->point2 - mf->point1;

  if(mf->slope1 == 0){                                    /* NEW */
      printf("mf->slope1 = 0, will cause div error\n");
  }

  run_1=mf->value / mf->slope1;

  if(mf->slope2 == 0){                                    /* NEW */
      printf("mf->slope2 = 0, will cause div error\n");
  }
  run_2=mf->value / mf->slope2;
  top=base - run_1 - run_2;
  area=mf->value*(base+top)/2;
  return area;
}                                        /* END AREA OF TRAPEZOID */

void initialize_system(){                      /* NEW FUNCTION INITIALIZE */
  int forOutputMf = TOTAL_NUMBER_OF_MF - NUMBER_OF_MF;
  int a, b, c, d, x;
  int k = 0;
  int l = 0;
  /* name of system input/output */
  inputOutput[0].name = "Temperature";
  inputOutput[1].name = "Humidity";
  inputOutput[2].name = "Intensity";
  for(int i = 0; i < NUMBER_OF_INPUT_OUTPUT; i++){
    int *ptr2 = &i;
    printf("%d\n", i);

    for(int j = 0; j < NUMBER_OF_MF; j++){
      mf[k].name = name[j];
      mf[k].point1=point[j][0];                   /* left x axis value */
      b = point[j][1];
      c = point[j][2];
      mf[k].point2=point[j][3];                   /* right x axis value */
      if((b-point[j][0]) == 0){                                 /* NEW */
        printf("b-point[j][0] = 0, will cause div error\n");
      }
      mf[k].slope1=UPPER_LIMIT/(b-point[j][0]);     /* left slope */

      if((point[j][3]-c) == 0){                                    /* NEW */
        // printf("%d", c);
        
        //printf("%d%i", point[j][3]);
        printf("point[j][3]-c = 0, will cause div error\n");
        int *ptr = &c;
        //printf("%d\n", c);
        //printf("%d\n", *ptr);

      }
      mf[k].slope2=UPPER_LIMIT/(point[j][3]-c);     /* right slope */
      k++;
    }
  }
  /* READ RULES FILE; INITIALIZE STRUCTURES */
  for (int i = 0; i < NUMBER_OF_RULE; i++){
    for(int j = 0; j < NUMBER_OF_MF; j++)
    {
      if((strcmp(mf[j].name,rule[i][0]))==0)
      {
        ruleBase[i].if_side=ifSide;
        ifSide[l].value=&mf[j].value;  /* needs address here */
        l++;
        break;                           /* match found */
      }
    }
    for(int h = NUMBER_OF_MF; h < (2*NUMBER_OF_MF); h++)
    {
      if((strcmp(mf[h].name,rule[i][1]))==0)
      {
        ifSide[l].value= &mf[h].value;  /* needs address here */
        l++;
        break;                       /* match found */
      }
    }
    for(int g = forOutputMf; g < TOTAL_NUMBER_OF_MF; g++)
    {
      if((strcmp(mf[g].name,rule[i][2]))==0)
      {
        ruleBase[i].then_side=thenSide;
        thenSide[i].value=&mf[g].value; /* needs address here */
        break;                        /* match found */
      }
    }
  }                                     /* END WHILE READING RULES FILE */
}                                        /* END INITIALIZE */

void put_system_outputs(){                     /* NEW */
  int forOutputMf = TOTAL_NUMBER_OF_MF - NUMBER_OF_MF;
  int a = 0;
  for(int i = 0; i < NUMBER_OF_INPUT; i++)
  {
    printf("%s: Value = %d\n",inputOutput[i].name,inputOutput[i].value);
    for(int j = 0; j < NUMBER_OF_MF; j++)
    {
      printf("  %s: Value %d Left %d Right %d\n",
      mf[a].name,mf[a].value,mf[a].point1,mf[a].point2);
      a++;
    }
    printf("\n");
  }
  for(int i = NUMBER_OF_INPUT; i < NUMBER_OF_INPUT_OUTPUT; i++){
    printf("%s: Value= %d\n",inputOutput[i].name,inputOutput[i].value);
    for(int j = forOutputMf; j < TOTAL_NUMBER_OF_MF; j++){
      printf("  %s: Value %d Left %d Right %d\n",
      mf[j].name,mf[j].value,mf[j].point1,mf[j].point2);
    }
  }
  /* print values pointed to by rule_type (if & then) */
  printf("\n");
  int b = 0;
  int c = 0;
  for(int i = 0; i < NUMBER_OF_RULE; i++){
    printf("Rule #%d:",(i+1));
    for(int j = 0; j < NUMBER_OF_IF_SIDE; j++)
    {
      printf("  %d",*(ifSide[b].value));
      b++;
    }
    for(int k = 0; k < NUMBER_OF_THEN_SIDE; k++){
      printf("  %d\n",*(thenSide[c].value));
      c++;
    }
  }
  printf("\n");
}                                        /* END PUT SYSTEM OUTPUTS */

void get_system_inputs(int input1, int input2){         /* NEW */
  inputOutput[0].value = input1;
  inputOutput[1].value = input2;
}                                        /* END GET SYSTEM INPUTS */