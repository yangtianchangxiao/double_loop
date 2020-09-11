#define cyclePlus 11
#define jiansu 20
//经计算得，一共需要转动220个脉冲，即90°
int coderA = 2;
int coderB = 3;
int num = 0;

int AB1 = 9;
int AB2 = 10;
float loc_kp = 0.72;
float loc_kd = 0.1;
float vel_kp = 1;
float vel_kd = 1;
int value = 0;
int x;
int k = 0;
int counter_tims = 0;
int interval = 0;
int a = 0;
int temp = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(coderA, INPUT);
  pinMode(coderB, INPUT);
  pinMode(AB1, OUTPUT);
  pinMode(AB2, OUTPUT);
  attachInterrupt(0, read_coder, CHANGE);
  attachInterrupt(1, read_coder, CHANGE);
  //attachInterrupt(1,read_coder,CHANGE);
  num = 0; //需要将脉冲数与角度对应起来
  Serial.begin(9600);
}

void read_coder()
{
  x = num = num + value;
  counter_tims++;
  counter_tims > 3 ? (a = micros(), interval = a - temp, counter_tims = 0, temp = a) : a = 0; //interval may have a relative bias if just ultilizing the current one.
}

void pid1(int goal_plus)
{
  struct pid_init
  {
    float err = 0;
    float output = 0;
    float err_last = 0;
    float err_last2 = 0;
  };
  struct pid_init loc_pid;
  struct pid_init vel_pid;
  float velocity = 0;
  while (1)
  {
    loc_pid.err = goal_plus - x;
    if (x > 215)
    {
      break;
    }
    if (loc_pid.err == loc_pid.err_last)
    {
      break;
    }
    loc_pid.output = loc_kp * loc_pid.err + loc_kd * (loc_pid.err - loc_pid.err_last);
    loc_pid.err_last = loc_pid.err;
    if (loc_pid.output > 150)
    {
      loc_pid.output = 150;
    }

    velocity = 3.14 / 2 / temp * 1000000;
    vel_pid.err = (loc_pid.output / 150 - velocity);
    vel_pid.output = vel_pid.output + vel_kp * (vel_pid.err - vel_pid.err_last) + vel_kd * (vel_pid.err + vel_pid.err_last2 - 2 * vel_pid.err_last);
    vel_pid.err_last = vel_pid.err;
    vel_pid.err_last2 = vel_pid.err_last;
    if (vel_pid.output > 120)
    {
      vel_pid.output = 120;
    }

    drive(255 - vel_pid.output, goal_plus);
  }
}

void pid2(int goal_plus)
{
  struct pid_init
  {
    float err = 0;
    float output = 0;
    float err_last = 0;
    float err_last2 = 0;
  };
  struct pid_init loc_pid;
  struct pid_init vel_pid;
  float velocity = 0;
  while (1)
  {
    loc_pid.err = x;
    if (x < 4)
    {
      break;
    }
    if (loc_pid.err == loc_pid.err_last)
    {
      break;
    }
    loc_pid.output = loc_kp * loc_pid.err + loc_kd * (loc_pid.err - loc_pid.err_last);
    loc_pid.err_last = loc_pid.err;
    if (loc_pid.output > 150)
    {
      loc_pid.output = 150;
    }

    velocity = 3.14 / 2 / temp * 1000000;
    vel_pid.err = (loc_pid.output / 150 - velocity);
    vel_pid.output = vel_pid.output + vel_kp * (vel_pid.err - vel_pid.err_last) + vel_kd * (vel_pid.err + vel_pid.err_last2 - 2 * vel_pid.err_last);
    vel_pid.err_last = vel_pid.err;
    vel_pid.err_last2 = vel_pid.err_last;
    if (vel_pid.output > 120)
    {
      vel_pid.output = 120;
    }

    drive(255 - vel_pid.output, goal_plus);
  }
}


void drive(int dutycycle, int goal_plus)
{
  if (goal_plus == 224)
  {
    value = 1;
    analogWrite(AB1, dutycycle);
    analogWrite(AB2, 255);
    delay(100);
  }
  else {
    value = -1;
    analogWrite(AB2, dutycycle);
    analogWrite(AB1, 255);
    delay(100);
  }
}


void loop() {
  // put your main code here, to run repeatedly:


  pid1(220);
  analogWrite(AB2, 255);
  analogWrite(AB1, 255);
  delay(200);
  pid2(0);
  analogWrite(AB2, 255);
  analogWrite(AB1, 255);
  delay(200);

}
