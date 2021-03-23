/*
 * 
 * 중앙대  fin control code
 *
 * 무단 공유 금지
 *
 * Date         Author      Comment
 * ----------   ----------  ---------------------------------------------------
 * 2017.6.9     김승현       Capstone design class / rocket fin control/MACH 2th
 */
#include <Wire.h>
#include <myAhrs.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#define SamplingTime 0.035

void print_3d_float(const char* tag_x, const char* tag_y, const char* tag_z, float x, float y, float z);/* print_3d_int 는 3개의 변수를 데리고 다니며 그를 구성하는 함수가 있음 밑에 나옴 ^^ */
void Limit_cut(float *ff, long int MIN_LIMIT, long int MAX_LIMIT);            /*함수를 쓰려면 함수선언이 필요 !!*/

myAhrs mysensor;
    float f1 = 0;
    float f2 = 0;
    float f3 = 0;
    float f4 = 0;                                          //f1,2,3,4 는 아웃풋으로 내보내는 값입니다 
    int p1,p2,p3,p4;                                       //f1,2,3,4 는 정수가 아니라서 서보모터가 읽어드릴 수 없어요 그래서 정수로 바꿔주는 작업을 해주게 됩니다 그 때 쓰이는 변수 에요
    //Roll control
    long int Roll_cmd             = 0;                     //무시 해도 되여 우리꺼는 별도의 조종기 입력이 없기 때문에
    long int phi_feedback         = 0;                     //pid 제어 에서 에러를 구성할 때 우리 꺼는 얼마나 기울어져 있는지 , 얼마나 급격하게 움직이는지 를 에러로 취급합니다~ 저 피드백을 0으로 만들려고 노력하는게 pid 제어의 핵심이죠
    long int phi_dot_feedback     = 0;                      // dot 은 각속도 를 의미 해요 ~  phi:롤 theta:피칭 psi :요잉
    
                       
    float roll_cont_ANG           = 0;                     //롤 의 pid 그중에서도 각도에 대한 pid 제어를 거치고 난 뒤 합산된 값이에요 이 값이 아웃풋에 반영됩니다.
    float ROLL_ANG_ERR            = 0;                     // 롤링 기준에서 볼때 각도가 얼마나 기울었는지를 나타냅니다. 이게 0이 아니면 아웃풋에 일정 값이 생겨서 서보를 움직이게 하죠
    float ROLL_ANG_P              = 0;                        
    float ROLL_ANG_I              = 0;
    float ROLL_ANG_D              = 0;
    float ROLL_ANG_ERR_PRE        = 0;                     // 얘는 pid 중 d 때문에 필요해요 c언어는 미분이 안되기 때문에 순간변화량으로 미분을 대체하여 나타냅니다. 밑에 내려가보시면 뭔지 알게 됩니다.
    
    float roll_cont               = 0;                     //ANG 가 안붙은거는 다 각속도에 대한 내용 이에요 ~
    float ROLL_RATE_ERR           = 0;
    float ROLL_RATE_P             = 0;
    float ROLL_RATE_I             = 0;
    float ROLL_RATE_D             = 0;
    float ROLL_RATE_ERR_PRE       = 0;

    float R_ANG_Pgain             = 3; // 2015.02.20       
    float R_ANG_Igain             = 0.05;
    float R_ANG_Dgain             = 1;  
    
    float R_inner_Pgain           = 1; // 2015.02.20     // 매트랩 블랙박스와 pid tuner 로 구한 PID 계수를 써주는게 가장 이상적입니다 ^^ 
    float R_inner_Igain           = 0.4;                  // 서보가 계속 흐르는 현상은 I 가 높기 때문에 나타나는 현상입니다. 즉 settling time 이 막 늘어나죠 !! 왜냐면 I 의 역할은 에러를 누적시키기 때문이죠 ~ 근데 또 너무 적으면 Steady state error 가 발생해요 ~
    float R_inner_Dgain           = 0.01;                // 매트랩으로 구했더니 0으로 떴어요 ㅋㅋ 근데 그냥 엄청 작은값이나마 넣어 줬습니다.
    

    
    //Pitch control
    long int Pitch_cmd            = 0;                     //무시
    long int theta_feedback       = 0;                     //  위 내용과 동일 적용되는 것만 다를뿐 ~
    long int theta_dot_feedback   = 0;
    
    
    float pitch_cont_ANG          = 0;
    float PITCH_ANG_ERR           = 0;
    float PITCH_ANG_P             = 0;
    float PITCH_ANG_I             = 0;
    float PITCH_ANG_D             = 0;
    float PITCH_ANG_ERR_PRE       = 0;
    
    float pitch_cont              = 0;
    float PITCH_RATE_ERR          = 0; 
    float PITCH_RATE_P            = 0;
    float PITCH_RATE_I            = 0;
    float PITCH_RATE_D            = 0;
    float PITCH_RATE_ERR_PRE      = 0;

    float P_ANG_Pgain           = 3;
    float P_ANG_Igain           = 0.05;
    float P_ANG_Dgain           = 1;   
    

    float P_inner_Pgain           = 2;
    float P_inner_Igain           = 0.4;
    float P_inner_Dgain           = 0.01;  
    

   //Yaw Control
    float yaw_cont_ANG          = 0;
    float YAW_ANG_ERR           = 0;
    float YAW_ANG_P             = 0;
    float YAW_ANG_I             = 0;
    float YAW_ANG_D             = 0;
    float YAW_ANG_ERR_PRE       = 0;


   
    //long int Yaw_cmd            = 0;
    long int psi_feedback         = 0;
    long int psi_dot_feedback     = 0;
    float yaw_cont                = 0;
    float YAW_RATE_ERR            = 0;
    
    float YAW_RATE_P              = 0;
    float YAW_RATE_I              = 0;
    float YAW_RATE_D            = 0;
    float YAW_RATE_ERR_PRE      = 0;

    float Y_ANG_Pgain           = 1;
    float Y_ANG_Igain           = 0.01;
    float Y_ANG_Dgain           = 0.0135; 
   
    float Y_inner_Pgain           = 1;
    float Y_inner_Igain           = 0.05;
    float Y_inner_Dgain           = 0.0135;    
    float AX,AY,AZ, GX,GY,GZ,MX,MY,MZ,ROLL,PITCH,YAW; 
    
    Servo fin1_servo;                                 // 요거는 알아두면 좋아 ~~ Servo 는 내가 서보를 달겠다 !! 라는 걸 선언하는 거야 
    Servo fin2_servo;
    Servo fin3_servo;
    Servo fin4_servo;
    Servo para_servo; //낙하산
    int val1,val2,val3,val4;
    
    SoftwareSerial nano1(2,3);
    
    int parachute_STATE=0;
void setup()
{
  Serial.begin(115200);// 시리얼 통신은 실제로 어떤값들이 흘러 들어가고 있는지 내눈으로 볼 수 있게 해주는 거야 이게 없으면 실제로 내가 잘 코딩을 했는지 서보 움직임만으로 만 알 수 있어서 구체적인 값을 확인하기 위해 필요 !! 통신 채널은 여러개야 ~ 우리는 그중에 115200 채널을 써
  
  nano1.begin(57600);
  Serial.println("##############################");
  Serial.println("   EX 01) read sensor value");         // 통신 채널에 등장하는 내용 들이야 ㅋㅋ printf 랑 같은 그런거 ㅋㅋ 아두이노 오른쪽 상단에 돋보기 모양 있지? 아두이노 연결된 상태로 저버튼 클릭하면 통신되는 내용이 나와
  Serial.println("##############################");
  
  if(mysensor.begin(&Wire) == false) {
    Serial.println("Initialize error");                  // 이거는 연결이 잘 안되어 있으면 에러 내용을 띄우게 해논거야 시리얼 통신 할 때
    while(1);
  }  
  
  Serial.print("REV ID : "); Serial.println(mysensor.revision());
  Serial.print("STATUS : ");                 
  if(mysensor.status() == true) {
    Serial.println("OK");
  }                                                       // 여기도 연결이 성공 되면 OK 라고 뜨게 해논거
  else {
    Serial.println("ERROR");
  }
  fin1_servo.attach(9);                                   // fin1_servo 는 내가 만든 함수야 ㅋㅋ 위에 Servo 라고 하고 내가 명명하고 싶은 함수이름 쓰면 돼 ~ 번호는 몇번 채널에 전선을 꼽을 건지 나타낸거야 나는 4 5 6 7 번 에다가 전선을 연결할거야 
  fin2_servo.attach(10); 
  fin3_servo.attach(11);      
  fin4_servo.attach(12);       
  para_servo.attach(4);   
  para_servo.write(2); 
}

void loop()                                                // 루프는 흔히 보통 c언어 에서 for 구문 같은거야 아두이노는 친절하게 setup 과 loop 두가지로 분리 시켜 놨지 ~ 
{
  static int count;
  static int sequence;
  
  count++; /*1,2,3,4 루프 돌 때 마다 몇번째 루프를 돌고 있는지 나타내기 위함*/

	delay(35); // ,  delay(1000) 이 1초다 ^^ 오해 하지말길 우리 로켓이 0.035초 니까 delay를 35로 잡아야 할듯
  
  /*
   * read sensor data from myAhrs
   */
  if(mysensor.update_data() == false) {
    Serial.println("Update error\n");
  }/*만약 데이터를 받아 올 수 없다면 여기서 끝내 버림!!*/

 
Serial.print(count); Serial.print(" : ");

  /*
   * access acceleration (floating point. unit(G))
   */
  AX= mysensor.get_data(myAhrs::ACCEL_X);
  AY= mysensor.get_data(myAhrs::ACCEL_Y);                    // 이거는 일반 아두이노에서는 못쓰고 myAHRS 센서를 사용 하면서 #include 에 myAhrs.h 를 읽어 들여와야 쓸 수 있는 함수 들이야 자이로 값들을 자동으로 읽어줘 ~~
  AZ= mysensor.get_data(myAhrs::ACCEL_Z);
  
  print_3d_float("AX", "AY", "AZ", AX, AY, AZ);
  
  /*
   * access angular velocity (floating point. unit(dps - degree per second))
   */
  GX= mysensor.get_data(myAhrs::GYRO_X);    /*원래 x y z 로 변수 받아서 처리 제어 프로그램 합치면 변수 유지 안됨 여기서는 print_3d 라는 별도 함수로 나오기 때문에 x,y,z 가 리셋 되는 것임*/
  GY = mysensor.get_data(myAhrs::GYRO_Y);
  GZ= mysensor.get_data(myAhrs::GYRO_Z);   /* 이게 핵심 */
  
  print_3d_float("GX", "GY", "GZ", GX, GY, GZ);
Serial.println ("\n  " );

   /*
   * access acceleration (floating point. unit(G))
   */
  MX = mysensor.get_data(myAhrs::MAG_X);
  MY = mysensor.get_data(myAhrs::MAG_Y);
  MZ = mysensor.get_data(myAhrs::MAG_Z);
  
  print_3d_float("MX", "MY", "MZ", MX, MY, MZ);
  
  /*
   * access angular velocity (floating point. unit(dps - degree per second))
   */
  ROLL = mysensor.get_data(myAhrs::ATTI_ROLL);
  PITCH= mysensor.get_data(myAhrs::ATTI_PITCH);
  YAW = mysensor.get_data(myAhrs::ATTI_YAW);
  
  print_3d_float("ROLL", "PITCH", "YAW", ROLL, PITCH, YAW);
Serial.println ("\n" );

        phi_feedback = ROLL;
        phi_dot_feedback = GX;    //phi 는 Roll 의 각도 , dot 은 각속도를 의미 
        theta_feedback = -PITCH;
        theta_dot_feedback = -GY; //theta 는 pitch 의 각도, dot 은 각속도 를 의미 
        psi_feedback = -YAW;
        psi_dot_feedback = -GZ; // psi 는 Yaw 의 각도 , dot 은 각속도 를 의미 
        
       // ROLL 각속도 Control ( ANG_P * RATE_PID controller )
         
        ROLL_RATE_ERR = - phi_dot_feedback;                                                      //여기부터 pid 제어야  순서대로 롤 각속도,각도 , 피치 각속도 , 각도 요잉 각속도 이렇게 5가지에 대해서 pid 제어를 진행 했어 ~
        ROLL_RATE_P = ROLL_RATE_ERR * R_inner_Pgain;
        ROLL_RATE_I = ROLL_RATE_I + (ROLL_RATE_ERR * R_inner_Igain) * SamplingTime; 
        Limit_cut(&ROLL_RATE_I,-100,100);
        ROLL_RATE_D = (ROLL_RATE_ERR - ROLL_RATE_ERR_PRE)/SamplingTime * R_inner_Dgain;
        ROLL_RATE_ERR_PRE = ROLL_RATE_ERR;  //PID 중 D 는 특이하게 이전 값을 필요로 함 왜냐하면 얘는 미분개념인데 C 언어에서는  리미트 이런거는 못쓰자나 그래서 변화율로 처리를 함 ㅋㅋ 한칸 위를 봐봐 
        
        roll_cont = ROLL_RATE_P + ROLL_RATE_I + ROLL_RATE_D;
     
     // ROLL 각 Control
        ROLL_ANG_ERR = -phi_feedback ; // 풍동 실험용이라 (90 -파이) 피드백임.
        ROLL_ANG_P = ROLL_ANG_ERR * R_ANG_Pgain;
        ROLL_ANG_I = ROLL_ANG_I + (ROLL_ANG_ERR * R_ANG_Igain) * SamplingTime;
        Limit_cut(&ROLL_ANG_I,-100,100);
        ROLL_ANG_D = (ROLL_ANG_ERR - ROLL_ANG_ERR_PRE)/SamplingTime * R_ANG_Dgain;
        ROLL_ANG_ERR_PRE = ROLL_ANG_ERR;  //PID 중 D 는 특이하게 이전 값을 필요로 함 왜냐하면 얘는 미분개념인데 C 언어에서는  리미트 이런거는 못쓰자나 그래서 변화율로 처리를 함 ㅋㅋ 한칸 위를 봐봐 
        
        roll_cont_ANG = ROLL_ANG_P + ROLL_ANG_I + ROLL_ANG_D;

        
        // PITCH 각속도 Control ( ANG_P * RATE_PID controller )
       
        PITCH_RATE_ERR = - theta_dot_feedback;
        PITCH_RATE_P = PITCH_RATE_ERR * P_inner_Pgain;
        PITCH_RATE_I = PITCH_RATE_I + (PITCH_RATE_ERR * P_inner_Igain) * SamplingTime;
        Limit_cut(&PITCH_RATE_I,-100,100);
        PITCH_RATE_D = (PITCH_RATE_ERR - PITCH_RATE_ERR_PRE)/SamplingTime * P_inner_Dgain;
        PITCH_RATE_ERR_PRE = PITCH_RATE_ERR;
        
        pitch_cont = PITCH_RATE_P + PITCH_RATE_I + PITCH_RATE_D;
        
        // PITCH 각도 Control  
        PITCH_ANG_ERR = - theta_feedback ;                                                      // 실험용 이라서 -90 임
        PITCH_ANG_P = PITCH_ANG_ERR * P_ANG_Pgain;
        PITCH_ANG_I = PITCH_ANG_I + (PITCH_ANG_ERR * P_ANG_Igain) * SamplingTime;
        Limit_cut(&PITCH_ANG_I,-100,100);
        PITCH_ANG_D = (PITCH_ANG_ERR - PITCH_ANG_ERR_PRE)/SamplingTime * P_ANG_Dgain;
        PITCH_ANG_ERR_PRE = PITCH_ANG_ERR;
        pitch_cont_ANG = PITCH_ANG_P + PITCH_ANG_I + PITCH_ANG_D;
        
        
      // YAW RATE Control ( RATE_PI controller )                            
        YAW_RATE_ERR = - psi_dot_feedback;   // 내가 명령한 YAW - 벗어난 각속도 = 에러!  
        YAW_RATE_P = YAW_RATE_ERR * Y_inner_Pgain;   
        YAW_RATE_I = YAW_RATE_I + YAW_RATE_ERR*Y_inner_Igain*SamplingTime; //sampling time 이 좁을 수록 데이터를 빨리빨리 받겠지? 근데 이게 또 아두이노 랑 센서랑 좀 맞아야 된다고 들었음. 무작정 0.0001 초로 한다고 다 소화 못함... 체 해가지고 렉걸릴수도.. 
        YAW_RATE_D = (YAW_RATE_ERR - YAW_RATE_ERR_PRE)/SamplingTime * Y_inner_Dgain;
        YAW_RATE_ERR_PRE = YAW_RATE_ERR;
        Limit_cut(&YAW_RATE_I,-100,100);

        YAW_ANG_ERR = -90- psi_feedback ; // 실험용 이라서 -90 임
        YAW_ANG_P = YAW_ANG_ERR * Y_ANG_Pgain;
        YAW_ANG_I = YAW_ANG_I + (YAW_ANG_ERR * Y_ANG_Igain) * SamplingTime;
        Limit_cut(&PITCH_ANG_I,-100,100);
        YAW_ANG_D = (YAW_ANG_ERR - YAW_ANG_ERR_PRE)/SamplingTime * Y_ANG_Dgain;
        YAW_ANG_ERR_PRE = YAW_ANG_ERR;
        yaw_cont_ANG = YAW_ANG_P + YAW_ANG_I + YAW_ANG_D;
         
         yaw_cont = YAW_RATE_P + YAW_RATE_I; 
       
        f1 = -pitch_cont- pitch_cont_ANG+ yaw_cont;
        f2 = pitch_cont + pitch_cont_ANG + yaw_cont;                       //제어 한 값들을 합쳐서 이제 서보로 내보낼 꺼야 ~
        f3 = -roll_cont -roll_cont_ANG + yaw_cont;
        f4 = roll_cont + roll_cont_ANG + yaw_cont ;
       
        Limit_cut(&f1,-300,300);                 
        Limit_cut(&f2,-300,300);                // Limit_cut 은 일종의 장벽을 만드는 것 
        Limit_cut(&f3,-300,300);
        Limit_cut(&f4,-300,300);

        // transform f value to PWM duty ratio 
        p1=(int)(-f1)+1570;            //     P1 1630  R1 1570
        p2=(int)(-f2)+1600;            //     P2 1600  R2 1600                                                     // 정수로 바꾸는 중 ...
        p3=(int)(-f3)+1530;            //     P3 1550  R3 1550
        p4=(int)(-f4)+1600;            //     P4 1600  R4 1570

        val1=map(p1,1300,1900,1,90);
        val2=map(p2,1300,1900,1,90);
        val3=map(p3,1300,1900,1,90);                                                     // map 은 1290 ~1890 을 1~90 에 맡게 다운스케일링 해주는 것 
        val4=map(p4,1300,1900,1,90);

        fin1_servo.write(val1);
        fin2_servo.write(val2);                                                          //write 는 값을 전선을 통해 서보로 내보내는 것
        fin3_servo.write(val3);
        fin4_servo.write(val4);
        
      /*  Serial.print("\n");
        Serial.print("PITCH_ANG_ERR :");Serial.print(PITCH_ANG_ERR);Serial.print("     ");   Serial.print("YAW_ANG_ERR :");Serial.print(YAW_ANG_ERR);Serial.print("     ");    Serial.print("ROLL_ANG_ERR: ");Serial.print(ROLL_ANG_ERR);  
        Serial.print("\n");
        Serial.print("PITCH_RATE_ERR :");Serial.print(PITCH_RATE_ERR);Serial.print("     ");       Serial.print("ROLL_RATE_ERR : ");Serial.print(ROLL_RATE_ERR);    // 이거는 pid 제어된 값들이 어떻게 나오나 보려고 했어 시리얼 통신으로
        Serial.print("\n");*/

         Serial.print("\n");
        Serial.print("f1");Serial.print(f1);Serial.print("     ");   Serial.print("f2");Serial.print(f2);Serial.print("     ");    Serial.print("f3 ");Serial.print(f3);  
        Serial.print("\n");
        Serial.print("f4");Serial.print(f4);Serial.print("     ");      // 이거는 pid 제어된 값들이 어떻게 나오나 보려고 했어 시리얼 통신으로
        Serial.print("\n");


        
        
        
        
        
        
        if (sequence > 162 && (PITCH_ANG_ERR > 15 || ROLL_ANG_ERR >15 || PITCH_ANG_ERR < -15 || ROLL_ANG_ERR <-15)) //
        //if (count > 600 && (PITCH_ANG_ERR > 35 || ROLL_ANG_ERR >35 || PITCH_ANG_ERR < -35 || ROLL_ANG_ERR <-35)) //600 이 37초 !!  // 162 가 10초 !!
        { 
           para_servo.write(100);
        }
        
        else if (PITCH_ANG_ERR > 45 || ROLL_ANG_ERR > 45 || PITCH_ANG_ERR < -45 || ROLL_ANG_ERR <-45)
        { 
           para_servo.write(100);
        }
        
        else
        {
          para_servo.write(2);
        }

/////////////////////////////////////////////////////////////
   /*  if (uno1.available())
      {
       delay(10);
      char a=uno1.read();
      Serial.print(a);
     
      

            if (a=='e')//(a==101) //e,사출 실험
             {    Serial.print("yes");
               para_servo.write(40); // 낙하산 사출
               delay(500);
               para_servo.write(100); //
               delay(500);
               para_servo.write(30);
               delay(1000);
             }

          else if(a=='s')//(a==115) // 115="s" 시퀀스 전개
               {
                parachute_STATE=1;
             
               delay(1);
               }
                else
               { 
             parachute_STATE=0;
               }
      } 
     
     else
     {
         
     }
   */
   
   
   ///////////////////////////시퀀스 /////////////////////
   if (parachute_STATE==1)
        {
             
             sequence++;
          
        }
       else{
              
            }

    ///////////////////////////자이로 - 에어브레이크 연동 ///////////////////////
     if (PITCH_ANG_ERR > 45 || ROLL_ANG_ERR > 45 || PITCH_ANG_ERR < -45 || ROLL_ANG_ERR <-45)
        { 
           char b='A';
           nano1.write(b);
        }
        
        
  
}




void Limit_cut(float *ff, long int MIN_LIMIT, long int MAX_LIMIT)  // pid 제어 루프를 돌다보면 값이 천정부지로 치솟을 수 도 있는데 그걸 방지 하는 거야 특정 구간안에서 루프를 돌도록 일단 내 코딩에는 그런증상은 없어서 잠깐 빼둠
{
    if(*ff < MIN_LIMIT)
    {
        *ff = MIN_LIMIT;
    }
    else if(*ff > MAX_LIMIT)
    {
        *ff = MAX_LIMIT;
    }
} 


void print_3d_float(const char* tag_x, const char* tag_y, const char* tag_z, float x, float y, float z) /*float 를 쓰는 이유는 소수라서 그런거고 4 는 소수점 4자리에서 끊겠다는 것임, cont char 에는 MX , ROLL 과 같은 문자가 들어감 , tag 는 태그 한다는 의미*/
{
  Serial.print(tag_x); Serial.print("("); Serial.print(x, 4); Serial.print("), "); /**/ // 이것도 시리얼 통신으로 볼려고 해논거 ~~
  Serial.print(tag_y); Serial.print("("); Serial.print(y, 4); Serial.print("), "); 
  Serial.print(tag_z); Serial.print("("); Serial.print(z, 4); Serial.print("), ");  
}


