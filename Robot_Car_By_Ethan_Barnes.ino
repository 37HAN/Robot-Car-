#include <Servo.h>    //servo library

Servo lServo;         //left servo motor
Servo rServo; 

int motorSpeedA = 1500; 
int motorSpeedB = 1500;

const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  11;      // the number of the LED pin
int buttonState = 0;         // variable for reading the pushbutton status

void setup() { 

  pinMode(ledPin, OUTPUT); // initialize the LED pin as an output:
  pinMode(buttonPin, INPUT); // initialize the pushbutton pin as an input:
  
  lServo.attach(9);
  rServo.attach(10);
     
  } 
  
  void loop() { 
    
    int xAxis = analogRead(A0); // Read Joysticks X-axis 
    int yAxis = analogRead(A1); // Read Joysticks Y-axis 
    
    //Y-axis used for forward and backward control 
    
    if (yAxis < 470) { 
           
      //Convert the declining Y-axis readings for going backward from 470 to 0 into 1480 to 1200 value for the PWM signal for increasing the motor speed 
      
      motorSpeedA =map(yAxis, 470, 0, 1480, 1200); 
      motorSpeedB = map(yAxis,470, 0, 1480, 1200); 

      lServo.writeMicroseconds(motorSpeedA); // Send PWM signal to motor A      Set Motor A backward 
      rServo.writeMicroseconds(motorSpeedB); // SendPWM signal to motor B       Set Motor B backward

     } 
     
     else if (yAxis > 550) {                
             
      // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
       
      motorSpeedA = map(yAxis, 550, 1023, 1520, 2000);  // Set Motor A forward
      motorSpeedB = map(yAxis, 550, 1023, 1520, 2000);  // Set Motor B forward

      } 
      
      // If joystick stays in middle the motors are not moving 
      
      else { 
        
        motorSpeedA = 1500; // Set Motor A at stop 
        motorSpeedB = 1500; // Set Motor B at stop
        
        } 
  
        // X-axis used for left and right control 
        
        if (xAxis < 470) { 
          
         // Convert the declining X-axis readings from 470 to 0 into increasing 1520 to 1800 value 
         
         int xMapped = map(xAxis, 470, 0, 1520,1800); 
         
         // Move to left - decrease left motor speed,Increase right motor speed 
         
         motorSpeedA = motorSpeedA - xMapped; 
         motorSpeedB = motorSpeedB + xMapped; 
         
         // Confine the range from 1520 to 1800 
         
         if (motorSpeedA < 1520) { 
          
          motorSpeedA = 1520; 
          
          } 
          
          if (motorSpeedB > 1800) { 
            
            motorSpeedB = 1800; 
            
            } 
            
           } 
           
           else if (xAxis > 550) { 
            
            // Convert the declining X-axis readings from 550 to 1023 into increasing 1520 to 1800 value 
            
            int xMapped = map(xAxis, 550, 1023, 1520, 1800); 
            
            
            // Move right - decrease right motor speed, increase left motor speed 
            
            motorSpeedA = motorSpeedA + xMapped; 
            motorSpeedB = motorSpeedB - xMapped; 
            
            // Confine the range from 1520 to 1800 
            
            if (motorSpeedA > 1800) {
               
              motorSpeedA = 1800;
              
              } 
              
              if (motorSpeedB < 1520) { 
                
                motorSpeedB = 1520;
                 
                }
                 
               }

                
               
               // Prevent buzzing at low speeds  
               
               if (motorSpeedA < 70) { 
                motorSpeedA = 1500; 
                } 
                
                if (motorSpeedB < 70) { 
                  motorSpeedB = 1500;
                  }
              
              lServo.writeMicroseconds(motorSpeedA); // Send PWM signal to motor A 
              rServo.writeMicroseconds(motorSpeedB); // SendPWM signal to motor B 

              
              buttonState = digitalRead(buttonPin); // read the state of the pushbutton value:

              // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
              if (buttonState == HIGH) {
    
              digitalWrite(ledPin, HIGH); // turn LED on:
              
              } 
              
              else {
              
              digitalWrite(ledPin, LOW);// turn LED off:
              }

        }
