// IMPORTANT: LCDWIKI_KBV LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.

//Set the pins to the correct ones for your development shield or breakout board.
//the 16bit mode only use in Mega.you must modify the mode in the file of lcd_mode.h
//when using the BREAKOUT BOARD only and using these 16 data lines to the LCD,
//pin usage as follow:
//             CS  CD  WR  RD  RST  D0  D1  D2  D3  D4  D5  D6  D7  D8  D9  D10  D11  D12  D13  D14  D15 
//Arduino Mega 40  38  39  /   41   37  36  35  34  33  32  31  30  22  23  24   25   26   27   28   29

//Remember to set the pins to suit your display module!

/**********************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**********************************************************************************/

#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_KBV.h> //Hardware-specific library

LCDWIKI_KBV mylcd(ILI9486,40,38,39,-1,41); //model,cs,cd,wr,rd,reset

//define some colour values
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 480

#define JOY_X A0
#define JOY_Y A1
#define JOY_SW 2

#define NUM_OF_INDICES 12

// Feel free to change these rotation speeds
float rotationXspeed = 0.09;
float rotationYspeed = 0.16;
float rotationZspeed = 0.3;

float cubeX = 0;
float cubeY = 0;
float cubeZ = 2;

int cubeOffset_x = SCREEN_WIDTH / 2;
int cubeOffset_y = SCREEN_HEIGHT / 2;
float cubeSize = 100;
float cameraDistance = 2;
float rotationX = 0;
float rotationY = 0;
float rotationZ = 0;

float vertices[][3] = { {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                      {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1} };

int indices[][2] = { {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7} };

void setup() {  
  Serial.begin(9600);
  mylcd.Init_LCD();
  mylcd.Fill_Screen(BLACK);

  pinMode(JOY_SW, INPUT);
  digitalWrite(JOY_SW, HIGH);
}

float* rotate(float& x, float& y, float angle)
{
  float s = sin(angle);
  float c = cos(angle);
  float* result = new float[2];
  result[0] = x * c - y * s;
  result[1] = y * c + x * s;
  return result;
}

void DrawLine(const float& x1, const float& y1, const float& x2, const float& y2, const int& color)
{
  mylcd.Set_Draw_color(color);
  mylcd.Draw_Line(x1, y1, x2, y2);
}

void loop() {
  float xMovement, yMovement, zMovement;      
  xMovement = -(analogRead(JOY_X) - 512);
  if (xMovement > -10 && xMovement < 10)
    xMovement = 0;
  yMovement = -(analogRead(JOY_Y) - 512);
  if (yMovement > -10 && yMovement < 10)
    yMovement = 0;
  zMovement = digitalRead(JOY_SW);  

  xMovement /= 64;
  yMovement /= 64;
  zMovement -= 1;
  zMovement *= 0.05;  

  //cubeX -= 10;
  //cubeY += 10;
  //cubeZ += zMovement;

  for (int n = 0; n < 2; n++)
  {
    int lineColor = n ? BLACK : GREEN;
    for (int i = 0; i < NUM_OF_INDICES; i++)
    {
      int* index = indices[i];
      int a = index[0];
      int b = index[1];
  
      float* vertex1 = vertices[a];
      float* vertex2 = vertices[b];
  
      float x1, y1, z1, x2, y2, z2;
      x1 = vertex1[0];
      y1 = vertex1[1];        
      z1 = vertex1[2];
  
      float* temp_Rotation = rotate(x1, z1, rotationY);
      x1 = temp_Rotation[0];
      z1 = temp_Rotation[1];
      delete temp_Rotation;
  
      temp_Rotation = rotate(y1, z1, rotationX);
      y1 = temp_Rotation[0];
      z1 = temp_Rotation[1];
      delete temp_Rotation;
  
      x1 = (float)(x1 * cubeSize);
      y1 = (float)(y1 * cubeSize);
      
      x2 = (float)vertex2[0];
      y2 = (float)vertex2[1];
      z2 = (float)vertex2[2];
  
      temp_Rotation = rotate(x2, z2, rotationY);
      x2 = temp_Rotation[0];
      z2 = temp_Rotation[1];
      delete temp_Rotation;
  
      temp_Rotation = rotate(y2, z2, rotationX);
      y2 = temp_Rotation[0];
      z2 = temp_Rotation[1];
      delete temp_Rotation;    
      
      x2 = (float)(x2 * cubeSize);
      y2 = (float)(y2 * cubeSize);            
  
      x1 += cubeX;
      x2 += cubeX;
      y1 += cubeY;
      y2 += cubeY;
      z1 += cubeZ;
      z2 += cubeZ;
  
      x1 /= z1 * cameraDistance;
      x2 /= z2 * cameraDistance;
      y1 /= z1 * cameraDistance;
      y2 /= z2 * cameraDistance;
  
      x1 += cubeOffset_x;
      x2 += cubeOffset_x;
      y1 += cubeOffset_y;
      y2 += cubeOffset_y;
              
      DrawLine(x1, y1, x2, y2, lineColor);
    }
  }

  rotationX += rotationXspeed;
  rotationY += rotationYspeed;
  rotationZ += rotationZspeed;
}
