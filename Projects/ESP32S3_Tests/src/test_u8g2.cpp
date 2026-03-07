#include <Arduino.h>
#include <U8g2lib.h>

#define MASTER_SDA 4
#define MASTER_SCL 5

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R2, MASTER_SCL, MASTER_SDA, U8X8_PIN_NONE);

// Cube vertices (3D coordinates)
float points[8][3] = {
  {-10, -10,  10}, {10, -10,  10}, {10,  10,  10}, {-10,  10,  10},
  {-10, -10, -10}, {10, -10, -10}, {10,  10, -10}, {-10,  10, -10}
};

float angle = 0;

void setup() {
  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();
  
  float rotated_points[8][2]; // Store 2D projections
  
  // Rotate and project points
  for (int i = 0; i < 8; i++) {
    float x = points[i][0];
    float y = points[i][1];
    float z = points[i][2];

    // Rotation math
    float rad = angle * PI / 180;
    // Rotate around Y axis
    float x_rot = x * cos(rad) + z * sin(rad);
    float z_rot = -x * sin(rad) + z * cos(rad);
    // Rotate around X axis
    float y_rot = y * cos(rad) - z_rot * sin(rad);
    z_rot = y * sin(rad) + z_rot * cos(rad);

    // Project to 2D screen (Center is 64, 32)
    rotated_points[i][0] = 64 + x_rot;
    rotated_points[i][1] = 32 + y_rot;
  }

  // Draw edges
  for (int i = 0; i < 4; i++) {
    u8g2.drawLine(rotated_points[i][0], rotated_points[i][1], rotated_points[(i+1)%4][0], rotated_points[(i+1)%4][1]);
    u8g2.drawLine(rotated_points[i+4][0], rotated_points[i+4][1], rotated_points[((i+1)%4)+4][0], rotated_points[((i+1)%4)+4][1]);
    u8g2.drawLine(rotated_points[i][0], rotated_points[i][1], rotated_points[i+4][0], rotated_points[i+4][1]);
  }

  u8g2.sendBuffer();
  angle += 5; // Speed of rotation
  if (angle >= 360) angle = 0;
}