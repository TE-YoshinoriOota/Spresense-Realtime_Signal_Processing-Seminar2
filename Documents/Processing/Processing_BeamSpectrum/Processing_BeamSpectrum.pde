import java.util.Date;
import java.text.SimpleDateFormat;
import processing.serial.*;

Serial myPort;
float centerX, centerY;
float maxRadius;
float [] angles = new float[40];
float [] values = new float[40];
boolean bRecording = false;
int index = 0;

final String SERIAL_PORTNAME = "COM12";
final int    SERIAL_BAUDRATE = 115200;

void drawGraph() {
  translate(centerX, centerY);
  background(color(0, 22, 55));
  strokeWeight(1);
  stroke(0, 255, 0, 50);
  for (int i = 0; i < 360; i+=10) {
    line(0, 0, maxRadius*cos(radians(i)), maxRadius*sin(radians(i)));
  }
  stroke(0, 255, 0, 50);  
  noFill();
  for (int i = 0; i <= (int)maxRadius; i += 30) {
    ellipse(0, 0, i*2, i*2);
  }
  strokeWeight(3);
  line(-maxRadius, 0, maxRadius, 0);
  line(0, maxRadius, 0, -maxRadius);
  
}

void setup() {
  size(400, 400);
  centerX = width/2;
  centerY = height/2;
  maxRadius = min(width, height)/2 -20;
  drawGraph();
  myPort = new Serial(this, SERIAL_PORTNAME, SERIAL_BAUDRATE);
}

void draw() {

  if (bRecording != false) return;
  drawGraph();
  stroke(255, 255, 0);
  float zero_x, zero_y, last_x, last_y;
  zero_x = values[0] * maxRadius * cos(radians(angles[0]-90));
  zero_y = values[0] * maxRadius * sin(radians(angles[0]-90));
  
  last_x = zero_x;
  last_y = zero_y;
  for (int i = 1; i < 10; ++i) {
    float x = values[i] * maxRadius * cos(radians(angles[i]-90));
    float y = values[i] * maxRadius * sin(radians(angles[i]-90));
    line(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
  }
  
  last_x = values[27] * maxRadius * cos(radians(angles[27]-90));
  last_y = values[27] * maxRadius * sin(radians(angles[27]-90));
  for (int i = 28; i < 36; ++i) {
    float x = values[i] * maxRadius * cos(radians(angles[i]-90));
    float y = values[i] * maxRadius * sin(radians(angles[i]-90));
    line(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
    if (i == 35) line(x, y, zero_x, zero_y);     
  }
  delay(100);
}

void serialEvent(Serial p) {
  String incomingData;  
  incomingData = myPort.readStringUntil('\n');   
  incomingData = trim(incomingData);  
  if (incomingData == null) return;
  
  // check sync word
  if (bRecording == false) {
    if (incomingData.equals("SPRS"))  {
      println("found sync");
      bRecording = true;
      return;
    } else {    
      return;
    }
  }
  
  // storing data
  String[] invalues = split(incomingData, ',');
  if (invalues.length == 2) {
    float angle = float(invalues[0]);
    float value = float(invalues[1]);
    angles[index] = angle;
    values[index] = value;
    println("[" + index + "] " + angle + ", " + value);
  }
  
  if (++index == 36) {
    index = 0; bRecording = false;
  }
}
