// MLX90641_heatmap.pde
// Description: This sketch generates a colour heatmap with small control panel for the MLX90641 16x12 IR sensor.
// It works in conjunction with the file https://github.com/dndubins/MLX90641/blob/main/examples/MLX90641_processing/MLX90641_processing.ino
// Author: D. Dubins with assitance from Perplexity.AI
// Date: 11-Dec-25
// Simple 16x12 heat map for MLX90641 serial output
// Expects lines: Tth, p0, p1, ... p191 (comma-separated)
// Match port + baud (9600) to your serial port settings
// Libraries: ControlP5 v 2.2.6, by Andreas Schlegal
// (tutorial here: https://www.kasperkamperman.com/blog/processing-code/controlp5-library-example1/)
// Note: This is not a sketch for the Arduino IDE. It was written for the Processing environment, available here: https://processing.org/

import processing.serial.*;
import controlP5.*;    // import controlP5 library

final int PIXELS=192;  // number of pixels in the array
PFont boldFont;        // declare a bold font for the control window header
PFont smallFont;       // declare small font to display ambient temperature

float fontScale=1.0/displayDensity();   // scaling factor to adjust font sizes for different resolution screens

ControlP5 cp5; // controlP5 object called cp5

// Adjust COM port settings here
Serial myPort; // for communications port
int portNum = 0; // index for COM port number - update to open the correct serial port.
int portSpeed = 115200; // COM port baud rate in bps

float[] pixels = new float[PIXELS];
boolean haveFrame = false;

// grid / window settings
int cols = 16;
int rows = 12;
int cellSize = 60;         // pixel size of each cell
int margin = 20;           // outer margin

// value range for color mapping (adjust to your environment)
float minTemp = 15;        // cold color at/below this (default: 20)
float maxTemp = 30;        // hot color at/above this (default: 35)
float Tamb = 0.0;          // to hold ambient temperature
float Tavg = 0.0;          // to hold average temperature

// second window definition
SecondWindow win;
// array to store 7 colors that can be changed by the different
// user interface elements
boolean invert_x=false; // for x-invert state
boolean invert_y=false; // for y-invert state
boolean pause=false; // for pause button state
float offsetval=0.0; // for temperature offset slider

void settings() {
  size(cols * cellSize + margin * 2, rows * cellSize + margin * 2);
  pixelDensity(displayDensity()); // ensures proper rendering on high-DPI displays
}

void setup() {
  // List available ports in the console
  println(Serial.list());

  // Pick the right index from the printed list
  String portName = Serial.list()[portNum];   // change to correct index of COM port as needed
  myPort = new Serial(this, portName, portSpeed);

  // Read one line at a time
  myPort.bufferUntil('\n');

  textAlign(CENTER, CENTER);
  textSize(14);
  win = new SecondWindow();

  // Create bold font (Calibri Bold, size 16)
  boldFont = createFont("Calibri Bold", 17);
  // Create small font (Calibri Bold, size 16)
  smallFont = createFont("Calibri Bold", 15);
}

void draw() {
  background(0);

  if (!haveFrame) {
    // Show simple message until first valid frame
    fill(255);
    text("Waiting for data...", width/2, height/2);
    return;
  }

  // Draw 8x8 heatmap

  int xadj, yadj;
  float total=0.0; // reset the average
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      xadj=(invert_x)?cols-x-1:x;
      yadj=(invert_y)?rows-y-1:y;

      int idx = yadj * cols + xadj;    // linear index 0..191
      float t = pixels[idx];
      total = total + t;

      // Clamp to [minTemp, maxTemp]
      float tt = constrain(t, minTemp, maxTemp);
      float frac = map(tt, minTemp, maxTemp, 0, 1);

      // Simple blue->red gradient
      // cold: blue (0,0,255), hot: red (255,0,0)
      float r = lerp(0, 255, frac);
      float g = 0;
      float b = lerp(255, 0, frac);

      int x0 = margin + x * cellSize;
      int y0 = margin + y * cellSize;

      noStroke();
      fill(r, g, b);
      rect(x0, y0, cellSize, cellSize);

      // Draw temperature text in white or black depending on background
      float brightness = (r + g + b) / 3.0;
      if (brightness < 128) {
        fill(255);
      } else {
        fill(0);
      }

      // Show 1 decimal place; adjust as desired
      String label = nf(t, 0, 1);
      text(label, x0 + cellSize / 2.0, y0 + cellSize / 2.0);
    }
  }
  Tavg=total/(float)PIXELS;
}

// Called automatically when a '\n' is received
void serialEvent(Serial s) {
  String line = trim(s.readStringUntil('\n'));
  if (line == null || line.length() == 0) {
    return;
  }

  // Split on commas
  String[] parts = split(line, ',');

  // Expect 1 + 192 = 193 values (thermistor + 192 pixels)
  if (parts.length < PIXELS+1) {
    // Not a full frame; ignore
    println("Short line, len = " + parts.length + ": " + line);
    return;
  }

  // Parse pixel temperatures (skip index 0 which is thermistor)
  if (!pause) {
    Tamb=parseFloat(parts[0]) + offsetval;
    for (int i = 0; i < PIXELS; i++) {
      pixels[i] = parseFloat(parts[i + 1]) + offsetval;
    }
  }
  haveFrame = true;
}

public class SecondWindow extends PApplet {

  public SecondWindow() {
    // Launch the second window
    PApplet.runSketch(new String[] { this.getClass().getSimpleName() }, this);
  }

  public void settings() {
    size(235, 155); // width, height
    pixelDensity(displayDensity()); // ensures proper rendering on high-DPI displays
  }

  public void setup() {
    surface.setTitle("Control Window");
    surface.setLocation(0, 0);
    cp5 = new ControlP5(this);

    // define a toggle button for pausing the heat map
    cp5.addToggle("Pause")
      .setPosition(20, 80)
      .setSize(60, 30)
      .getCaptionLabel()
      .setText("Pause").toUpperCase(false)
      .setFont(createFont("Arial Bold", 12*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a toggle button for inverting in the x-direction
    cp5.addToggle("Invert_x")
      .setPosition(90, 80)
      .setSize(60, 30)
      .getCaptionLabel()
      .setText("Invert X").toUpperCase(false)
      .setFont(createFont("Arial Bold", 12*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a toggle button for inverting in the y-direction
    cp5.addToggle("Invert_y")
      .setPosition(160, 80)
      .setSize(60, 30)
      .getCaptionLabel()
      .setText("Invert Y").toUpperCase(false)
      .setFont(createFont("Arial Bold", 12*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a slider button for the lower colour temperature (blue)
    cp5.addSlider("min_T")
      .setPosition(25, 35) // xpos, ypos
      .setSize(140, 10) // width, height
      .setRange(0, 30) // min/max range
      .setValue(15)   // default value of slider
      .setDecimalPrecision(1)
      .setNumberOfTickMarks(61)
      .showTickMarks(false)
      .setCaptionLabel("Min °C") // label text
      .setFont(createFont("Arial Bold", 12*fontScale));

    // define a slider button for the lower colour temperature (blue)
    cp5.addSlider("max_T")
      .setPosition(25, 55) // xpos, ypos
      .setSize(140, 10) // width, height
      .setRange(20, 50) // min/max range
      .setValue(30)   // default value of slider
      .setDecimalPrecision(1)
      .setNumberOfTickMarks(61)
      .showTickMarks(false)
      .setCaptionLabel("Max °C") // label text
      .setFont(createFont("Arial Bold", 12*fontScale));

    // define a numberbox for the temperature offset
    cp5.addNumberbox("Offset")
      .setPosition(160, 120)
      .setSize(60, 14)
      .setRange(-10.0, 10.0)
      .setValue(0.0)
      .setDirection(Controller.HORIZONTAL)  // drag left/right
      .setMultiplier(0.1)         // smaller multiplier => slower change
      .setScrollSensitivity(2)   // mouse wheel faster changes
      .setCaptionLabel("Offset °C") // label text
      .setDecimalPrecision(1)
      .setFont(createFont("Arial Bold", 12*fontScale));
  }

  public void draw() {
    background(100, 100, 100); // R, G, B channels
    fill(255);
    textFont(boldFont);         // set bold font
    text("Heat Map Image Control", 25, 20);
    textFont(smallFont);         // set bold font
    text("Average:", 25, 128);   
    text("Ambient:", 25, 148);
    text("°C", 128, 128);   
    text("°C", 128, 148);   
    fill(220,220,220); // lighter grey
    String label = nf(Tavg, 0, 1);     // Show 1 decimal place; adjust as desired
    text(label, 90, 128);
    label = nf(Tamb, 0, 1);
    text(label, 90, 148);
  }

  public void controlEvent(ControlEvent theEvent) {

    if (theEvent.isController()) {

      print("control event from : "+theEvent.getController().getName());
      println(", value : "+theEvent.getController().getValue());


      if (theEvent.getController().getName()=="min_T") {
        minTemp=theEvent.getController().getValue();
      }

      if (theEvent.getController().getName()=="max_T") {
        maxTemp=theEvent.getController().getValue();
      }

      if (theEvent.getController().getName()=="Offset") {
        offsetval = theEvent.getController().getValue();
      }

      if (theEvent.getController().getName()=="Invert_x") {
        invert_x=!invert_x;
      }

      if (theEvent.getController().getName()=="Invert_y") {
        invert_y=!invert_y;
      }

      if (theEvent.getController().getName()=="Pause") {
        pause=!pause;
      }
    }
  }
}
