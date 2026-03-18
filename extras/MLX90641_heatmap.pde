// MLX90641_Heatmap.pde
// Description: This sketch generates a colour heatmap with small control panel for the MLX90641 16x12 IR sensor.
// Author: D. Dubins with assitance from Perplexity.AI
// Date: 25-Feb-26
// Last Updated: 18-Mar-26
// Simple 16x12 heat map for MLX90641 serial output
// Expects lines: Tamb, p0, p1, ... p191 (comma-separated)
// Match port + baud (115200) to your serial port settings
// Libraries: ControlP5 v 2.2.6, by Andreas Schlegal
// (tutorial here: https://www.kasperkamperman.com/blog/processing-code/controlp5-library-example1/)
// Note: This is not a sketch for the Arduino IDE. It was written for the Processing environment, available here: https://processing.org/

import processing.serial.*;
import controlP5.*;    // import controlP5 library

// Adjust COM port settings here
Serial myPort; // for communications port
int portNum = 2; // index for COM port number - update to open the correct serial port.
int portSpeed = 115200;    // COM port baud rate in bps

final int PIXELS=192;  // number of pixels in the array (needs to be ROWSxCOLS)
PFont boldFont;        // declare a bold font for the control window header
PFont smallFont;       // declare small font to display sensor temperature

float fontScale=1.0/displayDensity();   // scaling factor to adjust font sizes for different resolution screens

ControlP5 cp5; // controlP5 object called cp5

float[] pixels = new float[PIXELS];
boolean haveFrame = false;

// grid / window settings
int COLS = 16;
int ROWS = 12;
int cellSize = 50;         // pixel size of each cell: use this to adjust window size
int margin = 5;            // outer margin

// value range for colour mapping (adjust to your environment)
float minTemp = 20;        // cold colour at/below this (default: 20)
float maxTemp = 35;        // hot colour at/above this (default: 35)
float Tamb = 0.0;          // to hold sensor temperature
float Tavg = 0.0;          // to hold average temperature
float Tmin = 0.0;          // to hold average temperature
float Tmax = 0.0;          // to hold average temperature

// second window definition
SecondWindow win;
// array to store 7 colours that can be changed by the different
// user interface elements
boolean invert_x=false;    // for x-invert state
boolean invert_y=false;    // for y-invert state
boolean pause=false;       // for pause button state
boolean softer=false;      // for softer colours
boolean hide_vals=false;   // for hiding temperature values button state
boolean invert_heat=false; // for inverting the heat map (cold=red, hot=blue)
float offsetval=0.0;       // for temperature offset slider

void settings() {
  size(COLS * cellSize + margin * 2, ROWS * cellSize + margin * 2);
  pixelDensity(displayDensity()); // ensures proper rendering on high-DPI displays
}

void setup() {
  // List available ports in the console
  println("Available Ports:");
  println("----------------");
  int numPorts=Serial.list().length;
  for (int i=0; i<numPorts; i++) {
    String pInfo="["+i+"] "+Serial.list()[i];
    if (i==portNum) {
      pInfo+=" <- PORT SELECTED";
    }
    println(pInfo);
  }
  if (portNum < 0 || portNum > numPorts) {
    println("Invalid port number. Exiting sketch.");
    exit(); // leave the sketch
  }
  // Now open the port. Pick the right index from the printed list
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

  // Draw ROWSxCOLS heatmap

  int xadj, yadj;
  float total=0.0;   // reset the average
  float Tlocmin=200.0;  // unrealistically high number
  float Tlocmax=-200.0; // unrealistically low number

  for (int y = 0; y < ROWS; y++) {
    for (int x = 0; x < COLS; x++) {
      xadj=(invert_x)?COLS-x-1:x;
      yadj=(invert_y)?ROWS-y-1:y;

      int idx = yadj * COLS + xadj;    // linear index 0..191
      float t = pixels[idx];
      total = total + t;
      if (t>Tlocmax)Tlocmax=t;
      if (t<Tlocmin)Tlocmin=t;
      // Clamp to [minTemp, maxTemp]
      float tt = constrain(t, minTemp, maxTemp);
      float frac = map(tt, minTemp, maxTemp, 0, 1);
      if(invert_heat)frac = 1-frac;
      float r, g, b;

      if (softer) {  // softer colours
        //soft red: (238. 105, 112)
        //white: (255,255,255)
        //soft blue: (92, 138, 199)

        float coldfrac=2*frac;   // rescale fraction to cold colours
        float hotfrac=2*(frac-0.5); // rescale fraction to hot colours
        if (frac>0.5) {
          r = lerp(255, 238, hotfrac);
          g = lerp(255, 105, hotfrac);
          b = lerp(255, 112, hotfrac);
        } else {
          r = lerp(92, 255, coldfrac);
          g = lerp(138, 255, coldfrac);
          b = lerp(199, 255, coldfrac);
        }
      } else {
        //red: (255, 0, 0)
        //blue: (0, 0, 255)
        r = lerp(0, 255, frac);
        g = 0;
        b = lerp(255, 0, frac);
      }

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

      // Adjust significant digits here
      if (!hide_vals) {
        String temp1 = nf(t, 0, 1); // 1 significant digit (e.g. "99.9")
        String label;
        if (temp1.length() <= 4) {  // up to twi digits + decimal + one decimal place
          label = temp1;            // keep the decimal
        } else {                    // for 3 digits, drop decimal for display purposes
          label = nf(t, 0, 0);      // switch to integer
          int ti = round(t);
          label = nf(ti, 0);
        }
        text(label, x0 + cellSize / 2.0, y0 + cellSize / 2.0);
      }
    }
  }
  Tavg=total/(float)PIXELS; // calculate average
  Tmax=Tlocmax; // calculate Tmax
  Tmin=Tlocmin; // calculate Tmin
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
    size(235, 195); // width, height
    pixelDensity(displayDensity()); // ensures proper rendering on high-DPI displays
  }

  public void setup() {
    surface.setTitle("Control Window");
    surface.setLocation(0, 0);
    cp5 = new ControlP5(this);

    // define a toggle button for pausing the heat map
    cp5.addToggle("Pause")
      .setPosition(20, 75)
      .setSize(60, 20)
      .getCaptionLabel()
      .setText("Pause").toUpperCase(false)
      .setFont(createFont("Arial Bold", 12*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a toggle button for softer colours
    cp5.addToggle("Soft")
      .setPosition(90, 125)
      .setSize(60, 20)
      .getCaptionLabel()
      .setText("Soft").toUpperCase(false)
      .setFont(createFont("Arial Bold", 12*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a toggle button for inverting in the x-direction
    cp5.addToggle("Invert_x")
      .setPosition(90, 75)
      .setSize(60, 20)
      .getCaptionLabel()
      .setText("Invert X").toUpperCase(false)
      .setFont(createFont("Arial Bold", 12*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a toggle button for inverting in the y-direction
    cp5.addToggle("Invert_y")
      .setPosition(160, 75)
      .setSize(60, 20)
      .getCaptionLabel()
      .setText("Invert Y").toUpperCase(false)
      .setFont(createFont("Arial Bold", 12*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a toggle button for toggling number display
    cp5.addButton("Auto_Scale")
      .setPosition(20, 100)
      .setSize(60, 20)
      .getCaptionLabel()
      .setText("Auto Scale").toUpperCase(false)
      .setFont(createFont("Arial Bold", 10*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a toggle button for inverting in the x-direction
    cp5.addToggle("Hide_Vals")
      .setPosition(90, 100)
      .setSize(60, 20)
      .getCaptionLabel()
      .setText("Hide Vals").toUpperCase(false)
      .setFont(createFont("Arial Bold", 10*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a toggle button for inverting in the y-direction
    cp5.addToggle("Invert_Heat")
      .setPosition(160, 100)
      .setSize(60, 20)
      .getCaptionLabel()
      .setText("Invert Heat").toUpperCase(false)
      .setFont(createFont("Arial Bold", 10*fontScale))
      .align(ControlP5.CENTER, ControlP5.CENTER);

    // define a slider button for the lower colour temperature (blue)
    cp5.addSlider("min_T")
      .setPosition(25, 35) // xpos, ypos
      .setSize(140, 10) // width, height
      .setRange(0, 30)  // min/max range
      .setValue(20)     // manually set default value of slider here
      .setDecimalPrecision(1)
      .setNumberOfTickMarks(61)
      .showTickMarks(false)
      .setCaptionLabel("Min °C") // label text
      .setFont(createFont("Arial Bold", 12*fontScale));

    // define a slider button for the higher colour temperature (red)
    cp5.addSlider("max_T")
      .setPosition(25, 55) // xpos, ypos
      .setSize(140, 10) // width, height
      .setRange(20, 50) // min/max range
      .setValue(35)     // manually set default value of slider here
      .setDecimalPrecision(1)
      .setNumberOfTickMarks(61)
      .showTickMarks(false)
      .setCaptionLabel("Max °C") // label text
      .setFont(createFont("Arial Bold", 12*fontScale));

    // define a numberbox for the temperature offset
    cp5.addNumberbox("Offset")
      .setPosition(160, 153)
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
    text("Average:", 25, 165);
    text("Sensor:", 25, 185);
    text("°C", 128, 165);
    text("°C", 128, 185);
    fill(220, 220, 220); // lighter grey
    String label = nf(Tavg, 0, 1);     // Show 1 decimal place; adjust as desired

    text(label, 90, 165);
    label = nf(Tamb, 0, 1);
    text(label, 90, 185);
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

      if (theEvent.getController().getName()=="Soft") {
        softer=!softer;
      }
      
      if (theEvent.getController().getName()=="Auto_Scale") {
        minTemp = Tmin - 2;
        maxTemp = Tmax + 2;
        // update sliders
        cp5.getController("min_T").setValue(minTemp);
        cp5.getController("max_T").setValue(maxTemp);
      }

      if (theEvent.getController().getName()=="Hide_Vals") {
        hide_vals=!hide_vals;
      }

      if (theEvent.getController().getName()=="Invert_Heat") {
        invert_heat=!invert_heat;
      }
    }
  }
}
