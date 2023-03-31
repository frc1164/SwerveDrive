// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import javax.xml.stream.util.StreamReaderDelegate;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LED extends SubsystemBase {
  /** Creates a new LED. */
private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(48);
private AddressableLED m_led = new AddressableLED(LEDConstants.LEDport);

public enum ledMode {
  RED, GREEN, RAINBOW, TEAM, BLUE, PURPLE, YELLOW
}
  
public LED() {
  setLEDmode(ledMode.TEAM);
  LED_init();
}


public void setLEDmode (ledMode mode) {
  ledMode m_mode = mode;

switch (m_mode) {
  case RED: setRED(0,48);
  break;
  case GREEN: setGREEN(0, 48);
  break;
  case RAINBOW: rainbow(0, 48, 0, 0);
  break;
  case TEAM: setTEAM(0, 48);
  break;
  case BLUE: setBLUE(0, 48);
  break;
  case PURPLE: setPURPLE(0, 48);
  break;
  case YELLOW: setYELLOW(0, 48);
  break;
  default:break;
}
}

private void setRED(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 255, 0, 0);
  }
}

private void setGREEN(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 0, 255, 0);
  }
}

private void setBLUE(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 0, 0, 255);
  }
}

private void setPURPLE(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 135, 0, 211);
  }
}

private void setYELLOW(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 255, 255, 0);
  }
}

private void setTEAM(int startPos, int Length) {
  for (var i = startPos; i < 8; i++) {
    m_ledBuffer.setRGB(i, 135, 0, 211);
  }
  for (var i = 8; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 255, 20, 0);
  }
}

  private void rainbow(int startPos, int Length, double rainbowOffset, double hueModdifier) {
    // if (startPos + Length < m_ledBuffer.getLength()) {
      for (var i = startPos; i < startPos + Length; i++) {
        final int hue = (int) (((((rainbowOffset + i) * 180) / Length) + hueModdifier) % 180);
        m_ledBuffer.setHSV(i, (hue), 255, 128);
      }
    }

  public void LED_init() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
