// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.trobot5013lib.led.TrobotAddressableLED;

public class LEDs extends SubsystemBase {
  int ledLength = 90;
  int ledPWMPort = 3;
  
  TrobotAddressableLED m_ledStrip = new TrobotAddressableLED(ledPWMPort, ledLength);
  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(ledLength);
  
  /** Creates a new LEDs. */
  public LEDs(RobotContainer robotContainer) {
    super();
    m_RobotContainer = robotContainer;
  }

  private void ledsOn() {
    
  }
  
  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
