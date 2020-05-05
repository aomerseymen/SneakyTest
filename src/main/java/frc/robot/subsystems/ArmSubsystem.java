/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /**
   * Creates a new ArmSubsystem.
   */
  private final VictorSP armMotor =  new VictorSP(ArmConstants.armMotorPort);
  public final AnalogPotentiometer armPot = new AnalogPotentiometer(ArmConstants.armPotPort,270,5);
  public ArmSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runArm(double speed){
    armMotor.set(speed);
  }

  public void stopArm(){
    armMotor.set(0);
  }

  public double getAngle(){
    return armPot.get();
  }
}
