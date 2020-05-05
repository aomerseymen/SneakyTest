/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
  /**
   * Creates a new LiftSubsystem.
   */
  private final VictorSP liftMotor_1 = new VictorSP(LiftConstants.liftMotorPort_1);
  private final VictorSP liftMotor_2 = new VictorSP(LiftConstants.liftMotorPort_2);
  private final SpeedControllerGroup liftMotors =  new SpeedControllerGroup(liftMotor_1, liftMotor_2);
  private final DigitalInput topLimitSwitch = new DigitalInput(LiftConstants.topLimitSwitchPort);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(LiftConstants.bottomLimitSwitchPort);
  private final Encoder liftEncoder = new Encoder(LiftConstants.liftEncoder_A,LiftConstants.liftEncoder_B,false,EncodingType.k4X);
  public LiftSubsystem() {
    liftEncoder.setDistancePerPulse(1.0/2048.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getLiftHeight(){
    return liftEncoder.getDistance();
  }

  public boolean getTopSwitch(){
    return topLimitSwitch.get();
  }
  public boolean getBottomSwitch(){
    return bottomLimitSwitch.get();
  }

  public void runLift(double speed){
    liftMotors.set(speed);
  }
  
  public void stopLift(){
    liftMotors.set(0);
  }
}
