/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final VictorSP rightIntakeMotor = new VictorSP(IntakeConstants.rightIntakeMotorPort);
  private final VictorSP leftIntakeMotor = new VictorSP(IntakeConstants.leftIntakeMotorPort);

  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {
    rightIntakeMotor.set(speed);
    leftIntakeMotor.set(-speed);
  }

  public void stopIntake() {
    rightIntakeMotor.set(0);
    leftIntakeMotor.set(0);
  }

}
