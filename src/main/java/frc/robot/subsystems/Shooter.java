/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static VictorSP shooterMotor_1 = new VictorSP(ShooterConstants.shooterMotorPort_1);
  private static VictorSP shooterMotor_2 = new VictorSP(ShooterConstants.shooterMotorPort_2);

  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(double speed) {
    shooterMotor_1.set(speed);
    shooterMotor_2.set(-speed);
  }

  public void stopShooter() {
    shooterMotor_1.set(0);
    shooterMotor_2.set(0);
  }
}
