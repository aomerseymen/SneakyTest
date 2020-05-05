/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class RunLift extends CommandBase {
  /**
   * Creates a new RunLift.
   */
  private final LiftSubsystem m_lift;
  private final double m_speed;
  public RunLift(LiftSubsystem lift,double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_lift =  lift;
    this.m_speed = speed;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.runLift(m_speed);
    System.out.println("Lift yüksekliği : " + m_lift.getLiftHeight());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_speed>0){
      return m_lift.getTopSwitch();
    }
    else{
      return m_lift.getBottomSwitch();
    }
  }
}
