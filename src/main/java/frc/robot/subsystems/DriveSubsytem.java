/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsytem extends SubsystemBase {
  /**
   * Creates a new DriveSubsytem.
   */
  private final Encoder driveEncoder = new Encoder(0, 1, false);

  private final VictorSP frontLeftMotor = new VictorSP(DriveConstants.frontLeftMotorPin);
  private final VictorSP frontRightMotor = new VictorSP(DriveConstants.frontRightMotorPin);
  private final VictorSP rearLeftMotor = new VictorSP(DriveConstants.rearLeftMotorPin);
  private final VictorSP rearRightMotor = new VictorSP(DriveConstants.rearRightMotorPin);

  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, rearRightMotor);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  
  private final Encoder leftWheelEncoder =  new Encoder(DriveConstants.lEncoderPortA,DriveConstants.lEncoderPortB);
  private final Encoder rightWheelEncoder =  new Encoder(DriveConstants.rEncoderPortA,DriveConstants.rEncoderPortB);

  private final DifferentialDriveOdometry m_odometry;

  public DriveSubsytem() {
    
    driveEncoder.setDistancePerPulse(15.24*Math.PI/2048);
    leftWheelEncoder.setDistancePerPulse(15.24*Math.PI/2048);
    rightWheelEncoder.setDistancePerPulse(15.24*Math.PI/2048);
    gyro.calibrate();
    
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftWheelEncoder.getDistance(),
                      rightWheelEncoder.getDistance());
  }
  
  public double getDistance(){
    return (leftWheelEncoder.getDistance()+rightWheelEncoder.getDistance())/2;
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot, true);
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  public double getHeadingReverse(){
    return Math.IEEEremainder(-1*gyro.getAngle(), 360);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftWheelEncoder.getRate(), rightWheelEncoder.getRate());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(-rightVolts);
    m_drive.feed();
  }


}
