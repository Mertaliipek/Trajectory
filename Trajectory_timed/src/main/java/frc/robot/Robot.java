// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
/**
 * 
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final Victor leftMaster = new Victor(0);
  private final Victor rightMaster = new Victor(1);
  private final Victor leftSlave = new Victor(2);
  private final Victor rightSlave = new Victor(3);
  
  private final MotorControllerGroup left_m
  = new MotorControllerGroup(leftMaster, leftSlave);
  private final MotorControllerGroup right_m
  = new MotorControllerGroup(rightMaster,rightSlave);  

  private final Encoder encoderLeft = new Encoder(0,1);
  private final Encoder encoderRight = new Encoder(8,9);

  private final DifferentialDrive motor = new DifferentialDrive(left_m,right_m);
  private final AnalogGyro gyro = new AnalogGyro(0);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(30)); //inch i metreye otomatik çeviriyor
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.264, 1.89,0.243);

  PIDController LeftpidController = new PIDController(1, 0,0);
  PIDController righpidController = new PIDController(1, 0, 0);
  Pose2d pose = new Pose2d();

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle()); //clockwise
  }

  public PIDController getLeftPidController() {
    return LeftpidController;
  }
  public PIDController getRightPidController() {
    return righpidController;
  }

  public SimpleMotorFeedforward returnFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
  public Pose2d getPose() {
    return pose;
  }
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
    encoderLeft.getDistance() / 7.39 * 2 * Math.PI * Units.inchesToMeters(3) / 60,
    encoderRight.getDistance() / 7.39 * 2 * Math.PI * Units.inchesToMeters(3) / 60
    );
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    left_m.setInverted(true);
    right_m.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    pose = odometry.update(getHeading(),encoderLeft.getDistance() / 7.39 * 2 * Math.PI * Units.inchesToMeters(3) / 60,
     encoderRight.getDistance() / 7.39 * 2 * Math.PI * Units.inchesToMeters(3) / 60);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    TrajectoryConfig config = new TrajectoryConfig(Units.inchesToMeters(3),Units.inchesToMeters(3));    
    config.setKinematics(getKinematics());

    //Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d(),config));

    /*RamseteCommand ramseteCommand = new RamseteCommand(
    driveTrain::getPose,
    new RamseteController(2,.7),
    driveTrain.returnFeedforward(),
    driveTrain.getKinematics(),
    driveTrain.getLeftPidController(),
    driveTrain.getRightPidController(),
    driveTrain::getSpeeds,

    driveTrain
    ); */

    //return ramseteCommand.andThen(() -> driveTrain.setOutputVolts(0, 0));
    
    // An ExampleCommand will run in autonomous
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
