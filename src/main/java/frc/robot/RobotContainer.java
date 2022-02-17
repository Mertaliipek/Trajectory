// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.ObjectInputFilter.Config;
import java.lang.reflect.Array;
import java.util.Arrays;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveTrain driveTrain = new DriveTrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(Units.inchesToMeters(3),Units.inchesToMeters(3));    
    config.setKinematics(driveTrain.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d()),
          new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))),
      config
  );

    /*RamseteCommand command = new RamseteCommand(trajectory, driveTrain.getPose(), new RamseteController(2,.7), driveTrain.returnFeedforward(), 
    driveTrain.getKinematics(), driveTrain.getSpeeds(), driveTrain.getLeftPidController(), driveTrain.getRightPidController(),
     driveTrain.setOutputVolts(1, 2), driveTrain  )

     return command.andThen(() -> drive.setOutputVolts(0, 0)); */

     RamseteCommand command = new RamseteCommand(
      trajectory,
      driveTrain::getPose,
      new RamseteController(2, .7),
      driveTrain.returnFeedforward(),
      driveTrain.getKinematics(),
      driveTrain::getSpeeds,
      driveTrain.getLeftPidController(),
      driveTrain.getRightPidController(),
      driveTrain::setOutputVolts,
      driveTrain
  );



    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
