package frc.robot.subsystems;


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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
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


  public DriveTrain() {

  }

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



    @Override
    public void periodic() {

      // This method will be called once per scheduler run
      left_m.setInverted(true);
      right_m.setInverted(true);
    
      pose = odometry.update(getHeading(),encoderLeft.getDistance() / 7.39 * 2 * Math.PI * Units.inchesToMeters(3) / 60,
       encoderRight.getDistance() / 7.39 * 2 * Math.PI * Units.inchesToMeters(3) / 60);
    }
  
    public void reset() {
      odometry.resetPosition(new Pose2d(), getHeading());
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
}
