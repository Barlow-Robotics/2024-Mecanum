// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/package-summary.html
// set​(TalonSRXControlMode mode, double value)	 To Move to robot
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final PWMSparkMax m_frontLeft = new PWMSparkMax(DriveConstants.kFrontLeftMotorPort);
  private final PWMSim m_frontLeftSim = new PWMSim(m_frontLeft);

  private final PWMSparkMax m_rearLeft = new PWMSparkMax(DriveConstants.kRearLeftMotorPort);
  private final PWMSim m_rearLeftSim = new PWMSim(m_rearLeft);

  private final PWMSparkMax m_frontRight = new PWMSparkMax(DriveConstants.kFrontRightMotorPort);
  private final PWMSim m_frontRightSim = new PWMSim(m_frontRight);

  private final PWMSparkMax m_rearRight = new PWMSparkMax(DriveConstants.kRearRightMotorPort);
  private final PWMSim m_rearRightSim = new PWMSim(m_rearRight);

  private final MecanumDrive m_drive =
      new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  // The front-left-side drive encoder
  private final Encoder m_frontLeftEncoder =
      new Encoder(
          DriveConstants.kFrontLeftEncoderPorts[0],
          DriveConstants.kFrontLeftEncoderPorts[1],
          DriveConstants.kFrontLeftEncoderReversed);
  private final EncoderSim m_frontLeftEncoderSim = new EncoderSim(m_frontLeftEncoder);

  // The rear-left-side drive encoder
  private final Encoder m_rearLeftEncoder =
      new Encoder(
          DriveConstants.kRearLeftEncoderPorts[0],
          DriveConstants.kRearLeftEncoderPorts[1],
          DriveConstants.kRearLeftEncoderReversed);
  private final EncoderSim m_rearLeftEncoderSim = new EncoderSim(m_rearLeftEncoder);

  // The front-right--side drive encoder
  private final Encoder m_frontRightEncoder =
      new Encoder(
          DriveConstants.kFrontRightEncoderPorts[0],
          DriveConstants.kFrontRightEncoderPorts[1],
          DriveConstants.kFrontRightEncoderReversed);
  private final EncoderSim m_frontRightEncoderSim = new EncoderSim(m_frontRightEncoder);

  // The rear-right-side drive encoder
  private final Encoder m_rearRightEncoder =
      new Encoder(
          DriveConstants.kRearRightEncoderPorts[0],
          DriveConstants.kRearRightEncoderPorts[1],
          DriveConstants.kRearRightEncoderReversed);
  private final EncoderSim m_rearRightEncoderSim = new EncoderSim(m_rearRightEncoder);

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);


  // The field object for the simulator
  private final Field2d m_field = new Field2d();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field", m_field);

    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    // Sets the distance per pulse for the encoders
    m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());

    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {

    var before = getCurrentWheelDistances();

    // Compute the distance traveled based on PWM velocity
    // There are unit-conversions to work out here. PWM speed is -1 to 1;
    double frontLeftVel = m_frontLeftSim.getSpeed();
    double frontLeftPos = m_frontLeftEncoderSim.getDistance() + frontLeftVel * 0.02;
    m_frontLeftEncoderSim.setDistance(frontLeftPos);

    // The right encoder is also inverted
    double frontRightVel = m_frontRightSim.getSpeed();
    double frontRightPos = m_frontRightEncoderSim.getDistance() - frontRightVel * 0.02;
    m_frontRightEncoderSim.setDistance(frontRightPos);

    double rearLeftVel = m_rearLeftSim.getSpeed();
    double rearLeftPos = m_rearLeftEncoderSim.getDistance() + rearLeftVel * 0.02;
    m_rearLeftEncoderSim.setDistance(rearLeftPos);

    // The right encoder is also inverted
    double rearRightVel = m_rearRightSim.getSpeed();
    double rearRightPos = m_rearRightEncoderSim.getDistance() - rearRightVel * 0.02;
    m_rearRightEncoderSim.setDistance(rearRightPos);

    var after = getCurrentWheelDistances();

    var twist = DriveConstants.kDriveKinematics.toTwist2d(before, after);

    var prev_angle = m_gyro.getAngle();
    m_gyroSim.setAngle(prev_angle + Units.radiansToDegrees(twist.dtheta));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      // jleibs: Even though ySpeed is supposed to be positive=left, the MecanumDriveKinematics
      // implementation appears to be incorrect. Swap the command here so that the odometry
      // ends up correct. We could maintain our own patchedc version of MecanumDriveKinematics
      // if we wanted everything to be consistent.
      m_drive.driveCartesian(xSpeed, -ySpeed, rot, m_gyro.getRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, -ySpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_rearLeftEncoder.getRate(),
        m_rearRightEncoder.getRate());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_rearLeftEncoder.getDistance(),
        m_rearRightEncoder.getDistance());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
