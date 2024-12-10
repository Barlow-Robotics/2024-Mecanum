// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public enum IntakeState {
    INTAKING, BASE, ISMOVING
  }

  public static final double jKgMetersSquared = 0.0005;

  public static final class DriveConstants {
    public static final int kFrontLeftMotorPort = 0;
    public static final int kFrontRightMotorPort = 1;
    public static final int kBackRightMotorPort = 2;
    public static final int kBackLeftMotorPort = 3;


    public static final int[] kFrontLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kBackLeftEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightEncoderPorts = new int[] {4, 5};
    public static final int[] kBackRightEncoderPorts = new int[] {6, 7};

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kBackLeftEncoderReversed = true;
    public static final boolean kFrontRightEncoderReversed = false;
    public static final boolean kBackRightEncoderReversed = true;

    public static final double sensitivityScale = 0.3;
    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
  }

  public static final class FloorIntakeConstants {
    /* PID CONTROLLER */
    public static final double KP = 0.2;
    public static final double KI = 0;
    public static final double KD = 0.01;
    public static final double IZone = 0;
    // public static final double FF = 1 / KrakenX60MaxRPM / 60;
    public static final double FF = 0.13; // KV

    public static final int SupplyCurrentLimit = 20;

    // public static final double MotorRPM = 2500;
    public static final double MotorRPM = 1750;
    public static final double VelocityTolerance = 5;
  }

  public static final class ShoulderMountConstants {
    // FIX CONSTANTS, COPIED FROM SHOULDERMOUNT
    /* PID CONTROLLER */
    public static final double KP = 0.2;
    public static final double KI = 0;
    public static final double KD = 0.01;
    public static final double IZone = 0;
    // public static final double FF = 1 / KrakenX60MaxRPM / 60;
    public static final double FF = 0.13; // KV

    public static final int SupplyCurrentLimit = 20;

    // public static final double MotorRPM = 2500;
    public static final double MotorRPM = 1750;
    public static final double VelocityTolerance = 5;
    public static final int baseAngle = 0;
    public static final int intakeAngle = 69;

    /* ANGLE */

    public static final double AngleKP = 30;
    public static final double AngleKI = 0.000;
    public static final double AngleKD = 1.5;
    // public static final double AngleIZone = 0; // motor already does this
    public static final double AngleFF = 0.0;
    public static final double AngleKG = 0.29;

    public static final double AngleGearRatio = 46.67; // From K's spreadsheet
    public static final double AngleCruiseRotationsPerSec = 3; 
    public static final double AngleAcceleration = 12;
    public static final double AngleJerk = 40; //30; 
  }
public static final class ShooterMountConstants {

        public static final double SupplyCurrentLimit = 40;       
       
        /* HEIGHT / ANGLE PAIRS */

        public static final double AngleTolerance = 1.5; 
        public static final double HeightTolerance = 0.25;

        public static final double MaxAngleDegrees = 55;
        public static final double MinAngleDegrees = -61.5;
        public static final double MaxHeightInches = 45;
        public static final double StartingHeight = 24.75; //19.75;

        public static final double SpeakerAngle = 50; 
        public static final double SpeakerHeight = StartingHeight; 

        public static final double AmpAngle = -45;
        public static final double AmpHeight = 46.5; // 43.25; // 40;
        
        public static final double SourceIntakeAngle = 32; 
        public static final double SourceIntakeHeight = 31.6; // 27;

        public static final double FloorIntakeAngle = MinAngleDegrees; 
        public static final double FloorIntakeHeight = StartingHeight; 

        public static final double ClimbHeight = 47; // 44;
        public static final double TrapAngle = -5; // CHANGE
        public static final double TrapHeight = 46; // CHANGE

        // public static final double FerryAngle = -10;
        public static final double FerryAngle = 20;
        public static final double FerryHeight = StartingHeight;

       // public static final double AngleCANCoderMagnetOffset = 0.499755859375;
       // public static final double AngleCANCoderMagnetOffset = 0.513611111;
        public static final double AngleCANCoderMagnetOffset = 0.48583;


        public static final double AngleGearRatio = 46.67; // From K's spreadsheet
        public static final double AngleCruiseRotationsPerSec = 3; 
        public static final double AngleAcceleration = 12;
        public static final double AngleJerk = 40; //30; 

        /* ELEVATOR */

        public static final double ElevatorKP = 32;
        public static final double ElevatorKI = 0.001;
        public static final double ElevatorKD = 0.0;
        // public static final double ElevatorIZone = 0.1; // motor already does this
        public static final double ElevatorFF = 0.0;
        public static final double ElevatorKG = 2.7;
        public static final double ElevatorKS = 0; // use error sign instead of velocity sign? as part of initial config 


        public static final double ClimbKP = 32;
        public static final double ClimbKI = 0.001;
        public static final double ClimbKD = 0.0;
        public static final double ClimbFF = 0.0;
        public static final double ClimbKG = 4.0;
        public static final double ClimbKS = 0; // use error sign instead of velocity sign? as part of initial config 


        public static final double ElevatorGearRatio = 15;
//        public static final double ElevatorSprocketDiameter = 2.36;  // inches
        public static final double ElevatorSprocketDiameter = 2.16;  // inches
        public static final double ElevatorSprocketCircumference = ElevatorSprocketDiameter * Math.PI;
        public static final double RotationsPerElevatorInch = 1 / ElevatorSprocketCircumference * ElevatorGearRatio;
        // public static final double RotationsPerElevatorInch = 
        // ElevatorGearRatio / Units.metersToInches(ElevatorSprocketCircumference) / 2;
        public static final double ElevatorCruiseInchesPerSec = 10; 
        public static final double ElevatorInchesPerSecPerSec = 10; 
        public static final double ElevatorJerk = 800; // CHANGE - Target jerk of 1600 rps/s/s (0.1 seconds)
    
        // LMT added constants to enable changing shooter angle while driving to speaker
        public static final double CameraMountHeight = 24; // inches - possibly CHANGE
        public static final double CameraMountAngle = 3; // degrees - possibly CHANGE
        public static final double SpeakerAprilTagHeight = 52; /*inches - possibly CHANGE - is this the bottom of the
                                                            * AT? Might need to change to midpoint for the calc to
                                                            * work, originally did that from bottoms of speaker and
                                                            * AT */
        // public static final double MidSpeakerHeight = 80.4; // inches to middle of speaker hole - possibly CHANGE
        // public static final double MidSpeakerHeight = 88; // bottom of speaker opening is at 78", so aim above it.
        public static final double MidSpeakerHeight = 112; // bottom of speaker opening is at 78", so aim above it.
        //  public static final double MidSpeakerHeight = 105; // bottom of speaker opening is at 78", so aim above it.
        public static final double ElevatorHeightUnextended = 26; // inches - possibly CHANGE - height of elevator at rest
        
        public static final int MissedSpeakerTargetFrameTolerance = 13;  // rougly .25 seconds
    }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ElectronicsIDs {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 0;
    public static final int FloorMotorID = 61;
    public static final int ShoulderMotorID = 62; // placeholder
  }
}
