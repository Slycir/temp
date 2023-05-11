// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class Drivetrain extends SubsystemBase {
  public Alliance teamColor;
  /** Creates a new SwerveSystem. */
  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTable limelightTwoTable = NetworkTableInstance.getDefault().getTable("limelight-two");

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleXOffsetMeters, kModuleYOffsetMeters),
      new Translation2d(kModuleXOffsetMeters, -kModuleYOffsetMeters),
      new Translation2d(-kModuleXOffsetMeters, kModuleYOffsetMeters),
      new Translation2d(-kModuleXOffsetMeters, -kModuleYOffsetMeters));

  private SwerveDrivePoseEstimator odometer;
  private AHRS navx = new AHRS();
  // private Pose2d oldPos = null;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  
  public DoubleArraySubscriber botPoseSub, tagPoseSub; 

  public double desiredVelocityAverage = 0;
  public double actualVelocityAverage = 0;

  public TrajectoryConfig config = new TrajectoryConfig(
      Constants.MeasurementConstants.kMaxSpeedMetersPerSecond / 2,
      Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared / 2)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(m_kinematics);



  /** 
   * Constructor creates 4 objects of the {@link SwerveModule} class. 
   * The Drivetrain class is responsible for orienting and manipulating them to manuever the robot, 
   * primarily through this class's drive method 
   */
  public Drivetrain() {
    teamColor = DriverStation.getAlliance();
    botPoseSub = limelightTable.getDoubleArrayTopic("botpose").subscribe(new double[]{});
    botPoseSub = limelightTwoTable.getDoubleArrayTopic("botpose").subscribe(new double[]{});
    tagPoseSub = limelightTable.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[]{});

    m_frontLeft = new SwerveModule(
        kFrontLeftDriveMotorID,
        kFrontLeftSteerMotorID,
        kFrontLeftEncoderID,
        kFrontLeftEncoderOffset);

    m_frontRight = new SwerveModule(
        kFrontRightDriveMotorID,
        kFrontRightSteerMotorID,
        kFrontRightEncoderID,
        kFrontRightEncoderOffset);

    m_backLeft = new SwerveModule(
        kBackLeftDriveMotorID,
        kBackLeftSteerMotorID,
        kBackLeftEncoderID,
        kBackLeftEncoderOffset);

    m_backRight = new SwerveModule(
        kBackRightDriveMotorID,
        kBackRightSteerMotorID,
        kBackRightEncoderID,
        kBackRightEncoderOffset);

    odometer = new SwerveDrivePoseEstimator(m_kinematics, getGyroRotation2d(), getModulePositions(), new Pose2d());
  }

  public void recalibrateModulesEncoders() { // Call if modules are not in the correct position
    m_frontLeft.recalibrateRelativeEncoder();
    m_frontRight.recalibrateRelativeEncoder();
    m_backLeft.recalibrateRelativeEncoder();
    m_backRight.recalibrateRelativeEncoder();
  }

  public double getNavxYaw() {
    var pos = navx.getYaw() + Timer.getFPGATimestamp() * 0.009 % 360;
    return pos < -180 ? pos + 360 : pos;
  }

  public double getNavxPitch() {
    return navx.getPitch();
  }

  public double getNavxRoll() {
    return navx.getRoll();
  }

  public void zeroGyro() {
    navx.reset();
  }

  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(-navx.getFusedHeading());
  }

  public void updateOdometry() {
    odometer.update(
        getGyroRotation2d(),
        getModulePositions());
    // If Limelight has a target, update the odometry with the Limelight's pose
    if(limelightTable.getEntry("tv").getDouble(0) == 1 || limelightTwoTable.getEntry("tv").getDouble(0) == 1) {
      updateOdometryIfTag();
    }
  }

  public double getOdometryYaw() {
    var pos = -odometer.getEstimatedPosition().getRotation().getDegrees() % 360;
    return pos < -180 ? pos + 360 : pos;
  }

  public void zeroOdometry() {
    odometer.resetPosition(getGyroRotation2d(), getModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public void setOdometry(Pose2d pose) {
    odometer.resetPosition(new Rotation2d(-getNavxYaw() * Math.PI / 180), getModulePositions(), pose);
  }
  
  public void setAprilTagOdometry(Pose2d pose) {
    odometer.resetPosition(new Rotation2d(), getModulePositions(), pose);
  }
  
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };
  }

  public void setFieldPosition(Pose2d pose) {
    odometer.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }

  public Pose2d getFieldPosition() {
    return odometer.getEstimatedPosition();
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void xPose() {
    SwerveModuleState frontLeft = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState frontRight = new SwerveModuleState(0, Rotation2d.fromDegrees(315));
    SwerveModuleState backLeft = new SwerveModuleState(0, Rotation2d.fromDegrees(315));
    SwerveModuleState backRight = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState[] swerveModuleStates = {frontLeft, frontRight, backLeft, backRight};
    setModuleStates(swerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            rot,
            odometer.getEstimatedPosition().getRotation()));

    setModuleStates(swerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, odometer.getEstimatedPosition().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    setModuleStates(swerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d ctrOfRot) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, odometer.getEstimatedPosition().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot), ctrOfRot);

    setModuleStates(swerveModuleStates);
  }

  public void printModuleAbsoluteAngles() {
    System.out.println("Front Left: " + m_frontLeft.getAbsoluteAngle());
    System.out.println("Front Right: " + m_frontRight.getAbsoluteAngle());
    System.out.println("Back Left: " + m_backLeft.getAbsoluteAngle());
    System.out.println("Back Right: " + m_backRight.getAbsoluteAngle());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    desiredVelocityAverage = Math.abs(states[0].speedMetersPerSecond + states[1].speedMetersPerSecond + states[2].speedMetersPerSecond + states[3].speedMetersPerSecond) / 4;

    m_frontLeft.setDesiredStateOpen(states[0]);
    m_frontRight.setDesiredStateOpen(states[1]);
    m_backLeft.setDesiredStateOpen(states[2]);
    m_backRight.setDesiredStateOpen(states[3]);
  }

  public Command getCommandForTrajectory(PathPlannerTrajectory trajectory) {
    xController = new PIDController(kDriveP, 0, 0);
    yController = new PIDController(kDriveP, 0, 0);
    thetaController = new PIDController(kTurnP, 0, 0); // Kp value, Ki=0, Kd=0
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // setFieldPosition(trajectory.getInitialHolonomicPose());

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        trajectory,
        this::getFieldPosition,
        m_kinematics,
        xController,
        yController,
        thetaController,
        this::setModuleStates,
        true,
        this);
    return swerveControllerCommand.andThen(() -> stop());
  }

  @Override
  public void periodic() {
    actualVelocityAverage = (m_frontLeft.getModuleVelocity() + m_frontRight.getModuleVelocity() + m_backLeft.getModuleVelocity() + m_backRight.getModuleVelocity()) / 4;
    SmartDashboard.putString("Pose1", getRobotPoseFromAprilTag().toString());
    SmartDashboard.putString("Pose2", getAlternateRobotPoseFromAprilTag().toString());
    updateOdometry();
    // SmartDashboard.putNumber("TA", getTA()); 
    // SmartDashboard.putNumber("TX", getTX());
    // SmartDashboard.putNumber("NavXYaw", getNavxYaw());
    SmartDashboard.putString("Gyro Rotation", getGyroRotation2d().toString());
    // SmartDashboard.putNumber("Pitch", getNavxPitch());
    SmartDashboard.putString("Position", odometer.getEstimatedPosition().toString());
    SmartDashboard.putNumber("Desired Velocity Average", desiredVelocityAverage);
    SmartDashboard.putNumber("Actual Velocity Average", actualVelocityAverage);
    SmartDashboard.putNumber("Desired/Acutal Velocity Ratio", desiredVelocityAverage / actualVelocityAverage);
  }

  public void updateOdometryIfTag() {
    // if (getTV() == 1 && getTID() < 4 || getTID() > 5 &&  getTID() < 9 && isDetectingAprilTags()) {
    //   var newPos = new Pose2d(getRobotPoseFromAprilTag().getTranslation(), getFieldPosition().getRotation());
    //   setOdometry(newPos);
    // }

    if((limelightTwoTable.getEntry("targetpose-cameraspace").getDoubleArray(new double[6])[2] < limelightTable.getEntry("targetpose-cameraspace").getDoubleArray(new double[6])[2]) || (limelightTable.getEntry("tv").getDouble(0.0) == 0 && limelightTwoTable.getEntry("tv").getDouble(0.0) == 1)) {
      odometer.addVisionMeasurement(
        getRobotPoseFromAprilTag(2), 
        Timer.getFPGATimestamp() - 
          limelightTable.getEntry("tl").getDouble(0.0)/1000 - 
          limelightTable.getEntry("cl").getDouble(0.0)/1000
      );
    } else if((limelightTwoTable.getEntry("targetpose-cameraspace").getDoubleArray(new double[6])[2] > limelightTable.getEntry("targetpose-cameraspace").getDoubleArray(new double[6])[2]) || (limelightTable.getEntry("tv").getDouble(0.0) == 1 && limelightTwoTable.getEntry("tv").getDouble(0.0) == 0)){
      odometer.addVisionMeasurement(
        getRobotPoseFromAprilTag(1), 
        Timer.getFPGATimestamp() - 
          limelightTwoTable.getEntry("tl").getDouble(0.0)/1000 - 
          limelightTwoTable.getEntry("cl").getDouble(0.0)/1000
      );
    } else {
      System.err.println("Something went wrong with the vision measurements");
    }
    
  }



  /// **********VISION SECTION *************/
  public int getTV() {
    return (int) limelightTable.getEntry("tv").getInteger(0);
  }

  public int getTID() {
    return (int) limelightTable.getEntry("tid").getInteger(0);
  }

  public double getTX() {
    return limelightTable.getEntry("tx").getDouble(0);
  }

  public double getTA() {
    return limelightTable.getEntry("ta").getDouble(0);
  }

  public boolean isDetectingAprilTags() {
    var entry = limelightTable.getEntry("botpose");
    return entry.getDoubleArray(new double[]{}).length == 6;
  }

  public void limelightToTapeMode() {
    limelightTable.getEntry("pipeline").setNumber(1);
    limelightTable.getEntry("ledMode").setNumber(3); // 3 means on, 1 means off
  }

  public Integer getPIP() {
    return (int)limelightTable.getEntry("getpip").getInteger(0);
  }

  public void limelightToTagMode() {
    limelightTable.getEntry("pipeline").setNumber(0);
    limelightTable.getEntry("ledMode").setNumber(1);
  }

  public Pose2d getRobotPoseFromAprilTag() {
      var entry = limelightTable.getEntry("botpose").getDoubleArray(new double[]{getFieldPosition().getX(), getFieldPosition().getY()});
      var pose2d = new Pose2d(new Translation2d(entry[0], entry[1]), odometer.getEstimatedPosition().getRotation());
  
      return pose2d;
  }

  public Pose2d getRobotPoseFromAprilTag(int limelight) {
    Pose2d pose2d;

    if(limelight == 1){
      var entry = limelightTable.getEntry("botpose").getDoubleArray(new double[]{getFieldPosition().getX(), getFieldPosition().getY()});
      pose2d = new Pose2d(new Translation2d(entry[0], entry[1]), odometer.getEstimatedPosition().getRotation());
    } else {
      var entry = limelightTwoTable.getEntry("botpose").getDoubleArray(new double[]{getFieldPosition().getX(), getFieldPosition().getY()});
      pose2d = new Pose2d(new Translation2d(entry[0], entry[1]), odometer.getEstimatedPosition().getRotation());
    }

    return pose2d;
}

  public Pose2d getAlternateRobotPoseFromAprilTag() {
      var entry = limelightTwoTable.getEntry("botpose").getDoubleArray(new double[]{getFieldPosition().getX(), getFieldPosition().getY()});
      var pose2d = new Pose2d(new Translation2d(entry[0], entry[1]), odometer.getEstimatedPosition().getRotation());
  
      return pose2d;
  }

  public double getRobotRotationFromAprilTag() {
    var entry = limelightTable.getEntry("botpose").getDoubleArray(new double[]{getFieldPosition().getX(), getFieldPosition().getY()});
    var rot2d = entry[5];
    return rot2d;
  }

}
