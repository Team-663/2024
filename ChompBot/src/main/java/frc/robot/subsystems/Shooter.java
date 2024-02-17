// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Shooter extends SubsystemBase {
   /** Creates a new Arm. */
   //private final CANSparkMax m_shooterMotorLeader = new CANSparkMax(Constants.ArmConstants.CANID_SHOOTER_SPARKMAX_1,
   //      MotorType.kBrushless);
   //private final CANSparkMax m_shooterMotorFollower = new CANSparkMax(Constants.ArmConstants.CANID_SHOOTER_SPARKMAX_2,
   //      MotorType.kBrushless);
   //private final WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(Constants.ArmConstants.CANID_INTAKE_VICTOR);
   
   private final WPI_TalonSRX m_armMotor = new WPI_TalonSRX(ArmConstants.CANID_ARM_TALON_1);
   private final WPI_VictorSPX m_armMotor2 = new WPI_VictorSPX(ArmConstants.CANID_ARM_VICTOR_2);

   //private final SparkPIDController m_shooterPID = m_shooterMotorLeader.getPIDController();
   //private final RelativeEncoder m_shooterEnc = m_shooterMotorLeader.getEncoder();   
   
   private final LaserCan m_laser = new LaserCan(21);
   private boolean m_laserValidMeasurement = false;
   private double m_shooterSetpoint = 0.0;
   private boolean m_isShooterAtSetpoint = false;
   private double m_laserDistInches = 0.0;

   private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
   private GenericEntry sbtLaserEntry = tab.add("Laser Distance", 0).getEntry();
   private GenericEntry sbtShooterSpeed = tab.add("Shooter RPM", 0).getEntry();
   private GenericEntry sbtShooterSetpoint = tab.add("Shooter Setpt", 0).getEntry();
   private GenericEntry sbtNoteInsideIntake = tab.add("Is Note Inside", 0).getEntry();
   private GenericEntry sbtArmSpeed = tab.add("Arm Speed", 0).getEntry();
   private GenericEntry sbtArmPosition = tab.add("Arm Position", 0).getEntry();
   private GenericEntry stbArmXboxInput = tab.add("Arm Xbox Input", 0).getEntry();
   public Shooter() {
      //m_intakeMotor.setNeutralMode(NeutralMode.Brake);
      //m_intakeMotor.setInverted(true);
      //m_shooterMotorLeader.setIdleMode(IdleMode.kCoast);
      //m_shooterMotorLeader.setInverted(true);
      
      //m_shooterMotorFollower.setIdleMode(IdleMode.kCoast);
      //m_shooterMotorFollower.follow(m_shooterMotorLeader, true);

      initLaserCAN();
      initShooterMotorSettings();
      initArmMotorSettings();

   }

   private boolean CheckIfNoteInIntake()
   {
      if (m_laserDistInches < Constants.ArmConstants.LASER_BEAM_BREAK_THRESHOLD)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   private void armByXbox(double armValue)
   {
      stbArmXboxInput.setDouble(armValue);
      m_armMotor.set(armValue);
   }

   private void shooterByXbox(double intake, double shooter)
   {
      //m_intakeMotor.set(intake);
      //m_shooterMotorLeader.set(shooter);
   }

   private void intakeNote()
   {
      // run intake motors at intake speed
      // if beam is broken, stop
      if (!CheckIfNoteInIntake())
         intakeMotorSet(Constants.ArmConstants.INTAKE_MOTOR_SPEED);
      else
         intakeIdle();

   }
   private void intakeIdle()
   {
      //m_intakeMotor.stopMotor();
      //m_shooterPID.setReference(0.0, ControlType.kVelocity);

   }

   private void intakeMotorSet(double val)
   {
      //m_intakeMotor.set(val);
   }

   private void shooterEnablePID()
   {
      //m_shooterPID.setReference(m_shooterSetpoint, ControlType.kVelocity);
   }

   private void shootNote(double targetRPM)
   {
      // 1) Set shooter wheel to target RPM
      // 2) IF shooter at target RPM, shoot the note

      // TODO: if beam is broken, drive the intake backwards slowly-ish
      // Dont do anything else until beam is no longer broken
   
      m_shooterSetpoint = targetRPM;
      shooterEnablePID();
      if (m_isShooterAtSetpoint)
         intakeMotorSet(Constants.ArmConstants.INTAKE_MOTOR_SHOOT_SPEED);
   }

   public Command armByXboxCommand(double armVal)
   {
      return run(() -> armByXbox(armVal)).withName("armByXbox");
   }
   public Command shooterByXboxCommand(DoubleSupplier intake, DoubleSupplier shooter) {
      return run(() -> shooterByXbox(intake.getAsDouble(), shooter.getAsDouble()))
            .withName("shooterByXbox");
   }

   public Command intakeNoteCommand()
   {
      return run(() -> intakeNote()).withName("intakeNote");
   }

   public Command shootNoteCommand(double rpm)
   {
      return run(() -> shootNote(rpm)).withName("shootNote");
   }

   public Command shooterIdleCommand()
   {
      return run(() -> intakeIdle()).withName("idle");
   }


   private void UpdateLaserDistance()
   {
      LaserCan.Measurement measurement = m_laser.getMeasurement();
      if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) 
      {
         m_laserValidMeasurement = true;
         m_laserDistInches = (double)measurement.distance_mm / 25.4;
      }
      else
      {
         m_laserValidMeasurement = false;
      }
    
   }

   public double GetLaserDistance()
   {
      return m_laserDistInches;
   }

   private void initLaserCAN()
   {
      // TODO: set defaults here if necessary
      /*
      try
      {
        // m_laser.setRangingMode(LaserCan.RangingMode.SHORT);
         //m_laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
         //m_laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      }
      catch (ConfigurationFailedException e)
      {
         System.out.println("LaserCAN configuration failed! " + e);
      }
      */
   }
   private void UpdateShuffleboardValues()
   {
      sbtLaserEntry.setDouble(m_laserDistInches);
      //sbtShooterSpeed.setDouble(m_shooterEnc.getVelocity());
      sbtNoteInsideIntake.setBoolean(CheckIfNoteInIntake());
      sbtShooterSetpoint.setDouble(m_shooterSetpoint);
      sbtArmSpeed.setDouble((m_armMotor.get()));
      sbtArmPosition.setDouble(m_armMotor.getSensorCollection().getQuadraturePosition());
   }

   private void initShooterMotorSettings()
   {
      //m_shooterPID.setP(ArmConstants.SHOOTER_PID_P);
      //m_shooterPID.setI(ArmConstants.SHOOTER_PID_I);
      //m_shooterPID.setD(ArmConstants.SHOOTER_PID_D);
      //m_shooterPID.setFF(ArmConstants.SHOOTER_PID_F);

   }
   private void initArmMotorSettings()
   {
      m_armMotor.configFactoryDefault();
      m_armMotor2.follow(m_armMotor);
      m_armMotor.setNeutralMode(NeutralMode.Brake);
      m_armMotor2.setNeutralMode(NeutralMode.Brake);

      //m_armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
      //m_armMotor.SetSensorPhase(true);
      m_armMotor.configAllowableClosedloopError(0, ArmConstants.ARM_MAX_PID_ERROR);
      m_armMotor.configClosedLoopPeakOutput(0, ArmConstants.ARM_MAX_PID_POWER);

      m_armMotor.config_kP(0, ArmConstants.ARM_PID_P);
      m_armMotor.config_kP(0, ArmConstants.ARM_PID_I);
      m_armMotor.config_kP(0, ArmConstants.ARM_PID_D);
      m_armMotor.config_kP(0, ArmConstants.ARM_PID_F);
      /* 
      m_elevator.ConfigForwardSoftLimitThreshold(-20000.0, kTimeoutMs);
      m_elevator.ConfigForwardSoftLimitEnable(false);
      m_elevator.ConfigReverseSoftLimitThreshold(-20000.0, kTimeoutMs);
      m_elevator.ConfigReverseSoftLimitEnable(false);
       */
   }

   @Override
   public void periodic() {
      UpdateLaserDistance();
      UpdateShuffleboardValues();

      // This method will be called once per scheduler run
   }

}
