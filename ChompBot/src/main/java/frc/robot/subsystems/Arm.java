// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
   /** Creates a new Arm. */
   private final CANSparkMax m_shooterMotorLeader = new CANSparkMax(Constants.ArmConstants.CANID_SHOOTER_SPARKMAX_1,
         MotorType.kBrushless);
   private final CANSparkMax m_shooterMotorFollower = new CANSparkMax(Constants.ArmConstants.CANID_SHOOTER_SPARKMAX_2,
         MotorType.kBrushless);
   private final WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(Constants.ArmConstants.CANID_INTAKE_VICTOR);

   private double m_shooterSetpoint = 0.0;

   public Arm() {
      m_intakeMotor.setNeutralMode(NeutralMode.Brake);
      m_shooterMotorLeader.setIdleMode(IdleMode.kCoast);
      m_shooterMotorLeader.setInverted(true);
      
      m_shooterMotorFollower.setIdleMode(IdleMode.kCoast);
      m_shooterMotorFollower.follow(m_shooterMotorLeader, true);

      initArmShuffleBoardTab();

   }

   private void armByXbox(double intake, double shooter, double angle)
   {
      m_intakeMotor.set(-intake);
      m_shooterMotorLeader.set(shooter);
   }

   public Command armByXboxCommand(DoubleSupplier intake, DoubleSupplier shooter) {
      return run(() -> armByXbox(intake.getAsDouble(), shooter.getAsDouble(), 0.0))
            .withName("armByXbox");
   }

   private void initArmShuffleBoardTab()
   {
      //ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
      //elevatorTab.add("Intake", m_intakeMotor);
      //elevatorTab.add("Shooter", m_shooterMotorLeader);
   }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run
   }

}
