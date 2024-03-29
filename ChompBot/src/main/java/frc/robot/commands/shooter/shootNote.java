// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class shootNote extends Command {
   Shooter m_shooter;
   BooleanSupplier m_shoot;
   double m_speed;
   private boolean m_hasShot;
   private Timer time;
   /** Creates a new shootNote. */
   public shootNote(Shooter shooter, double speed, BooleanSupplier shoot)
   {
      m_shooter = shooter;
      m_shoot = shoot;
      m_speed = speed;
      m_hasShot = false;
      addRequirements(m_shooter);
      time = new Timer();
      // Use addRequirements() here to declare subsystem dependencies.
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize()
   {
      m_shooter.shooterEnablePID(m_speed);
      m_hasShot = false;
      time.reset();

   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
      double intakeSpeed = 0.0;

      if (m_shoot.getAsBoolean() && m_shooter.IsShooterUpToSpeed())
      {
         m_hasShot = true;
         intakeSpeed = Constants.ArmConstants.INTAKE_MOTOR_SHOOT_SPEED;
         time.start();
         SmartDashboard.putBoolean("ShootNow", true);
      }
      else
      {
         SmartDashboard.putBoolean("ShootNow", false);
      }
      SmartDashboard.putNumber("ShootNote Intk", intakeSpeed);
      SmartDashboard.putBoolean("hasShot", m_hasShot);
      m_shooter.intakeMotorSet(intakeSpeed);

   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      m_shooter.intakeMotorSet(0);
      m_shooter.shooterIdle();
      time.reset();

   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
      // TODO: do we need a timer here?
      //return (m_hasShot && time.hasElapsed(Auton.AUTO_SHOOT_DURATION));
      return (!m_shooter.CheckIfNoteInIntake());
   }
}
