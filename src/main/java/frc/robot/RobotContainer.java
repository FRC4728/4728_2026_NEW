// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoAlignTurret;
import frc.robot.commands.DropIntake;
import frc.robot.commands.JogTurretNegative;
import frc.robot.commands.JogTurretPositive;
import frc.robot.commands.Pass;
import frc.robot.commands.ReverseAll;
import frc.robot.commands.RunIntakeIn;
import frc.robot.commands.RunIntakeOut;
import frc.robot.commands.RunSpindexerRev;
import frc.robot.commands.Score;
import frc.robot.commands.ScoreCorner;
import frc.robot.commands.ScoreDyn;
import frc.robot.commands.SetHoodMax;
import frc.robot.commands.SetHoodMid;
import frc.robot.commands.SetHoodMin;
import frc.robot.commands.SetShooterByDistance;
import frc.robot.commands.SetTurretCenter;
import frc.robot.commands.SetTurretZeroish;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;

public class RobotContainer {

    // Subsystems
    private final Intake intake = new Intake();
    private final Turret turret = new Turret();
    private final TurretShooter shooter = new TurretShooter();
    private final Kicker kicker = new Kicker();
    private final Indexer indexer = new Indexer();
    //private final LED led = new LED();

    // Drive speed multipliers
    private double translationMultiplier = 0.9;
    private double strafeMultiplier = 0.9;
    private double rotateMultiplier = 0.9;

    // Drivetrain speed limits
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Swerve drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.08)
            .withRotationalDeadband(MaxAngularRate * 0.08)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    // Controllers
    private final CommandXboxController driver   = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Drivetrain and telemetry
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /** Selects which of the two pre-configured pass coordinates to aim at. */
    private final SendableChooser<Integer> passTargetChooser = new SendableChooser<>();
    //public void periodic(){
    //}

    public RobotContainer() {
        configureDefaultCommands();
        configureDriverBindings();
        configureOperatorBindings();
        configureAutomation();
        drivetrain.registerTelemetry(logger::telemeterize);

        new EventTrigger("AutoAlignTurret").whileTrue(new AutoAlignTurret(turret, drivetrain).withTimeout(20));
        new EventTrigger("DropIntake").onTrue(new RunIntakeOut(intake).withTimeout(1));
        new EventTrigger("ZeroTurret").onTrue(new SetTurretZeroish(turret));
        new EventTrigger("CenterTurret").onTrue(new SetTurretCenter(turret));
        new EventTrigger("Score").whileTrue(new Score(indexer, kicker, shooter, turret, drivetrain).withTimeout(8));
    

        NamedCommands.registerCommand("Score",new Score(indexer, kicker, shooter, turret, drivetrain).withTimeout(8));
        NamedCommands.registerCommand("RunIntake",new RunIntakeIn(intake).withTimeout(20));
 
        //create auto chooser in dashboard
        autoChooser = AutoBuilder.buildAutoChooser("Main"); 
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Pass target chooser 
        passTargetChooser.setDefaultOption("Pass Target Left", 1);
        passTargetChooser.addOption("Pass Target Right", 2);
        SmartDashboard.putData("Pass Target", passTargetChooser);
    }

    // ── Default Commands ─────────────────────────────────────────────────────

    private void configureDefaultCommands() {
        // Drivetrain: field-centric drive
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * translationMultiplier)
                     .withVelocityY(-driver.getLeftX() * MaxSpeed * strafeMultiplier)
                     .withRotationalRate(-driver.getRightX() * MaxAngularRate * rotateMultiplier)
            )
        );

        // Idle drivetrain when disabled
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Turret always auto aligns
        turret.setDefaultCommand(new AutoAlignTurret(turret, drivetrain));

        // Intake: always running in unless interrupted
        intake.setDefaultCommand(new RunIntakeIn(intake));

        // LED: always check which hub is active
        //led.setDefaultCommand(new CheckHubStatus(led));
    }

    // ── Driver Controller (port 0) ────────────────────────────────────────────

    private void configureDriverBindings() {

        //driver.x().whileTrue(new ScoreDyn(intake, indexer, kicker, shooter, turret));
        //driver.b().whileTrue(new RunIntakeOut(intake));
        //driver.leftTrigger().whileTrue(new RunKickerUp(kicker));
        //driver.rightTrigger().whileTrue(new RunShooter(shooter));
        //driver.leftBumper().whileTrue(new RunIntakeIn(intake));

        driver.rightBumper().whileTrue(new Score(indexer, kicker, shooter, turret, drivetrain)).onFalse(new SetShooterByDistance(shooter, drivetrain, turret).withTimeout(1.0));
        driver.leftBumper().whileTrue(new ScoreDyn(indexer, kicker, shooter, turret));
        driver.leftTrigger().whileTrue(new ScoreCorner(indexer, kicker, shooter, turret));
        driver.rightTrigger().whileTrue(new Pass(indexer, kicker, shooter, turret, drivetrain,
                () -> turret.getAutoPassTarget(drivetrain.getState().Pose)));
        driver.a().whileTrue(new RunIntakeOut(intake));
        driver.start().whileTrue(new ReverseAll(kicker, indexer));
        driver.y().onTrue(drivetrain.runOnce(() -> {drivetrain.seedFieldCentric(); drivetrain.markPoseReset();}));

        //SCORE _ right bumper to toggle drvetrain to low speed
        driver.rightBumper().whileTrue(new InstantCommand(() -> translationMultiplier = .2));
        driver.rightBumper().whileFalse(new InstantCommand(() -> translationMultiplier = 0.85));
        driver.rightBumper().whileTrue(new InstantCommand(() -> strafeMultiplier = .2));
        driver.rightBumper().whileFalse(new InstantCommand(() -> strafeMultiplier = 0.85));
        driver.rightBumper().whileTrue(new InstantCommand(() -> rotateMultiplier = .25));
        driver.rightBumper().whileFalse(new InstantCommand(() -> rotateMultiplier = 0.85));

        //PASS _ right trigger to toggle drvetrain to low speed
        driver.rightTrigger().whileTrue(new InstantCommand(() -> translationMultiplier = .4));
        driver.rightTrigger().whileFalse(new InstantCommand(() -> translationMultiplier = 0.85));
        driver.rightTrigger().whileTrue(new InstantCommand(() -> strafeMultiplier = .4));
        driver.rightTrigger().whileFalse(new InstantCommand(() -> strafeMultiplier = 0.85));
        driver.rightTrigger().whileTrue(new InstantCommand(() -> rotateMultiplier = .35));
        driver.rightTrigger().whileFalse(new InstantCommand(() -> rotateMultiplier = 0.85));

    }

    // ── Operator Controller (port 1) ──────────────────────────────────────────

    private void configureOperatorBindings() {
        operator.b().onTrue(new SetHoodMax(shooter));
        operator.a().onTrue(new SetHoodMid(shooter));
        operator.x().onTrue(new SetHoodMin(shooter));

        operator.rightBumper().whileTrue(new JogTurretPositive(turret));
        operator.leftBumper().whileTrue(new JogTurretNegative(turret));
        operator.rightTrigger().whileTrue(new Pass(indexer, kicker, shooter, turret, drivetrain,
                () -> turret.getAutoPassTarget(drivetrain.getState().Pose)));

        operator.y().onTrue(new SetTurretZeroish(turret));
        operator.start().onTrue(new SetTurretCenter(turret));
    }

    // ── Automated Triggers ────────────────────────────────────────────────────

    private void configureAutomation() {
    }

    // ── Autonomous ───────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}