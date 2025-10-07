// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.common.AutoFactory;
import frc.robot.auto.common.AutoFactory.Auto;
import frc.robot.auto.modes.DeadReckoning;
import frc.robot.auto.modes.LOLILEFTLeftFirst;
import frc.robot.auto.modes.MiddleH4;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.commands.superstructure.SuperstructureCommandFactory;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.SuperstructureStateType;
import frc.robot.subsystems.superstructure.SuperstructurePosition.SuperstructureState;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Telemetry;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.io.Dashboard;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class RobotContainer {
    private final ControlBoard controlBoard = ControlBoard.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();

    public final RobotState robotState = RobotState.getInstance();
    public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    public final SuperstructureSubsystem superstructure = SuperstructureSubsystem.getInstance();
    public final ArmPivotSubsystem armPivot = ArmPivotSubsystem.getInstance();
    public final ArmRollerSubsystem armRollers = ArmRollerSubsystem.getInstance();
    public final IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();
    public final IntakeRollerSubsystem intakeRollers = IntakeRollerSubsystem.getInstance();
    public final VisionSubsystem vision = VisionSubsystem.getInstance();
    public final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    public final AutoFactory autoFactory = AutoFactory.getInstance();

    public static boolean deadReckoning = false;

    public RobotContainer() {
        drivetrain.setDefaultCommand(new DefaultDriveCommand(
                controlBoard::getThrottle,
                // Sideways velocity supplier.
                controlBoard::getStrafe,
                // Rotation velocity supplier.
                controlBoard::getRotation,
                () -> true));

        configureBindings();
    }

    private void configureBindings() {
        /* Primary Driver */
        configurePOVBindings();

        controlBoard
                .groundIntakeHold()
                .onTrue(IntakeCommandFactory.setHoldCoral(true))
                .onFalse(IntakeCommandFactory.setHoldCoral(false));

        // controlBoard.resetGyro().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));
        
        controlBoard.resetGyro().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset()));

        controlBoard
                .intake()
                .onTrue(Commands.either(
                        Commands.none(),
                        superstructure.setGoal(SuperstructureState.INTAKE),
                        robotState.isAlgaeMode()))
                .whileTrue(Commands.either(
                        ArmCommandFactory.algaeIn(),
                        Commands.parallel(ArmCommandFactory.intake(), IntakeCommandFactory.intake()),
                        () -> superstructure.getGoalState().getType() == SuperstructureStateType.ALGAE));           

        controlBoard
                .outtake()
                .onTrue(ArmCommandFactory.score())
                .onFalse(superstructure.setGoal(SuperstructureState.STOW));

        controlBoard.armRollerTapIn().whileTrue(ArmCommandFactory.coralIn());

        controlBoard
                .groundOuttake()
                .whileTrue(IntakeCommandFactory.outtake())
                .onFalse(superstructure.setGoal(SuperstructureState.STOW));

        controlBoard.confirmSuperstructure().onTrue(superstructure.confirm());

        controlBoard
                .alignWithReefLeft()
                .whileTrue(AlignmentCommandFactory.getReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH))
                .onFalse(robotState.setAlignOffsetCommand(AlignOffset.MIDDLE_REEF));

        controlBoard
                .alignWithReefRight()
                .whileTrue(AlignmentCommandFactory.getReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH))
                .onFalse(robotState.setAlignOffsetCommand(AlignOffset.MIDDLE_REEF));

        /* Secondary Driver */
        controlBoard.actTrigger().onTrue(superstructure.confirm());

        controlBoard.setGoalCL().onTrue(superstructure.setSelected(SuperstructureState.CLIMB, true));
        controlBoard
                .setGoalL1H()
                .onTrue(superstructure.setSelected(SuperstructureState.L1H, false));
        controlBoard.setGoalL2().onTrue(superstructure.setSelected(SuperstructureState.L2, false));
        controlBoard.setGoalL3().onTrue(superstructure.setSelected(SuperstructureState.L3, false));
        controlBoard.setGoalL4().onTrue(superstructure.setSelected(SuperstructureState.L4, false));
        controlBoard
                .setGoalLowerAlgae()
                .onTrue(superstructure.setSelected(SuperstructureState.LOWER_ALGAE, false));
        controlBoard
                .setGoalUpperAlgae()
                .onTrue(superstructure.setSelected(SuperstructureState.UPPER_ALGAE, false));

        controlBoard.setGoalCoralStation().onTrue(superstructure.setSelected(SuperstructureState.SPOOKY_STOW, false));
        controlBoard.homeElevator().onTrue(superstructure.setSelected(SuperstructureState.HOME, false));

        controlBoard.climbUp().whileTrue(ClimberCommandFactory.climberUp());
        controlBoard.climbDown().whileTrue(ClimberCommandFactory.climberDown());

        controlBoard.algaeScoreAngle().onTrue(superstructure.setSelected(SuperstructureState.ALGAE_NET, false));
        controlBoard.algaeLowAngle().onTrue(superstructure.setSelected(SuperstructureState.ALGAE_PROCESS, false));

        controlBoard.loadingStation().onTrue(superstructure.setSelected(SuperstructureState.HP, false));
        controlBoard.unJam().onTrue(superstructure.setSelected(SuperstructureState.UN_JAM, false));

        /* SysID */
        controlBoard.sysIDDynamicForward().onTrue(SuperstructureCommandFactory.setCoast());
        controlBoard.sysIDDynamicReverse().onTrue(SuperstructureCommandFactory.setBrake());
        // controlBoard.sysIDQuasiForward().whileTrue(intakePivot.sysIdQuasistatic(Direction.kForward));
        // controlBoard.sysIDQuasiReverse().whileTrue(intakePivot.sysIdQuasistatic(Direction.kReverse));
        // controlBoard.sysIDDynamicForward().whileTrue(intakePivot.sysIdDynamic(Direction.kForward));
        // controlBoard.sysIDDynamicReverse().whileTrue(intakePivot.sysIdDynamic(Direction.kReverse));
        // controlBoard
        //         .outtake()
        //         .onTrue(Commands.runOnce(SignalLogger::start))
        //         .onFalse(Commands.runOnce(SignalLogger::stop));
    }

    private void configurePOVBindings() {
        ControlBoard controlBoard = ControlBoard.getInstance();

        controlBoard.povUp().whileTrue(new DefaultDriveCommand(() -> 0.2, () -> 0.0, () -> 0.0, () -> false));
        controlBoard.povUpRight().whileTrue(new DefaultDriveCommand(() -> 0.2, () -> -0.2, () -> 0.0, () -> false));

        controlBoard.povRight().whileTrue(new DefaultDriveCommand(() -> 0.0, () -> -0.2, () -> 0.0, () -> false));

        controlBoard.povDownRight().whileTrue(new DefaultDriveCommand(() -> -0.2, () -> -0.2, () -> 0.0, () -> false));

        controlBoard.povDown().whileTrue(new DefaultDriveCommand(() -> -0.2, () -> 0.0, () -> 0.0, () -> false));

        controlBoard.povDownLeft().whileTrue(new DefaultDriveCommand(() -> -0.2, () -> 0.2, () -> 0.0, () -> false));

        controlBoard.povLeft().whileTrue(new DefaultDriveCommand(() -> 0.0, () -> 0.2, () -> 0.0, () -> false));

        controlBoard.povUpLeft().whileTrue(new DefaultDriveCommand(() -> 0.2, () -> 0.2, () -> 0.0, () -> false));

        System.out.println("POV Bindings Configured");
    }

    public Command getAutonomousCommand() {
        // autoFactory.setCurrentAuto(Auto.LEFT_LOLI_LEFT_FIRST);
        // autoFactory.recompile();
        // autoFactory.setCompledAuto(new LOLILEFTLeftFirst());
        return autoFactory.getCompiledAuto();
        // return new DeadReckoning();
    }

    public void precompileAuto() {
        if (AutoFactory.getInstance().recompileNeeded()) {
            AutoFactory.getInstance().recompile();
        }
    }

    public static boolean getDeadReckoning() {
        return deadReckoning;
    }

    public static void setDeadReckoning(boolean isDeadReckoning) {
        deadReckoning = isDeadReckoning;
    }
}
