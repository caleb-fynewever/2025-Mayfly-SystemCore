package frc.robot.commands.arm;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.arm.ArmRollerSubsystem;

public class ArmCommandFactory {
    private static final ArmRollerSubsystem rollers = ArmRollerSubsystem.getInstance();
    private static final ArmPivotSubsystem pivot = ArmPivotSubsystem.getInstance();
    private static final RobotState robotState = RobotState.getInstance();

    public static Command intake() {
        return Commands.either(
                algaeIn(),
                coralIn(),
                robotState.isAlgaeMode());
    }

    public static Command score() {
        return Commands.either(
                algaeOut(),
                scoreCoral(),
                robotState.isAlgaeMode());
    }

    public static Command scoreCoral() {
        return Commands.sequence(
            Commands.deadline(coralIn(), Commands.waitTime(Seconds.of(0.075))),
            Commands.deadline(coralOut(), Commands.waitTime(Seconds.of(0.5)))
        );
    }

    public static Command coralInTap() {
        return Commands.deadline(coralIn(), Commands.waitTime(Seconds.of(0.075)));
    }

    public static Command algaeInTap() {
        return Commands.deadline(algaeIn(), Commands.waitTime(Seconds.of(0.1)));
    }

    public static Command coralIn() {
        return Commands.runEnd(() -> rollers.coralIn(), () -> rollers.stopMotor(), rollers);
    }

    public static Command coralOut() {
        return Commands.runEnd(() -> rollers.coralOut(), () -> rollers.stopMotor(), rollers);
    }

    public static Command algaeIn() {
        return Commands.runEnd(() -> rollers.algaeIn(), () -> rollers.stopMotor(), rollers);
    }

    public static Command algaeOut() {
        return Commands.runEnd(() -> rollers.algaeOut(), () -> rollers.stopMotor(), rollers);
    }

    public static Command setCoast() {
        return Commands.runOnce(() -> pivot.setNeutralMode(NeutralModeValue.Coast), pivot);
    }

    public static Command setBrake() {
        return Commands.runOnce(() -> pivot.setNeutralMode(NeutralModeValue.Brake), pivot);
    }
}
