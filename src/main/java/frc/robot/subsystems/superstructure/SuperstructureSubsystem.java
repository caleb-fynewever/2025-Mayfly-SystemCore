package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.SuperstructureStateType;
import frc.robot.subsystems.superstructure.SuperstructurePosition.SuperstructureState;

public class SuperstructureSubsystem extends SubsystemBase {

    private static SuperstructureSubsystem INSTANCE;

    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private ArmPivotSubsystem armPivot = ArmPivotSubsystem.getInstance();
    private IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();
    private RobotState robotState = RobotState.getInstance();

    private SuperstructureState unconfirmedState = SuperstructureState.EXPLODE;
    private SuperstructureState goalState = SuperstructureState.EXPLODE;

    private SuperstructureState previousState;
    private boolean isChangingState;

    private boolean cancelHome = false;
    private boolean algaeScoreDownNeeded = false;
    private boolean movingFromIntake = false;

    /** Private constructor to prevent instantiation. */
    private SuperstructureSubsystem() {
        previousState = getUnconfirmedState();
        // pushChangedValueToShuffleboard(previousAction);
        isChangingState = false;
    }

    /** Public method to provide access to the instance. */
    public static SuperstructureSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SuperstructureSubsystem();
        }
        return INSTANCE;
    }

    public SuperstructureState getLatestAction() {
        return previousState;
    }

    // private void pushChangedValueToShuffleboard(TargetAction action) {
    //     switch (action) {
    //         case L1H:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.L1);
    //             break;
    //         case L2:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.L2);
    //             break;
    //         case L3:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.L3);
    //             break;
    //         case L4:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.L4);
    //             break;
    //         case UPPER_ALGAE:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.A2);
    //             break;
    //         case LOWER_ALGAE:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.A1);
    //             break;
    //         default:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.NONE);
    //             break;
    //     }
    // }

    public Command setSelected(SuperstructureState unconfirmedState, boolean autoConfirm) {
        return Commands.runOnce(() -> setUnconfirmedState(unconfirmedState, autoConfirm));
    }

    private void setUnconfirmedState(SuperstructureState unconfirmedState, boolean autoConfirm) {
        this.unconfirmedState = unconfirmedState;
        if(autoConfirm) {
            confirmSelectedAction();
        }
        // pushChangedValueToShuffleboard(selectedTargetAction);
        revealCombination();
    }

    public Command setGoal(SuperstructureState goalState) {
        return Commands.runOnce(() -> setGoalState(goalState));
    }

    private void setGoalState(SuperstructureState goalState) {
        this.goalState = goalState;
    }

    public Command confirm() {
        return Commands.run(() -> confirmSelectedAction());
    }

    private void confirmSelectedAction() {
        goalState = unconfirmedState;
        movingFromIntake = false;
        revealCombination();
    }

    public SuperstructureState getUnconfirmedState() {
        return unconfirmedState;
    }

    public SuperstructureState getGoalState() {
        return goalState;
    }

    public void revealCombination() {
        System.out.println("Unconfirmed State : " + getUnconfirmedState().toString());
    }

    public void stow() {
        setGoalState(getStowFromGoalState());
    }

    @Override
    public void periodic() {
        robotState.setAlgaeMode(goalState.getType() == SuperstructureStateType.ALGAE);
        isChangingState = goalState != previousState;
        algaeScoreDownNeeded = goalState == SuperstructureState.ALGAE_NET;
        cancelHome = isChangingState && goalState != SuperstructureState.HOME;

        if (IntakeRollerSubsystem.getInstance().isHoldingCoral()) {
            setGoalState(SuperstructureState.TRAVEL);
        } else if (!movingFromIntake
                && robotState.getHasCoral()
                && (goalState == SuperstructureState.INTAKE)
                && armPivot.atPosition(goalState)) {
            movingFromIntake = true;
            setGoalState(DriverStation.isAutonomous() ? SuperstructureState.L3 : SuperstructureState.STOW);
        }

        if (armPivot.atPosition(SuperstructureState.STOW)) {
            movingFromIntake = false;
        }

        if (isChangingState) {
            if (cancelHome) {
                elevator.setWantHome(false);
                cancelHome = false;
            } else if (goalState == SuperstructureState.HOME && !elevator.isHoming()) {
                elevator.setWantHome(true);
                intakePivot.setAngle(SuperstructureState.HOME.getIntakePivotPosition());
                armPivot.setArmPosition(SuperstructureState.HOME);
                System.out.println("HOMING");
                return;
            }

            boolean armCrossingDanger = willArmCrossDangerZone(
                    armPivot.getArmAngle().in(Degrees),
                    goalState.getArmPivotAngle().in(Degrees));

            if (goalState.getElevatorPositionRotations() < SuperstructureConstants.MIN_SAFE_ROTATION
                    && elevator.getPosition() < SuperstructureConstants.MIN_SAFE_ROTATION
                    && armCrossingDanger) {
                intakePivot.setPosition(goalState);

                if (!armPivot.isAtPosition(5, goalState.getArmPivotAngle())) {
                    elevator.setPositionMotionMagic(SuperstructureState.SAFE_ARM_HEIGHT);
                }

                if (elevator.atPosition(1.0, SuperstructureState.SAFE_ARM_HEIGHT)) {
                    armPivot.setArmPosition(goalState);
                    if (armPivot.isAtPosition(5, goalState.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(goalState);
                    }
                }
            } else if (goalState.getElevatorPositionRotations() < elevator.getPosition()) {
                if (algaeScoreDownNeeded) {
                    armPivot.setArmPosition(goalState);
                    intakePivot.setPosition(goalState);

                    if (armPivot.isAtPosition(3, goalState.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(goalState);
                        algaeScoreDownNeeded = false;
                    }
                }
                if (goalState.getElevatorPositionRotations() > SuperstructureConstants.MIN_SAFE_ROTATION
                        || elevator.getPosition() > SuperstructureConstants.MIN_MOVE_ROTATION) {
                    elevator.setPositionMotionMagic(goalState);
                    armPivot.setArmPosition(goalState);
                    intakePivot.setPosition(goalState);
                } else {
                    armPivot.setArmPosition(goalState);
                    intakePivot.setPosition(goalState);

                    if (armPivot.isAtPosition(10, goalState.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(goalState);
                    }
                }
            } else if (goalState.getElevatorPositionRotations() > elevator.getPosition()) {
                elevator.setPositionMotionMagic(goalState);
                intakePivot.setPosition(goalState);

                if (elevator.atPosition(20, goalState)) {
                    armPivot.setArmPosition(goalState);
                }
            }

            if (elevator.atPosition(goalState)
                    && armPivot.isAtPosition(5, goalState.getArmPivotAngle())) {
                isChangingState = false;
            } 
        }

        previousState = goalState;
    }

    public boolean willArmCrossDangerZone(double currentDeg, double goalDeg) {
        if (goalDeg > SuperstructureConstants.LEFT_LIMIT && currentDeg < SuperstructureConstants.RIGHT_LIMIT) {
            return true;
        } else if (goalDeg < SuperstructureConstants.RIGHT_LIMIT && currentDeg > SuperstructureConstants.LEFT_LIMIT) {
            return true;
        } else if (currentDeg > SuperstructureConstants.RIGHT_LIMIT
                && currentDeg < SuperstructureConstants.LEFT_LIMIT
                && (goalDeg > SuperstructureConstants.LEFT_LIMIT || goalDeg < SuperstructureConstants.RIGHT_LIMIT)) {
            return true;
        }

        return false;
    }

    public boolean atConfirmedPosition() {
        return atPosition(goalState);
    }

    public boolean atPosition(SuperstructureState state) {
        return elevator.atPosition(state)
                && armPivot.atPosition(state)
                && intakePivot.atPosition(state);
    }

    public SuperstructureState getStowFromGoalState() {
        if (goalState == SuperstructureState.ALGAE_NET) {
            return SuperstructureState.POST_ALGAE_STOW;
        } else if (goalState == SuperstructureState.ALGAE_PROCESS) {
            return SuperstructureState.ALGAE_PROCESS;
        } else {
            return SuperstructureState.STOW;
        }
    }
}
