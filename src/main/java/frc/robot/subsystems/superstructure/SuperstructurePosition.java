package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.SuperstructureConstants;

public class SuperstructurePosition {
    public enum SuperstructureState {
        // spotless:off
        HOME(1.1, Degrees.of(130.0), IntakePivotPositions.MID.position, SuperstructureStateType.NONE),
        HP(0.75, Degrees.of(133.0), IntakePivotPositions.MID.position, SuperstructureStateType.NONE),
        EXPLODE(3.75, Degrees.of(130.5), IntakePivotPositions.STOW.position, SuperstructureStateType.NONE),
        INTAKE(1.5, Degrees.of(307), IntakePivotPositions.INTAKE.position, SuperstructureStateType.CORAL),
        UN_JAM(12.0, Degrees.of(200.0), IntakePivotPositions.INTAKE.position, SuperstructureStateType.CORAL),
        L1H(4.5, Degrees.of(255.0), IntakePivotPositions.L1.position, SuperstructureStateType.CORAL),
        L2(4.75, Degrees.of(166.0), IntakePivotPositions.STOW.position, SuperstructureStateType.CORAL),
        L3(17.5, Degrees.of(166.0), IntakePivotPositions.STOW.position, SuperstructureStateType.CORAL),
        L4(46.0, Degrees.of(188.5), IntakePivotPositions.STOW.position, SuperstructureStateType.CORAL),
        LOWER_ALGAE(3.5, Degrees.of(265.0), IntakePivotPositions.L1.position, SuperstructureStateType.ALGAE),
        UPPER_ALGAE(17.25, Degrees.of(265.0), IntakePivotPositions.L1.position, SuperstructureStateType.ALGAE),
        SAFE_ARM_HEIGHT(SuperstructureConstants.MIN_SAFE_ROTATION, Degrees.of(228), IntakePivotPositions.STOW.position, SuperstructureStateType.NONE),
        STOW(9.0, Degrees.of(225.0), IntakePivotPositions.L1.position, SuperstructureStateType.NONE), // default state
        SPOOKY_STOW(9.0, Degrees.of(130.0), IntakePivotPositions.STOW.position, SuperstructureStateType.NONE), // default state
        POST_ALGAE_STOW(9.0, Degrees.of(240.0), IntakePivotPositions.L1.position, SuperstructureStateType.NONE),
        ALGAE_NET(46, Degrees.of(190.0), IntakePivotPositions.MID.position, SuperstructureStateType.ALGAE),
        ALGAE_PROCESS(2.25, Degrees.of(325.0), IntakePivotPositions.MID.position, SuperstructureStateType.ALGAE),
        TRAVEL(3.75, Degrees.of(255.0), IntakePivotPositions.L1.position, SuperstructureStateType.ALGAE),
        CLIMB(10.5, Degrees.of(265.0), IntakePivotPositions.L1.position, SuperstructureStateType.NONE);

        // spotless:on
        private final double elevatorPosition;
        private final Angle armPivotAngle;
        private final double intakePosition;
        private final SuperstructureStateType type;

        private SuperstructureState(double elevatorPosition, Angle armPivotAngle, double intakePosition, SuperstructureStateType type) {
            this.elevatorPosition = elevatorPosition;
            this.armPivotAngle = armPivotAngle;
            this.intakePosition = intakePosition;
            this.type = type;
        }

        public double getElevatorPositionRotations() {
            return elevatorPosition;
        }

        public Angle getArmPivotAngle() {
            return armPivotAngle;
        }

        public double getIntakePivotPosition() {
            return intakePosition;
        }

        public SuperstructureStateType getType() {
            return type;
        }
    }

    public enum SuperstructureStateType {
        NONE,
        CORAL,
        ALGAE
    }

    private enum IntakePivotPositions {
        STOW(17.5),
        INTAKE(0.8), // was 75
        L1(15),
        MID(7);

        private double position;

        private IntakePivotPositions(double position) {
            this.position = position;
        }
    }
}
