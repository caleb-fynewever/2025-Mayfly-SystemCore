package frc.robot.util.io;

import com.ctre.phoenix6.CANBus;
import com.team2052.lib.motor.CANItem;

public class Ports {
    /*
     *  CAN IDS
     */
    public static final CANBus LOWLOOP = new CANBus("*");
    /*
     * Bus: SystemCore
     */
    public static final CANBus MAINLOOP = CANBus.systemCore(0);
    public static final CANItem ARM_CANCODER_ID = new CANItem(1, MAINLOOP);
    public static final CANItem ARM_TALONFX_ID = new CANItem(2, MAINLOOP);

    public static final CANItem ARM_ROLLER_TALONFX_ID = new CANItem(3, MAINLOOP);

    public static final CANItem INTAKE_ROLLER_ID = new CANItem(5, MAINLOOP);
    public static final CANItem INTAKE_PIVOT_ID = new CANItem(6, MAINLOOP);
    public static final CANItem INTAKE_ENCODER_ID = new CANItem(7, MAINLOOP);

    /*
     *  Bus: Krawlivore
     */
    public static final int ELEVATOR_FRONT_ID = 14;
    public static final int ELEVATOR_BACK_ID = 15;

    public static final int CLIMBER_ID = 16;

    /*
     *  DIO
     */
    public static final int CORAL_BEAM_BREAK_PIN = 1;
    public static final int INTAKE_BEAM_BREAK_ID = 2;

    public static final int LED_CHANNEL_1_PIN = 5;
    public static final int LED_CHANNEL_2_PIN = 6;
    public static final int LED_CHANNEL_3_PIN = 7;
    public static final int LED_CHANNEL_4_PIN = 8;
    public static final int LED_CHANNEL_5_PIN = 9;

    /*
     *  USB
     */

    public static final int GAMEPAD_PORT = 0;
    public static final int TRANSLATION_JOYSTICK_PORT = 0;
    public static final int ROTATION_JOYSTICK_PORT = 1;
    public static final int CONTROL_PANEL_PORT = 2;
}
