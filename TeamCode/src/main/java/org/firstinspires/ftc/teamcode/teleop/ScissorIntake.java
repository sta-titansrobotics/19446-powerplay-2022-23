package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ScissorIntake {

    private Servo rpServo;  // the servo that controls the rack and pinion
    private Servo scissorServo; // servo that controls the scissor mechanism;
    private TouchSensor scissorTouch; // scissor touch sensor
    private ColorSensor colorSensor; // cone touch sensor


    private final double CONE_DISTANCE_THRESHOLD_MM = 5;  // in mm, need to figure out this value
    private final double SCISSOR_HOME_POSITION = 0;
    private final double SCISSOR_CONTRACT_POSITION = 0.8;  // position of servo when closing to pick up the cone
    private final double SCISSOR_WIDE_OPEN_POSITION = -1.0; // position of servo to open wide (not really useful, but anyways)
    private final double RP_TOP_POSITION = -1.0;
    private final double RP_BOTTOM_POSITION = 1.0;
    private final double RP_HOME_POSITION = 0.0;

    /**
     * @param rbtRPServo            - rack & pinion servo from robot (vertical)
     * @param rbtScissorServo       - scissor servo from robot
     * @param rbtScissorTouch       - scissor touch sensor from robot
     */
    public ScissorIntake(Servo rbtRPServo, Servo rbtScissorServo, TouchSensor rbtScissorTouch) {
        this.rpServo = rbtRPServo;
        this.scissorServo = rbtScissorServo;
        this.scissorTouch = rbtScissorTouch;


    }


    /**
     * Check if the scissor touch sensor is press
     *
     * @return true if the scissor touch sensor is pressed
     */
    public boolean isScissorInCone() {
        return this.scissorTouch.isPressed();
    }

    /**
     * Contract scissor arms to for cone pick up
     */
    public void contractScissor() {
        this.scissorServo.setPosition(this.SCISSOR_CONTRACT_POSITION);
    }

    /**
     * Move the scissor up to te top position on the rack and pinion
     */
    public void moveScissorToTop() {
        this.rpServo.setPosition(this.RP_TOP_POSITION);
    }

    /**
     * Move the scissor down to the bottom position on the rack and pinion
     */
    public void moveScissorToBottom() {
        this.rpServo.setPosition(this.RP_BOTTOM_POSITION);
        // maybe add a timeout safety stop here;
    }

    /**
     * Reset the position of the RP servo to home position
     */
    public void moveScissorToHome() {
        this.rpServo.setPosition(this.RP_HOME_POSITION);
        // maybe add a timeout safety stop here;
    }


    /**
     * bring scissor arms to their position, releasing the cone
     */
    public void releaseCone() {
        this.scissorServo.setPosition(0);
    }
}

