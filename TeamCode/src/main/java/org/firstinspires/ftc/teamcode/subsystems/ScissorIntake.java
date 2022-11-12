package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ScissorIntake {

    private Servo rpServo;  // the servo that controls the rack and pinion
    private Servo scissorServo; // servo that controls the scissor mechanism;
    private TouchSensor scissorTouch; // scissor touch sensor
    private DistanceSensor coneDistanceSensor; // cone touch sensor

    private final double CONE_DISTANCE_THRESHOLD_MM = 5;  // in mm, need to figure out this value
    private final double SCISSOR_HOME_POSITION = 0;
    private final double SCISSOR_CONTRACT_POSITION = 0.8;  // position of servo when closing to pick up the cone
    private final double SCISSOR_WIDE_OPEN_POSITION = -1.0; // position of servo to open wide (not really useful, but anyways)
    private final double RP_TOP_POSITION = -1.0;
    private final double RP_BOTTOM_POSITION = 1.0;
    private final double RP_HOME_POSITION = 0.0;

    public ScissorIntake(Servo rbtRPServo, Servo rbtScissorServo, TouchSensor rbtScissorTouch, DistanceSensor rbtConeDistanceSensor){
        this.rpServo = rbtRPServo;
        this.scissorServo = rbtScissorServo;
        this.scissorTouch = rbtScissorTouch;
        this.coneDistanceSensor = rbtConeDistanceSensor;
    }

    /**
     * Check if the cone is placed in the bracket
     * @return true if the cone distance is less than the determined threshold
     */
    public boolean isConeInBracket(){
        return this.coneDistanceSensor.getDistance(DistanceUnit.MM) < this.CONE_DISTANCE_THRESHOLD_MM;
    }

    /**
     * Check if the scissor touch sensor is press
     * @return true if the scissor touch sensor is pressed
     */
    public boolean isConeInScissor(){
        return this.scissorTouch.isPressed();
    }

    /**
     * Contract scissor arms to for cone pick up
     */
    public void contractScissor(){
        this.scissorServo.setPosition(this.SCISSOR_CONTRACT_POSITION);
    }

    /**
     * Move the scissor up to te top position on the rack and pinion
     */
    public void moveScissorToTop(){
        this.rpServo.setPosition(this.RP_TOP_POSITION);
    }

    /**
     * Move the scissor down to the bottom position on the rack and pinion
     */
    public void moveScissorToBottom(){
        this.rpServo.setPosition(this.RP_BOTTOM_POSITION);
        // maybe add a timeout safety stop here;
    }

    /**
     * Reset the position of the RP servo to home position
     */
    public void moveScissorToHome(){
        this.rpServo.setPosition(this.RP_HOME_POSITION);
        // maybe add a timeout safety stop here;
    }

    /**
     * Automated action to pick up the cone
     * @return true if the action completes
     */
    public boolean autoPickUpCone(){
        // move to home position
        if (this.rpServo.getPosition() != this.RP_HOME_POSITION){
            this.rpServo.setPosition(this.RP_HOME_POSITION);
        }

        // move the scissor down until bottom or cone detected in scissor
        if (isConeInBracket()){
            while (this.rpServo.getPosition() > this.RP_BOTTOM_POSITION || !this.isConeInScissor()){
                this.rpServo.setPosition(this.rpServo.getPosition() + 0.1);
            }

            if (this.isConeInScissor()){
                this.contractScissor();
                this.moveScissorToTop();
                return true;
            }
        }

        return false;
    }


    public void releaseCone(){
        this.scissorServo.setPosition(0);
    }
















}
