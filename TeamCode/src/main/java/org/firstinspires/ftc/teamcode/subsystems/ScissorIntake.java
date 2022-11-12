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

    public boolean isConeInScissor(){
        return this.scissorTouch.isPressed();
    }

    public void contractScissor(){
        this.scissorServo.setPosition(this.SCISSOR_CONTRACT_POSITION);
    }

    public void moveScissorToTop(){
        this.rpServo.setPosition(this.RP_TOP_POSITION);
    }

    public void moveScissorToBottom(){
        this.rpServo.setPosition(this.RP_BOTTOM_POSITION);
        // maybe add a timeout safety stop here;
    }









}
