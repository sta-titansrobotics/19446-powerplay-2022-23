package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


@Disabled
public class ColorSensorTest extends LinearOpMode {


    @Override
    public void runOpMode() {



        ColorSensor cSensor = hardwareMap.get(ColorSensor.class, "colorSensor");



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            telemetry.addData("Amount of Red:", cSensor.argb());

            telemetry.update();

        }

    }




}
