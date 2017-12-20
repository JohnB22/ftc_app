package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Guard on 12/16/2017.
 */
@TeleOp(name = "Controller Tests")
public class ControllerTests extends LinearOpMode{

    @Override
    public void runOpMode() {


        telemetry.addData("RightTrigger:", gamepad1.right_trigger);
        telemetry.addData("LeftTrigger:", gamepad1.left_trigger);
        telemetry.addData("RightStick", gamepad1.right_stick_x + gamepad1.right_stick_y);
        telemetry.addData("LeftStick", gamepad1.left_stick_y + gamepad1.left_stick_x);

    }
}

