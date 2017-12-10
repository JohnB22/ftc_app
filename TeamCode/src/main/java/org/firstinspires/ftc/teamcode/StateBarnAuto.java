package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by johnb on 11/12/2017.
 */

public class StateBarnAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor spatMotor = null;

    private Servo spatServo = null;


    double servoPos1 = 0.45;
    double servoPos2 = 0.55;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        spatMotor = hardwareMap.get(DcMotor.class, "spat_motor");

        spatServo = hardwareMap.get(Servo.class, "spat_servo");

        waitForStart();
        runtime.reset();


        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        leftDrive.setPower(FORWARD_SPEED);
        rightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        /*
        // Step 2:  Spin right for 1.3 seconds
        leftDrive.setPower(TURN_SPEED);
        rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/

        // Step 4:  Stop and close the claw.
        leftDrive.setPower(0);
        rightDrive.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);


    }
}
