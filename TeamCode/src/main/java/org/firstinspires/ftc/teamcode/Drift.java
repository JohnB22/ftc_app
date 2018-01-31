package org.firstinspires.ftc.teamcode;
//imports that are necessary to access.
import android.os.PowerManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Bobby on 1/18/2018.
 */
//The claim of what the Java class is
@TeleOp(name="Drifty Tele-op")
public class Drift extends LinearOpMode {
    //Stating which variables are motors and which are servos.
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Servo driveTurn;
    //@Override means it will not run this code it's is there for accessibility.
    @Override
    //runOpMode is whenever the Driver presses init (initialize) on the Driver station phone.
    public void runOpMode() {
        /*Tell the phones what each motor is named in the configuration file.
        So it can send the power to the correct connector*/
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        driveTurn = hardwareMap.get(Servo.class,"driveTurn");
        /*Set the direction of the motors.
        Because we set the motors on opposite
        sides they have opposite rotation*/
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        driveTurn.setDirection(Servo.Direction.FORWARD);
        //Wait for Driver to press play
        waitForStart();
        //While the play button is pressed it will continually run the code below.
        while (opModeIsActive()){
            double Power;
            double servoPosition;


            Power = gamepad1.right_trigger - gamepad1.left_trigger;
            if (!gamepad1.right_bumper){
                Power = Range.scale(Power, -1,1,-0.5,0.5);
            }if (gamepad1.right_bumper){
                Power = Range.scale (Power, -1,1,-0.75,0.75);
            }
            rightDrive.setPower(Power);
            leftDrive.setPower(Power);

            if (gamepad1.left_stick_x > 1){
                rightDrive.setPower(Power+0.25);
            }if (gamepad1.left_stick_x < -1){
                leftDrive.setPower(Power+0.25);
            }



            servoPosition = gamepad1.left_stick_x;
            servoPosition = Range.scale(servoPosition,-1,1,0.25,0.69);
            driveTurn.setPosition(servoPosition);
        }

    }
}
