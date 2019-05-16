package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

@TeleOp(name="TestTeleOp", group="Drive-Type OpModes")
public class TestTeleOp extends OpMode
{
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
//    private ElapsedTime runtime = new ElapsedTime(); //We don't really need an elapsed time telemetry, but here it is.
    private DcMotor DriveLeftFront = null; //Left Front Motor
//    private DcMotor DriveRightFront = null; //Right Front Motor
//    private DcMotor DriveLeftRear = null; //Left Rear Motor
//    private DcMotor DriveRightRear = null; //Right Rear Motor

    private DcMotor GenericMotor = null;

    // Init some other variables

    //These outline the starting positions of all of the servos, as well as the range they're allowed to work in.
    //This is in degrees.

    private double motorSpeedMultiplier = .2;
    private String consoleOut = "Nothing Yet";

    @Override
    public void init() {
        // Code to run ONCE when the driver hits INIT

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
//        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");
//        DriveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
//        DriveRightRear = hardwareMap.get(DcMotor.class, "DriveRightRear");
//        DriveRightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Init MotorMineralArmPitch
        GenericMotor = hardwareMap.get(DcMotor.class, "GenericMotor");
        GenericMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GenericMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
        //Code loops once you hit init
        telemetry.addData("Status:", "Armed by in and out");
        //       telemetry.update();
    }


    @Override
    public void start() {
        //Code to run ONCE when the driver hits PLAY
 //       runtime.reset();
    }

    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        // Setup a variable for each drive wheel and servos to save power level for telemetry.

        //Gamepad 1 Controls
        // Original POV Mode written by dmssargent, sourced from the FTC Forum. Modified for use with our robot.
        //POV Mode. One stick controls translation and one controls rotation.

        /*        double r = Math.hypot((gamepad1.left_stick_x * 2.5), gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, (gamepad1.left_stick_x * 2.5)) - Math.PI / 4;
        double rightX = (gamepad1.right_stick_x * .75);
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
*/


        //Controller One
        if (gamepad1.right_trigger < 0.1) {
            motorSpeedMultiplier = .2;
        }

        if (gamepad1.right_trigger >= 0.1) {
            motorSpeedMultiplier = .55; //RETURN TO .55 ASAP
        }

        //Controller Two

        double v5 = -gamepad2.left_stick_y * 0.6; // GenericMotor
        double v6 = -gamepad2.right_stick_y; //empty

        if (gamepad2.dpad_up) {  //empty
        }

        if (gamepad2.dpad_down) { //empty
        }

        if (gamepad2.dpad_left) {  //empty
        }

        if (gamepad2.dpad_right) {  //empty
        }

        if (gamepad2.right_stick_button) {
        }

        if (gamepad2.left_bumper) {
        }

        if (gamepad2.right_bumper) {
        }

        if (gamepad2.a) {
        }
        if (gamepad2.b) {
        }

        if (gamepad2.x) {
        }

        if (gamepad2.y) {
        }

        // Set Servo positions to variable "servoPosition"(s)

 //       GenericMotor.setPosition(servoMineralVacPosition); //a
        // Set Auxiliary Motor Powers
        GenericMotor.setPower(v5);

/*
        // Send calculated power to wheels
        DriveLeftFront.setPower(v1 * motorSpeedMultiplier);
        DriveRightFront.setPower(v2 * motorSpeedMultiplier);
        DriveLeftRear.setPower(v3 * motorSpeedMultiplier);
        DriveRightRear.setPower(v4 * motorSpeedMultiplier);


        //Read Positions Of Motors
        double DriveLeftFrontPos = DriveLeftFront.getCurrentPosition();
        double DriveRightFrontPos = DriveRightFront.getCurrentPosition();
        double DriveLeftRearPos = DriveLeftRear.getCurrentPosition();
        double DriveRightRearPos = DriveRightRear.getCurrentPosition();
*/




        // Show the elapsed game time and wheel power.
//        telemetry.addData("Status", "Running, Run Time: " + runtime.toString());

/*        telemetry.addData("FrontMotors", "Left: " + DriveLeftFrontPos);
        telemetry.addData("FrontMotors", "Right: " + DriveRightFrontPos);
        telemetry.addData("RearMotors", "Left: "+ DriveLeftRearPos);
        telemetry.addData("RearMotors", "Right: " + DriveRightRearPos);
*/
        telemetry.addData( "Motor Speed","%.2f", motorSpeedMultiplier);
        telemetry.addData( "Console in and out burger", consoleOut);
    }


    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP.

        telemetry.addData("Status:", "Stopped");

    }

}