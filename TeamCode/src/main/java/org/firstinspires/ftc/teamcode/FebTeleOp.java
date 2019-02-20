package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

public class FebTeleOp extends OpMode
{
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
    private ElapsedTime runtime = new ElapsedTime(); //We don't really need an elapsed time telemetry, but here it is.
    private DcMotor DriveLeftFront = null; //Left Front Motor
    private DcMotor DriveRightFront = null; //Right Front Motor
    private DcMotor DriveLeftRear = null; //Left Rear Motor
    private DcMotor DriveRightRear = null; //Right Rear Motor

    private DcMotor MotorMineralArmPitch = null;

    private Servo MineralVac = null; //Continuous rotation servo for Mineral Collection

    // Init some other variables

    //These outline the starting positions of all of the servos, as well as the range they're allowed to work in.
    //This is in degrees.
    private static double CRServoStop = .5  ; //Should be the stop position of the servos
    private double servoMineralVacPosition = CRServoStop;

    private final static double servoMinRange  = 1;
    private final static double servoMaxRange  = 180;

    private double motorSpeedMultiplier = .2;
    private String consoleOut = "Nothing Yet";

    @Override
    public void init() {
        // Code to run ONCE when the driver hits INIT

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");
//        DriveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
        DriveRightRear = hardwareMap.get(DcMotor.class, "DriveRightRear");
//        DriveRightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Init MotorMineralArmPitch
        MotorMineralArmPitch = hardwareMap.get(DcMotor.class, "MotorMineralArmPitch");
        MotorMineralArmPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorMineralArmPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Init MineralVacServos
        MineralVac = hardwareMap.get(Servo.class, "MineralVac");


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
        //Code loops once you hit init
        telemetry.addData("Status:", "Armed by saira");
 //       telemetry.update();
    }


    @Override
    public void start() {
        //Code to run ONCE when the driver hits PLAY
        runtime.reset();
    }

    private boolean MineralVacForward = false;
    private boolean MineralVacBackwards = false;
    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        // Setup a variable for each drive wheel and servos to save power level for telemetry.

        //Gamepad 1 Controls
        // Original POV Mode written by dmssargent, sourced from the FTC Forum. Modified for use with our robot.
        //POV Mode. One stick controls translation and one controls rotation.
        double r = Math.hypot((gamepad1.left_stick_x * 2.5), gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, (gamepad1.left_stick_x * 2.5)) - Math.PI / 4;
        double rightX = (gamepad1.right_stick_x * .75);
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;



        //Controller One
        if (gamepad1.right_trigger < 0.1) {
            motorSpeedMultiplier = .2;
        }

        if (gamepad1.right_trigger >= 0.1) {
            motorSpeedMultiplier = .55; //RETURN TO .55 ASAP
         }

        //Controller Two

        double v5 = -gamepad2.left_stick_y * 0.6; //MotorMineralArmPitch
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
            if (MineralVacForward) {
                servoMineralVacPosition = CRServoStop;
                MineralVacForward = false;
                MineralVacBackwards = false; //Reset for safety
            } else {
                servoMineralVacPosition = 1;
                MineralVacForward = true;
            }
        }
        if (gamepad2.b) {
            if (MineralVacBackwards) {
                servoMineralVacPosition = CRServoStop;
                MineralVacBackwards = false;
                MineralVacForward = false; //Reset for safety
            }
            else {
                servoMineralVacPosition = 0;
                MineralVacBackwards = true;
            }
        }

        if (gamepad2.x) {
        }

        if (gamepad2.y) {
        }

        // Set Servo positions to variable "servoPosition"(s)

         MineralVac.setPosition(servoMineralVacPosition); //Why hello Jeffery! I have taken over your code! I will rule them all! Mwah ha ha!!

        // Send calculated power to wheels
        DriveLeftFront.setPower(v1 * motorSpeedMultiplier);
        DriveRightFront.setPower(v2 * motorSpeedMultiplier);
        DriveLeftRear.setPower(v3 * motorSpeedMultiplier);
        DriveRightRear.setPower(v4 * motorSpeedMultiplier);

        // Set Auxiliary Motor Powers
        MotorMineralArmPitch.setPower(v5);

        //Read Positions Of Motors
        double DriveLeftFrontPos = DriveLeftFront.getCurrentPosition();
        double DriveRightFrontPos = DriveRightFront.getCurrentPosition();
        double DriveLeftRearPos = DriveLeftRear.getCurrentPosition();
        double DriveRightRearPos = DriveRightRear.getCurrentPosition();





        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running, Run Time: " + runtime.toString());

        telemetry.addData("FrontMotors", "Left: " + DriveLeftFrontPos);
        telemetry.addData("FrontMotors", "Right: " + DriveRightFrontPos);
        telemetry.addData("RearMotors", "Left: "+ DriveLeftRearPos);
        telemetry.addData("RearMotors", "Right: " + DriveRightRearPos);

        telemetry.addData( "Motor Speed","%.2f", motorSpeedMultiplier);
        telemetry.addData( "Console in and out burger", consoleOut);
     }


    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP.

        telemetry.addData("Status:", "Stopped");

    }

}
