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

@TeleOp(name="TestTeleOp", group="Drive-Type OpModes")

public class TestTeleOp extends OpMode
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



    //encoderDrive constants
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_DEGREE =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) * 1/6;

    private DcMotor MotorMineralArmPitch = null;
    //private DcMotor MotorMineralArmExtension = null;

    private Servo MineralVac = null; //Continuous rotation servo for Mineral Collection
    private Servo MineralVacWrist = null; //Servo for Mineral Collector "Wrist"


    private DeviceInterfaceModule DeviceIM;
    private GyroSensor Gyro = null; // Das ist die Gyro

    //These outline the starting positions of all of the servos, as well as the range they're allowed to work in.
    //This is in degrees.
    private double servoMineralVacPosition = 0; //Use .45 for reverse, .5 for stop, and .55 for forwards
    private double servoMineralVacWristPosition = 0;
    private double servoMineralVacBucketPosition = 0;
    private final static double servoMinRange  = 1;
    private final static double servoMaxRange  = 180;
    private double motorSpeedMultiplier = .2;
    private String consoleOut = "Nothing Yet";

    @Override
    public void init() {
        // Code to run ONCE when the driver hits INIT

        //This is a comment

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");
        DriveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
        DriveRightRear = hardwareMap.get(DcMotor.class, "DriveRightRear");
        DriveRightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Init MotorMineralArmPitch
        MotorMineralArmPitch = hardwareMap.get(DcMotor.class, "MotorMineralArmPitch");
        MotorMineralArmPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorMineralArmPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Init MotorMineralArmExtension
        //MotorMineralArmExtension = hardwareMap.get(DcMotor.class, "MotorMineralArmExtension");
        //MotorGlyphGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //MotorGlyphGrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Init MotorRobotLifter
        //MotorRobotLifter = hardwareMap.get(DcMotor.class, "MotorRobotLifter");
        //MotorRobotLifter.setmode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //MotorRobotLifter

        //Init MineralVacServos
        MineralVac = hardwareMap.get(Servo.class, "MineralVac");
        MineralVacWrist = hardwareMap.get(Servo.class, "MineralVacWrist");


        //Init Misc Devices

        DeviceIM = hardwareMap.get(DeviceInterfaceModule.class, "DIM");
        Gyro = hardwareMap.get(GyroSensor.class, "Gyro");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
        //Code loops once you hit init
        telemetry.addData("Status:", "Armed");

    }


    @Override
    public void start() {
        //Code to run ONCE when the driver hits PLAY
        runtime.reset();
        Gyro.calibrate();
    }

    private boolean UseServoStick = true;
    private boolean DriveMode = false; //Currently Unused, set for 2 drive modes
    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        // Setup a variable for each drive wheel and servos to save power level for telemetry.
        //double RelicGrabberExtensionPos = MotorRelicExtension.getCurrentPosition();
        //double MotorGlyphGrabberPos = MotorGlyphGrabber.getCurrentPosition();

        // Init some local variables.

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




        if (gamepad1.right_trigger < 0.1) {
            motorSpeedMultiplier = .2;
            DeviceIM.setLED(1, false);
        }

        if (gamepad1.right_trigger >= 0.1) {
            motorSpeedMultiplier = 1.00; //RETURN TO .55 ASAP
            DeviceIM.setLED(1, true);
        }

        //Gamepad 2 Controls

        double v5 = -gamepad2.left_stick_y * 0.6;

        if (gamepad2.dpad_up) {

        }

        if (gamepad2.dpad_down) {

        }

        if (gamepad2.dpad_left) {

        }

        if (gamepad2.dpad_right) {
            if (DriveMode){
                DriveMode = false;
                consoleOut = "Set Drive Mode to Mode 1";
            } else {
                DriveMode = true;
                consoleOut = "Set Drive Mode to Mode 2";
            }
        }

        //Toggle the Right Stick controlling the glyph grabber servos.
        if (gamepad2.right_stick_button) {

        }

        if (gamepad2.left_bumper) {

        }

        if (gamepad2.right_bumper) {

        }

        if (gamepad2.a) {
            servoMineralVacPosition = 1;
        }

        if (gamepad2.b) {
            servoMineralVacPosition = .5;
        }

        if (gamepad2.x) {

        }

        if (gamepad2.y) {

        }

        if (gamepad1.a) {
            encoderDrive(1,2);
        }

        // Set Servo positions to variable "servoPosition"(s)
        servoMineralVacPosition = Range.clip(servoMineralVacPosition, 0, 180);
        MineralVac.setPosition(servoMineralVacPosition);

        servoMineralVacWristPosition = Range.clip(servoMineralVacWristPosition, servoMinRange, servoMaxRange); //Clips servo range into usable area. Protects from over extension.
        MineralVacWrist.setPosition(servoMineralVacWristPosition / 180); //This converts from degrees into 0-1 automagically.

        // Send calculated power to wheels
        DriveLeftFront.setPower(v1 * motorSpeedMultiplier);
        DriveRightFront.setPower(v2 * motorSpeedMultiplier);
        DriveLeftRear.setPower(v3 * motorSpeedMultiplier);
        DriveRightRear.setPower(v4 * motorSpeedMultiplier);
        MotorMineralArmPitch.setPower(v5);

        //Read Positions Of Motors + Gyro
        double DriveLeftFrontPos = DriveLeftFront.getCurrentPosition();
        double DriveRightFrontPos = DriveRightFront.getCurrentPosition();
        double DriveLeftRearPos = DriveLeftRear.getCurrentPosition();
        double DriveRightRearPos = DriveRightRear.getCurrentPosition();
        int GyroPos = Gyro.getHeading();





        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running, Run Time: " + runtime.toString());

        telemetry.addData("FrontMotors", "Left: " + DriveLeftFrontPos);
        telemetry.addData("FrontMotors", "Right: " + DriveRightFrontPos);
        telemetry.addData("RearMotors", "Left: "+ DriveLeftRearPos);
        telemetry.addData("RearMotors", "Right: " + DriveRightRearPos);

        telemetry.addData("GyroPos:", GyroPos);
        telemetry.addData( "Motor Speed","%.2f", motorSpeedMultiplier);
        telemetry.addData( "Console Out", consoleOut);
    }

    public void encoderDrive(double speed,
                             double leftBackInches) {
        int MineralPitchTarget;


        //Ensures OpMode is running

            // Determine new target position, and pass to motor controller
            MineralPitchTarget = MotorMineralArmPitch.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);

            MotorMineralArmPitch.setTargetPosition(MineralPitchTarget);

            // Turn On RUN_TO_POSITION
            MotorMineralArmPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("encoderDriveOut", "encoderDrive starting");
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();

            MotorMineralArmPitch.setPower(speed);

            // Stop all motion;
            runtime.reset();
            MotorMineralArmPitch.setPower(0);


            // Turn off RUN_TO_POSITION
            MotorMineralArmPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }

    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP.

        telemetry.addData("Status:", "Stopped");

    }

}