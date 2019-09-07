package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.GyroSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.util.Range;

        import static android.os.SystemClock.sleep;

@TeleOp(name="OutTeleOp", group="Drive-Type OpModes")


public class OutTeleOp extends OpMode
{
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
    private ElapsedTime runtime = new ElapsedTime(); //We don't really need an elapsed time telemetry, but here it is.
    private DcMotor DriveMotor1 = null; // front right
    private DcMotor DriveMotor2 = null; // front left
    private DcMotor DriveMotor3 = null; // back right
    private DcMotor DriveMotor4 = null; // back left
    private DcMotor LMotorR = null; // launch motor right
    private DcMotor LMotorL = null; // launch motor left
    private DcMotor AimMotor = null;  // aim motor
    private DcMotor MopMotor = null;  // pick up system
    private Servo Servo = null

            // Init some other variables

            ;

    private double motorSpeed = 1;
    private String consoleOut = "Nothing Yet";
    private double motorSpeedMultiplier = .2;
    private double AmotorSpeedMultiplier = .15;

    @Override
    public void init() {
        // Code to run ONCE when the driver hits INIT

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveMotor1 = hardwareMap.get(DcMotor.class, "DriveMotor1");
        DriveMotor2 = hardwareMap.get(DcMotor.class, "DriveMotor2");
        DriveMotor3 = hardwareMap.get(DcMotor.class, "DriveMotor3");
        DriveMotor4 = hardwareMap.get(DcMotor.class, "DriveMotor4");
        LMotorR = hardwareMap.get(DcMotor.class, "LMotorR");
        LMotorL = hardwareMap.get(DcMotor.class, "LMotorL");
        AimMotor = hardwareMap.get(DcMotor.class, "AimMotor");
        MopMotor = hardwareMap.get(DcMotor.class, "MopMotor");
        Servo = hardwareMap.get(Servo.class, "Servo");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized again");

        //Servo = hardwareMap.get(Servo.class, "TestServo");


    }


    @Override
    public void init_loop() {
        //Code loops once you hit init
        telemetry.addData("Status:", "Armed by Ken");
//       telemetry.update();
    }


    @Override
    public void start() {
        //Code to run ONCE when the driver hits PLAY
        runtime.reset();
    }


    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        // Setup a variable for each drive wheel and servos to save power level for telemetry.

        //Gamepad 1 Controls
        // Original POV Mode written by dmssargent, sourced from the FTC Forum. Modified for use with our robot.
        //POV Mode. One stick controls translation and one controls rotation.


        // 4 Motor controller

        if (gamepad1.left_stick_y > 0.5) {      //Forward
            DriveMotor1.setPower(5);
            DriveMotor2.setPower(-5);
            DriveMotor3.setPower(5);
            DriveMotor4.setPower(-5);
        } else if (gamepad1.left_stick_y < -0.5) {       //Backwards
            DriveMotor1.setPower(-5);
            DriveMotor2.setPower(5);
            DriveMotor3.setPower(-5);
            DriveMotor4.setPower(5);
        } else if (gamepad1.left_stick_x < -0.5) {      //Right Translation
            DriveMotor1.setPower(-5);
            DriveMotor2.setPower(-5);
            DriveMotor3.setPower(5);
            DriveMotor4.setPower(5);
        } else if (gamepad1.left_stick_x > 0.5) {       //Left Translation
            DriveMotor1.setPower(5);
            DriveMotor2.setPower(5);
            DriveMotor3.setPower(-5);
            DriveMotor4.setPower(-5);
        } else if (gamepad1.right_stick_x > 0.5) {      //Left
            DriveMotor1.setPower(-5);
            DriveMotor2.setPower(-5);
            DriveMotor3.setPower(-5);
            DriveMotor4.setPower(-5);
        } else if (gamepad1.right_stick_x < -0.5) {       //Right
            DriveMotor1.setPower(5);
            DriveMotor2.setPower(5);
            DriveMotor3.setPower(5);
            DriveMotor4.setPower(5);
        } else {    // 0 set power for the 4 train drive system
            motorSpeed = 0;
            DriveMotor1.setPower(0);
            DriveMotor2.setPower(0);
            DriveMotor3.setPower(0);
            DriveMotor4.setPower(0);

        }
        if (gamepad2.x) {     // launch system
            LMotorL.setPower(-5);
            LMotorR.setPower(5);
        }

        if (gamepad2.b) {
            LMotorL.setPower(0);
            LMotorR.setPower(0);
        }


        if (gamepad2.left_stick_y > 0.5) {    //aim motor to aim the launch system
            AimMotor.setPower(.75);
        }
        else if (gamepad2.left_stick_y < -0.5) {
            AimMotor.setPower(-.75);
        }
        else {
            motorSpeed = 0;
            AimMotor.setPower(0);
        }

        if (gamepad2.right_stick_y > 0.5) {    //mop motor to mop the
            MopMotor.setPower(.75);
        }
        else if (gamepad2.right_stick_y < -0.5) {
            MopMotor.setPower(-.75);
        }
        else {
            motorSpeed = 0;
            MopMotor.setPower(0);
        }

        if (gamepad2.y) {      // servo for the pooper scooper
            // move to 0 degrees.
            Servo.setPosition(0);
        } else if (gamepad2.a) {
            // move to 90 degrees.
            Servo.setPosition(1);
        }







        telemetry.addData("Servo Position", Servo.getPosition());
        telemetry.addData("Status", "Running");
        telemetry.update();



    }}














//DeviceIM.setLED(2,true);

//Controller Two


// Set Servo positions to variable "servoPosition"(s)



// Send calculated power to wheels

// Set Auxiliary Motor Powers


//Read Positions Of Motors
// double Pos1 = DriveMotorL.getCurrentPosition();
// double Pos2 = DriveMotorR.getCurrentPosition();
// double Pos3 = DriveMotor3.getCurrentPosition();
// double Pos4 = DriveMotor4.getCurrentPosition();





// Show the elapsed game time and wheel power.
    /*telemetry.addData("Status", "Running, Run Time: " + runtime.toString());

     telemetry.addData("GenericMotor posL" ,Pos2);


     telemetry.addData( "Motor Speed","%.2f", motorSpeed);

     telemetry.addData( "Right_Stick" ,gamepad1.right_stick_y);
 }


 //@Override
 //public void stop() {
     //Code to run ONCE after the driver hits STOP.

//       telemetry.addData("Status:", "Stopped");

 }



*/


