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

@TeleOp(name = "SkyTeleOp", group = "Drive-Type OpModes")

public class SkyTeleOp extends OpMode {
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
    private ElapsedTime runtime = new ElapsedTime();
    //drive motors
    private DcMotor DriveMotor1 = null; // front right
    private DcMotor DriveMotor2 = null; // front left
    private DcMotor DriveMotor3 = null; // back right
    private DcMotor DriveMotor4 = null;
    // block grabbing motors
    private DcMotor Up= null;// back left
    private DcMotor PickUp = null;  // pick up system
    private DcMotor PickUp2 = null;  // pick up system
    private DcMotor Lifter = null;

    //servos
    private Servo Servo1 = null;
    private Servo Servo2 = null;  //CR
    private Servo ServoA = null;
    private Servo ServoB = null;
    private Servo ServoClaw= null;  // CR
    private Servo ServoFlip= null;

    private double Speed = .25;
    private double Soft = 0;

    private boolean Flipout= false;
   // saira 1/18 11:00 am

    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveMotor1 = hardwareMap.get(DcMotor.class, "DriveMotor1");
        DriveMotor2 = hardwareMap.get(DcMotor.class, "DriveMotor2");
        DriveMotor3 = hardwareMap.get(DcMotor.class, "DriveMotor3");
        DriveMotor4 = hardwareMap.get(DcMotor.class, "DriveMotor4");
        Up = hardwareMap.get(DcMotor.class, "Up");
        PickUp = hardwareMap.get(DcMotor.class, "PickUp");
        PickUp2 = hardwareMap.get(DcMotor.class, "PickUp2");
         Lifter = hardwareMap.get(DcMotor.class, "Lifter");
         ServoA = hardwareMap.get(Servo.class, "ServoA");
         ServoB = hardwareMap.get(Servo.class, "ServoB");
        ServoClaw= hardwareMap.get(Servo.class, "ServoClaw");
        ServoFlip= hardwareMap.get(Servo.class, "ServoFlip"); //Formerly ServoL2
        Servo1= hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");

        ServoA.setPosition(.75); // Sets servo grabber open
        ServoB.setPosition(0);
        Servo1.setPosition(.5); // CR servo
        Servo2.setPosition(.5); //  CR servo
        ServoClaw.setPosition(.55); // holds claw closed (BRO CR SERVO too)
        ServoFlip.setPosition(.5);  // CR servo
        telemetry.addData("Status", "Initialized again");


    }


    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        // 4 Motor controller
        if (gamepad1.x) {
            Speed = .15;

        }
        if (gamepad1.b) {
            Speed = 1;
        }

        if ((gamepad1.left_stick_y > 0.25) || (gamepad1.left_stick_y < -0.25)) {      //Forward/backwards
            if (Soft < 1) Soft = Soft + 0.02;
            DriveMotor1.setPower(-Speed * Soft * gamepad1.left_stick_y);
            DriveMotor2.setPower(-Speed * Soft * gamepad1.left_stick_y);
            DriveMotor3.setPower(Speed * Soft * gamepad1.left_stick_y);
            DriveMotor4.setPower(Speed * Soft * gamepad1.left_stick_y);
        } else if ((gamepad1.left_stick_x > 0.25) || (gamepad1.left_stick_x < -0.25)) {      // Translation
            if (Soft < 1) Soft = Soft + 0.02;
            DriveMotor1.setPower(Speed * Soft * gamepad1.left_stick_x);
            DriveMotor2.setPower(-Speed * Soft * gamepad1.left_stick_x);
            DriveMotor3.setPower(Speed * Soft * gamepad1.left_stick_x);
            DriveMotor4.setPower(-Speed * Soft * gamepad1.left_stick_x);
        } else if ((gamepad1.right_stick_x > 0.25) || (gamepad1.right_stick_x < -0.25)) {      //turning
           if (Soft < 1) Soft = Soft + 0.02;
            DriveMotor1.setPower(Speed * Soft * gamepad1.right_stick_x);
            DriveMotor2.setPower(Speed * Soft * gamepad1.right_stick_x);
            DriveMotor3.setPower(Speed* Soft * gamepad1.right_stick_x);
            DriveMotor4.setPower(Speed * Soft * gamepad1.right_stick_x);
        } else {    // 0 set power for the 4 train drive system
            Soft = .1 / Speed;
            DriveMotor1.setPower(0); //DriveMotor1.getPower();
            DriveMotor2.setPower(0);
            DriveMotor3.setPower(0);
            DriveMotor4.setPower(0);

        }
       if (gamepad2.x) {      // servo for block pick up
            // down
            ServoA.setPosition(.75);
            ServoB.setPosition(0);
        }
        if (gamepad2.y) {      // servo for block pick up
            // up
            ServoA.setPosition(.2);
            ServoB.setPosition(.55);
        }

        if (gamepad2.right_stick_y > 0.5) {    //marker servo up and down
            Lifter.setPower(.75);
        } else if (gamepad2.right_stick_y < -0.5) {
            Lifter.setPower(-.75);
        } else {
            Lifter.setPower(0);
        }
        if (gamepad2.dpad_up) {
           //up
            Servo1.setPosition(.4);
        }  else if (gamepad2.dpad_down) {
           //down
            Servo1.setPosition(.6);
        }  else {
            Servo1.setPosition(.5);
        }
        if (gamepad2.dpad_left) {
            //up
            Servo2.setPosition(.6);  // CR servo
        }  else if (gamepad2.dpad_right) {
           //down
            Servo2.setPosition(.4);  //CR servo
        }  else {
            Servo2.setPosition(.5);
        }

        if (gamepad2.a) {      // cap stone
            // close
            ServoClaw.setPosition(1); //yes
            sleep(50);
            ServoClaw.setPosition(.55);
        }
        if (gamepad2.b) {      // cap stone
            // open
            ServoClaw.setPosition(0);//claw CR
            sleep(50);
            ServoClaw.setPosition(.45);
        }
        if (gamepad2.left_bumper) {            // flip ting on robot CR servo
           // ServoFlip.setPosition(0);
            //sleep(50);
            ServoFlip.setPosition(.4);
        }  else if (gamepad2.left_trigger > 0){
            //ServoFlip.setPosition(1);
            //sleep(50);
            ServoFlip.setPosition(.6);
        }  else {
            ServoFlip.setPosition(.5);
        }

        if (gamepad2.left_stick_x > 0.5) {    //open and close pick up
            PickUp.setPower(-0.75);
            PickUp2.setPower(0.75);
        } else if (gamepad2.left_stick_x < -0.5) {
            PickUp.setPower(0.75);
            PickUp2.setPower(-0.75);
        } else {
            PickUp.setPower(0);
            PickUp2.setPower(0);
        }
        if (gamepad2.left_stick_y > 0.5) {    //up and down pick up
            Up.setPower(-0.75);
        } else if (gamepad2.left_stick_y < -0.5) {
            Up.setPower(0.75);
        } else {
            Up.setPower(0);
        }


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
