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
    private DcMotor DriveMotor1 = null; // front right
    private DcMotor DriveMotor2 = null; // front left
    private DcMotor DriveMotor3 = null; // back right
    private DcMotor DriveMotor4 = null;
   private DcMotor Up= null;// back left
    private Servo ServoA = null;
    private Servo ServoB = null;
   private DcMotor PickUpMotor = null;  // pick up system

    private double Speed = .25;
    private double Soft = 0;


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

        ServoA = hardwareMap.get(Servo.class, "ServoA");
       ServoB = hardwareMap.get(Servo.class, "ServoB");
      PickUpMotor = hardwareMap.get(DcMotor.class, "PickUpMotor");
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
            DriveMotor1.setPower(Speed * Soft * gamepad1.left_stick_y);
            DriveMotor2.setPower(-Speed * Soft * gamepad1.left_stick_y);
            DriveMotor3.setPower(Speed * Soft * gamepad1.left_stick_y);
            DriveMotor4.setPower(-Speed * Soft * gamepad1.left_stick_y);
        } else if ((gamepad1.left_stick_x > 0.25) || (gamepad1.left_stick_x < -0.25)) {      // Translation
            if (Soft < 1) Soft = Soft + 0.02;
            DriveMotor1.setPower(Speed * Soft * gamepad1.left_stick_x);
            DriveMotor2.setPower(Speed * Soft * gamepad1.left_stick_x);
            DriveMotor3.setPower(-Speed * Soft * gamepad1.left_stick_x);
            DriveMotor4.setPower(-Speed * Soft * gamepad1.left_stick_x);
        } else if ((gamepad1.right_stick_x > 0.25) || (gamepad1.right_stick_x < -0.25)) {      //turning
            if (Soft < 1) Soft = Soft + 0.02;
            DriveMotor1.setPower(-Speed * Soft * gamepad1.right_stick_x);
            DriveMotor2.setPower(-Speed * Soft * gamepad1.right_stick_x);
            DriveMotor3.setPower(-Speed * Soft * gamepad1.right_stick_x);
            DriveMotor4.setPower(-Speed * Soft * gamepad1.right_stick_x);
        } else {    // 0 set power for the 4 train drive system
            Soft = .1 / Speed;
            DriveMotor1.setPower(0); //DriveMotor1.getPower();
            DriveMotor2.setPower(0);
            DriveMotor3.setPower(0);
            DriveMotor4.setPower(0);

        }
        if (gamepad2.y) {      // servo for the pooper scooper
            // open
            ServoA.setPosition(0);
            ServoB.setPosition(1);
        }
        /*
        if (gamepad2.right_stick_y > 0.5) {    //marker servo up and down
            Up2.setPower(.75);
        } else if (gamepad2.right_stick_y < -0.5) {
            Up2.setPower(-.75);
        } else {
            Speed = 0;
            Up2.setPower(0);
        }*/



        if (gamepad2.left_stick_x > 0.5) {    //open and close pick up
            PickUpMotor.setPower(.75);
        } else if (gamepad2.left_stick_x < -0.5) {
            PickUpMotor.setPower(-.75);
        } else {
            Speed = 0;
            PickUpMotor.setPower(0);
        }
        if (gamepad2.left_stick_y > 0.5) {    //up and down pick up
            Up.setPower(.75);
        } else if (gamepad2.left_stick_y < -0.5) {
            Up.setPower(-.75);
        } else {
            Speed = 0;
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
