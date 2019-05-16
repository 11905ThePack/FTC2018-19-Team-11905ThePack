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

@TeleOp(name="FebTeleOp", group="Drive-Type OpModes")

public class FebTeleOp extends OpMode
{
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
    private ElapsedTime runtime = new ElapsedTime(); //We don't really need an elapsed time telemetry, but here it is.
    private DcMotor DriveMotorR = null; //Left Front Motor
    private DcMotor DriveMotorL = null; //Left Front Motor

    // Init some other variables

;

    private double motorSpeed = 1;
    private String consoleOut = "Nothing Yet";

    @Override
    public void init() {
        // Code to run ONCE when the driver hits INIT

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveMotorR = hardwareMap.get(DcMotor.class, "DriveMotorR");
      DriveMotorL = hardwareMap.get(DcMotor.class, "DriveMotorL");


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized again");
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




        //Controller One
        if (gamepad1.right_stick_y > 0.05) {       //forward
            motorSpeed = gamepad1.right_stick_y;
            DriveMotorR.setPower(motorSpeed);


        }else if(gamepad1.right_stick_y < -0.05) {  //reverse
            motorSpeed = gamepad1.right_stick_y;
            DriveMotorR.setPower(motorSpeed);

        }else {  //stopped
                motorSpeed = 0;
                DriveMotorR.setPower(0);

        }
        if (gamepad1.left_stick_y > 0.05) {       //forward
            motorSpeed = gamepad1.left_stick_y;
            DriveMotorL.setPower(-motorSpeed);

        }else if (gamepad1.left_stick_y < -0.05) {  //reverse
            motorSpeed = gamepad1.left_stick_y ;
            DriveMotorL.setPower(-motorSpeed);

  /*      } else if (gamepad1.right_stick_x < -0.1) {  //left
            motorSpeed = .55;
            DriveMotorR.setPower(motorSpeed);
            DriveMotorL.setPower(0);

        } else if (gamepad1.right_stick_x > 0.1) {   //right
            motorSpeed = .55;
            DriveMotorR.setPower(0);
            DriveMotorL.setPower(-motorSpeed);
*/
        } else {  //stopped
            motorSpeed = 0;
            DriveMotorL.setPower(0);
        }

        //Controller Two


        // Set Servo positions to variable "servoPosition"(s)



        // Send calculated power to wheels

        // Set Auxiliary Motor Powers


        //Read Positions Of Motors
        double PosR = DriveMotorR.getCurrentPosition();
        double PosL = DriveMotorL.getCurrentPosition();





        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running, Run Time: " + runtime.toString());

        telemetry.addData("GenericMotor posL" ,PosL);


        telemetry.addData( "Motor Speed","%.2f", motorSpeed);

        telemetry.addData( "Right_Stick" ,gamepad1.right_stick_y);
    }


    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP.

        telemetry.addData("Status:", "Stopped");

    }

}

