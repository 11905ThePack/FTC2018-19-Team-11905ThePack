package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



@Autonomous(name = "MarAuto", group = "First")

public class MarAuto extends LinearOpMode {

    //Elapsed Time
    ElapsedTime eTime = new ElapsedTime();
    ElapsedTime eTime2 = new ElapsedTime();
    ElapsedTime total_eTime = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor DriveMotorL;
    DcMotor DriveMotorR;

    //Colour Sensors

    //Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double DRIVE_GEAR_REDUCTION = 20.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.5;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_DEGREE =
           (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) * 1.8/90;


    @Override
    public void runOpMode() {

        //Motors
        DriveMotorL = hardwareMap.get(DcMotor.class, "DriveMotorL");
        DriveMotorR = hardwareMap.get(DcMotor.class, "DriveMotorR");

        telemetry.update();

        //Wait for Start Button to Be Pressed
        waitForStart();
        total_eTime.reset();



        ;
        while (opModeIsActive() ) {   //drive around;
                Drive(.05, 5);
        }


                telemetry.addData("ConsoleOut", "Finished, Wait for end.");
        telemetry.update();

        while (total_eTime.time() < 30);


          //  telemetry.addData("ConsoleOut", "Finished, end.");
        //telemetry.update();

        }



    public void Drive(double speed,
                             int drivetime) {
        ElapsedTime DriveTimer = new ElapsedTime();
        //DriveLeftRear.getCurrentPosition(),
        // DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //DriveLeftRear.setTargetPosition(newLeftBackTarget);
        //DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveTimer.reset();

        DriveMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      //  DriveMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  DriveMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       // DriveMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorL.setPower(-speed);
        DriveMotorR.setPower(speed);

        DriveMotorR.setTargetPosition(drivetime*1000);
        DriveMotorL.setTargetPosition(drivetime*-1000);


        DriveMotorL.getTargetPosition();
        DriveMotorL.getTargetPosition();
         //speed = - speed;

        runtime.reset();

        DriveMotorR.setPower(1.0);
        DriveMotorL.setPower(1.05);

        while (opModeIsActive()
                && (DriveTimer.seconds() < drivetime)
                && (DriveMotorL.isBusy() && DriveMotorR.isBusy())
                && (total_eTime.time() < 30)) {
            telemetry.addData("Path1", "Running at %7d :%7d",
                    DriveMotorL.getCurrentPosition(),
                    DriveMotorR.getCurrentPosition());
            telemetry.addData("DriveMotorL.isBusy()",DriveMotorL.isBusy());
            telemetry.addData("DriveMotorR.isBusy()",DriveMotorR.isBusy());

            telemetry.update();
        }

        /*Ensures OpMode is running
       while (opModeIsActive()&&(DriveTimer.time() < drivetime)) {
            telemetry.addData("Path2", "Running at %7d :%7d",
                    DriveMotorL.getCurrentPosition(),
                    DriveMotorR.getCurrentPosition());

            telemetry.update();
        }
           */ // Stop all motion;

            DriveMotorL.setPower(0);
            DriveMotorR.setPower(0);

            if (opModeIsActive()) {
               telemetry.addData("ConsoleOut", "stopped.");
                telemetry.addData("DriveMotorL.isBusy()",DriveMotorL.isBusy());
                telemetry.addData("DriveMotorR.isBusy()",DriveMotorR.isBusy());
            } else {
                telemetry.addData("ConsoleOut", "emergency stopped.");
            }
        telemetry.update();
        sleep(2000);
            //Turn off RUN_TO_POSITION

          //  sleep(2500);   // optional pause after each move



    }



    public void DriveRotate(double speed,
                                   double degrees,
                                   double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        if (opModeIsActive() && (total_eTime.time() < 29)) {



        }
    }




}