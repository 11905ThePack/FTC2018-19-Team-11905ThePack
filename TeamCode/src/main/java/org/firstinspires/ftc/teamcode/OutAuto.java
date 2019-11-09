package org.firstinspires.ftc.teamcode;


import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
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


@Autonomous(name = "OutAuto", group = "First")

public class OutAuto extends LinearOpMode {

    //Elapsed Time
    ElapsedTime eTime = new ElapsedTime();
    ElapsedTime eTime2 = new ElapsedTime();
    ElapsedTime total_eTime = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor DriveMotor1;
    DcMotor DriveMotor2;
    DcMotor DriveMotor3;
    DcMotor DriveMotor4;


    // ColorSensor color;


    //Motor DriveMotor3;// DcMotor DriveMotor4;


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
        DriveMotor1 = hardwareMap.get(DcMotor.class, "DriveMotor1");
        DriveMotor2 = hardwareMap.get(DcMotor.class, "DriveMotor2");
        DriveMotor3 = hardwareMap.get(DcMotor.class, "DriveMotor3");
        DriveMotor4 = hardwareMap.get(DcMotor.class, "DriveMotor4");


        // color = hardwareMap.colorSensor.get("color_sensor");


        telemetry.update();


        waitForStart();
        total_eTime.reset();

        //color.green();
        //color.blue();
        //color.red();  // Combined color value

        //drive around;

            Drive(.2, 3);
            Turn(.5, 3);
            Drive(-1, 5);
            Turn(-1, 3);


    }


    //  telemetry.addData("ConsoleOut", "Finished, end.");
    //telemetry.update();


    public void Drive(double speed,
                      int drivetime) {
        ElapsedTime DriveTimer = new ElapsedTime();
        if( opModeIsActive()) {
            DriveMotor1.setPower(speed);
            DriveMotor2.setPower(-speed);
            DriveMotor3.setPower(speed);
            DriveMotor4.setPower(-speed);
        }

        while (opModeIsActive() && (DriveTimer.time() < drivetime)) {


            telemetry.addData("Status", "Running");
            telemetry.update();
            //  color.enableLed(true);  // Turn the LED on
            // color.enableLed(false);


            if (opModeIsActive()) {
                telemetry.addData("ConsoleOut", "stopped.");
                telemetry.addData("DriveMotor1.isBusy()", DriveMotor1.isBusy());
                telemetry.addData("DriveMotor2.isBusy()", DriveMotor2.isBusy());
                telemetry.addData("DriveMotor3.isBusy()", DriveMotor3.isBusy());
                telemetry.addData("DriveMotor4.isBusy()", DriveMotor4.isBusy());

            } else {
                telemetry.addData("ConsoleOut", "emergency stopped.");
            }
            telemetry.update();

            //Turn off RUN_TO_POSITION😄

            //  sleep(2500);   // optional pause after each move


        }
        DriveMotor1.setPower(0);
        DriveMotor2.setPower(0);
        DriveMotor3.setPower(0);
        DriveMotor4.setPower(0);

    }
    public void Translate(double speed,
                      int drivetime) {
        ElapsedTime DriveTimer = new ElapsedTime();
        if( opModeIsActive()) {
            DriveMotor1.setPower(-speed);
            DriveMotor2.setPower(-speed);
            DriveMotor3.setPower(speed);
            DriveMotor4.setPower(speed);
        }

        while (opModeIsActive() && (DriveTimer.time() < drivetime)) {




        }
        DriveMotor1.setPower(0);
        DriveMotor2.setPower(0);
        DriveMotor3.setPower(0);
        DriveMotor4.setPower(0);

    }
    public void Turn(double speed,   //turning
                          int drivetime) {
        ElapsedTime DriveTimer = new ElapsedTime();
        if( opModeIsActive()) {
            DriveMotor1.setPower(-speed);
            DriveMotor2.setPower(-speed);
            DriveMotor3.setPower(-speed);
            DriveMotor4.setPower(-speed);
        }

        while (opModeIsActive() && (DriveTimer.time() < drivetime)) {




        }
        DriveMotor1.setPower(0);
        DriveMotor2.setPower(0);
        DriveMotor3.setPower(0);
        DriveMotor4.setPower(0);

    }
}

