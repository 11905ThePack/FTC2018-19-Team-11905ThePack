package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.hardware.Sensor;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;




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


@Autonomous(name = "SkyAuto5Red", group = "First")

public class SkyAuto5Red extends LinearOpMode {

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

    Servo Servo1;
    Servo Servo2;

    ColorSensor color_sensor;


    //Colour Sensors

    //Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double DRIVE_GEAR_REDUCTION = 20.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_DEGREE =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) * 1.8 / 90;


    @Override
    public void runOpMode() {

        DriveMotor1 = hardwareMap.get(DcMotor.class, "DriveMotor1");
        DriveMotor2 = hardwareMap.get(DcMotor.class, "DriveMotor2");
        DriveMotor3 = hardwareMap.get(DcMotor.class, "DriveMotor3");
        DriveMotor4 = hardwareMap.get(DcMotor.class, "DriveMotor4");

        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");

        color_sensor = hardwareMap.colorSensor.get("color_sensor");



        //Wait for Start Button to Be Pressed
        waitForStart();
        total_eTime.reset();


        if (opModeIsActive()) {   //drive around;


            while (opModeIsActive() && total_eTime.time() < 60) {
                Drive(.2,10,5);
                Turn(.2,180,5);
                Drive(.2,10,5);

                if( Yellow()){
                    Servo1.setPosition(1);
                    Servo2.setPosition(.5);
                    if( Yellow()){

                        // drive to last block
                    }
                }

            };

            


                //Drive(-.1, -44, 5);
            //Translation(-.1, -36, 5);
            // Drive(-.1, -68, 5);
           // Drive(.1, 68, 5);
        }


        telemetry.addData("ConsoleOut", "Finished, Wait for end.");
        telemetry.update();


        //  telemetry.addData("ConsoleOut", "Finished, end.");
        //telemetry.update();
    }


    private void Drive(double speed,
                       double inches,
                       int drivetime) {

        int ticks;
        boolean Motor1 = true;
        boolean Motor2 = true;
        boolean Motor3 = true;
        boolean Motor4 = true;

        ElapsedTime DriveTimer = new ElapsedTime();
        //DriveLeftRear.getCurrentPosition(),
        // DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //DriveLeftRear.setTargetPosition(newLeftBackTarget);
        //DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveTimer.reset();

        DriveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        DriveMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ticks = (int) (inches * COUNTS_PER_INCH);

        DriveMotor1.setTargetPosition(-ticks);
        DriveMotor2.setTargetPosition(-ticks);
        DriveMotor3.setTargetPosition(ticks);
        DriveMotor4.setTargetPosition(ticks);

        runtime.reset();

        DriveMotor1.setPower(speed);
        DriveMotor2.setPower(speed);
        DriveMotor3.setPower(-speed);
        DriveMotor4.setPower(-speed);


        while (opModeIsActive()
                && (DriveTimer.seconds() < drivetime)
                && (Motor1 || Motor2 || Motor3 || Motor4)
                && (total_eTime.time() < 30)) {
            if (Math.abs(DriveMotor1.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor1.setPower(0);
                Motor1 = false;
                //telemetry.addData("Stop1", "at 1 %7d : ticks %7d ",
                //      DriveMotor1.getCurrentPosition(),
                //    ticks);
                //    telemetry.update();
            }
            if (Math.abs(DriveMotor2.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor2.setPower(0);
                Motor2 = false;
            }
            if (Math.abs(DriveMotor3.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor3.setPower(0);
                Motor3 = false;
            }
            if (Math.abs(DriveMotor4.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor4.setPower(0);
                Motor4 = false;
            }
            //telemetry.addData("Path1", "Running at 1 %7d : 2 %7d ",
            //      DriveMotor1.getCurrentPosition(),
            //    DriveMotor2.getCurrentPosition());
            //   telemetry.update();
        }


        // DriveMotor1.setPower(0);
        //DriveMotor2.setPower(0);
        // DriveMotor3.setPower(0);
        // DriveMotor4.setPower(0);

        if (opModeIsActive()) {
            telemetry.addData("ConsoleOut", "stopped.");
            telemetry.addData("Path1", "DriveNew at 1 %7d : 2 %7d ",
                    DriveMotor1.getCurrentPosition(),
                    DriveMotor2.getCurrentPosition());
            telemetry.addData("Path2", "DriveNew at 3 %7d : 4 %7d",
                    DriveMotor3.getCurrentPosition(),
                    DriveMotor4.getCurrentPosition());
            telemetry.addData("DriveMotor1.isBusy()", DriveMotor1.isBusy());
            telemetry.addData("DriveMotor2.isBusy()", DriveMotor2.isBusy());
            telemetry.addData("DriveMotor3.isBusy()", DriveMotor3.isBusy());
            telemetry.addData("DriveMotor4.isBusy()", DriveMotor4.isBusy());
        } else {
            telemetry.addData("ConsoleOut", "emergency stopped.");
        }
        telemetry.update();
        //boo2

        //Turn off RUN_TO_POSITION

        sleep(2500);   // optional pause after each move


    }


    private void Turn(double speed,
                      double degrees,
                      int drivetime) {

        int ticks;
        boolean Motor1 = true;
        boolean Motor2 = true;
        boolean Motor3 = true;
        boolean Motor4 = true;

        ElapsedTime DriveTimer = new ElapsedTime();
        //DriveLeftRear.getCurrentPosition(),
        // DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //DriveLeftRear.setTargetPosition(newLeftBackTarget);
        //DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveTimer.reset();

        DriveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        DriveMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ticks = (int) (degrees * COUNTS_PER_INCH * 17.6 / 90);

        DriveMotor1.setTargetPosition(-ticks);
        DriveMotor2.setTargetPosition(-ticks);
        DriveMotor3.setTargetPosition(-ticks);
        DriveMotor4.setTargetPosition(-ticks);

        runtime.reset();

        DriveMotor1.setPower(-speed);
        DriveMotor2.setPower(-speed);
        DriveMotor3.setPower(-speed);
        DriveMotor4.setPower(-speed);


        while (opModeIsActive()
                && (DriveTimer.seconds() < drivetime)
                && (Motor1 || Motor2 || Motor3 || Motor4)
                && (total_eTime.time() < 30)) {
            if (Math.abs(DriveMotor1.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor1.setPower(0);
                Motor1 = false;
                //telemetry.addData("Stop1", "at 1 %7d : ticks %7d ",
                //      DriveMotor1.getCurrentPosition(),
                //    ticks);
                //    telemetry.update();
            }
            if (Math.abs(DriveMotor2.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor2.setPower(0);
                Motor2 = false;
            }
            if (Math.abs(DriveMotor3.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor3.setPower(0);
                Motor3 = false;
            }
            if (Math.abs(DriveMotor4.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor4.setPower(0);
                Motor4 = false;
            }
            //telemetry.addData("Path1", "Running at 1 %7d : 2 %7d ",
            //      DriveMotor1.getCurrentPosition(),
            //    DriveMotor2.getCurrentPosition());
            //   telemetry.update();
        }


        // DriveMotor1.setPower(0);
        //DriveMotor2.setPower(0);
        // DriveMotor3.setPower(0);
        // DriveMotor4.setPower(0);

        if (opModeIsActive()) {
            telemetry.addData("ConsoleOut", "stopped.");
            telemetry.addData("Path1", "DriveNew at 1 %7d : 2 %7d ",
                    DriveMotor1.getCurrentPosition(),
                    DriveMotor2.getCurrentPosition());
            telemetry.addData("Path2", "DriveNew at 3 %7d : 4 %7d",
                    DriveMotor3.getCurrentPosition(),
                    DriveMotor4.getCurrentPosition());
            telemetry.addData("DriveMotor1.isBusy()", DriveMotor1.isBusy());
            telemetry.addData("DriveMotor2.isBusy()", DriveMotor2.isBusy());
            telemetry.addData("DriveMotor3.isBusy()", DriveMotor3.isBusy());
            telemetry.addData("DriveMotor4.isBusy()", DriveMotor4.isBusy());
        } else {
            telemetry.addData("ConsoleOut", "emergency stopped.");
        }
        telemetry.update();

        //Turn off RUN_TO_POSITION

        sleep(2500);   // optional pause after each move


    }


    public void Translation(double speed,
                            double inches,
                            int drivetime) {

        int ticks;
        boolean Motor1 = true;
        boolean Motor2 = true;
        boolean Motor3 = true;
        boolean Motor4 = true;


        ElapsedTime DriveTimer = new ElapsedTime();
        //DriveLeftRear.getCurrentPosition(),
        // DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //DriveLeftRear.setTargetPosition(newLeftBackTarget);
        //DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveTimer.reset();

        DriveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        DriveMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ticks = (int) (inches * COUNTS_PER_INCH);
        DriveMotor1.setTargetPosition(-ticks);
        DriveMotor2.setTargetPosition(ticks);
        DriveMotor3.setTargetPosition(ticks);
        DriveMotor4.setTargetPosition(-ticks);

        runtime.reset();

        DriveMotor1.setPower(-speed);
        DriveMotor2.setPower(-speed);
        DriveMotor3.setPower(speed);
        DriveMotor4.setPower(speed);


        while (opModeIsActive()
                && (DriveTimer.seconds() < drivetime)
                && (Motor1 || Motor2 || Motor3 || Motor4)
                && (total_eTime.time() < 30)) {
            if (Math.abs(DriveMotor1.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor1.setPower(0);
                Motor1 = false;
                //telemetry.addData("Stop1", "at 1 %7d : ticks %7d ",
                //      DriveMotor1.getCurrentPosition(),
                //    ticks);
                //    telemetry.update();
            }
            if (Math.abs(DriveMotor2.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor2.setPower(0);
                Motor2 = false;
            }
            if (Math.abs(DriveMotor3.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor3.setPower(0);
                Motor3 = false;
            }
            if (Math.abs(DriveMotor4.getCurrentPosition()) >= Math.abs(ticks)) {
                DriveMotor4.setPower(0);
                Motor4 = false;
            }
            //telemetry.addData("Path1", "Running at 1 %7d : 2 %7d ",
            //      DriveMotor1.getCurrentPosition(),
            //    DriveMotor2.getCurrentPosition());
            //   telemetry.update();
        }


        //  DriveMotor1.setPower(0);
        // DriveMotor2.setPower(0);
        //DriveMotor3.setPower(0);
        //  DriveMotor4.setPower(0);

        if (opModeIsActive()) {
            telemetry.addData("ConsoleOut", "stopped.");
            telemetry.addData("Path1", "Running at 1 %7d : 2 %7d ",
                    DriveMotor1.getCurrentPosition(),
                    DriveMotor2.getCurrentPosition());
            telemetry.addData("Path2", "Running at 3 %7d : 4 %7d",
                    DriveMotor3.getCurrentPosition(),
                    DriveMotor4.getCurrentPosition());
            telemetry.addData("DriveMotor1.isBusy()", DriveMotor1.isBusy());
            telemetry.addData("DriveMotor2.isBusy()", DriveMotor2.isBusy());
            telemetry.addData("DriveMotor3.isBusy()", DriveMotor3.isBusy());
            telemetry.addData("DriveMotor4.isBusy()", DriveMotor4.isBusy());
        } else {
            telemetry.addData("ConsoleOut", "emergency stopped.");
        }
        telemetry.update();

        //Turn off RUN_TO_POSITION

        //  sleep(2500);   // optional pause after each move


    }

    public boolean Yellow() {
        int red = color_sensor.red();
        int green = color_sensor.green();
        int blue = color_sensor.blue();
        boolean temp = false;
        if ( red > 4 && green > 3 && (blue < (red/3))) {
            telemetry.addData("yellow", " R %7d : G %7d : B %7d ",
                    red,
                    green,
                    blue);
            temp = true;
            }
        else if ( red > 1 && green > 1 && blue > 1)
            telemetry.addData("not yellow", " R %7d : G %7d : B %7d ",
                    red,
                    green,
                    blue);

        telemetry.update();

        return (temp);
    }



}
