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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "MainAutonomous", group = "First")

public class MainAutonomous extends LinearOpMode {

    //Team Checker
    TouchSensor TeamBlueSwitch;
    TouchSensor Position1Switch;

    //Elapsed Time
    ElapsedTime eTime = new ElapsedTime();
    ElapsedTime eTime2 = new ElapsedTime();
    ElapsedTime total_eTime = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor DriveLeftFront;
    DcMotor DriveLeftRear;
    DcMotor DriveRightFront;
    DcMotor DriveRightRear;

    DcMotor MotorRobotLifter;

    //Servos
    Servo ServoRobotLifter;

    //Colour Sensors

    //Gyros

    //Servo Stuff
    private static double RobotLifterCRServoStop = .5;
    private double servoRobotLifterPosition = RobotLifterCRServoStop;

    private final static double servoMinRange  = 1;
    private final static double servoMaxRange  = 180;

    //Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double DRIVE_GEAR_REDUCTION = 20.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_DEGREE =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) * 1.8/90;


    @Override
    public void runOpMode() {

        //Blue Switch
        TeamBlueSwitch = hardwareMap.get(TouchSensor.class, "TeamBlueSwitch");
        boolean TeamBlueSwitchRead = TeamBlueSwitch.isPressed();

        //Red Switch
        Position1Switch = hardwareMap.get(TouchSensor.class, "Position1Switch");
        boolean Position1SwitchRead = Position1Switch.isPressed();

        //Motors
        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
        DriveLeftFront.setDirection(DcMotor.Direction.REVERSE);
        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
        DriveLeftRear.setDirection(DcMotor.Direction.REVERSE);
        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");
        DriveRightRear = hardwareMap.get(DcMotor.class, "DriveRightRear");

         MotorRobotLifter = hardwareMap.get(DcMotor.class,"RobotLifter");
        ServoRobotLifter = hardwareMap.get(Servo.class,"ServoRobotLifter");

        telemetry.update();

        //Wait for Start Button to Be Pressed
        waitForStart();
        total_eTime.reset();

        TeamBlueSwitchRead = TeamBlueSwitch.isPressed();
        Position1SwitchRead = Position1Switch.isPressed();

        eTime2.reset();
        while (eTime2.time() < 1) {
        }

        while (opModeIsActive()) {

            RobotDescend(1,4,10);
            sleep(100);
            RobotDescend(-1,1,10);
            encoderDriveStraight(0.2,1,10);
            RobotDescend(1,-3,10);



            encoderDriveStraight(1,10,5);
            encoderDriveRotate(1,90,5);


            if (TeamBlueSwitchRead)  {

               if (Position1SwitchRead)  {  //Blue Team Position One
                    telemetry.addData("ConsoleOut", "Blue Team Position One");
                    telemetry.update();


               } else  {    //Blue Team Position Two
                    telemetry.addData("ConsoleOut", "Blue Team Position Two");
                    telemetry.update();


               }
            }  else  {  //Red Team

                if (Position1SwitchRead)  { //Red Team Position One
                    telemetry.addData("ConsoleOut", "Red Team Position One");
                    telemetry.update();


                } else  {   //Red Team Position Two
                    telemetry.addData("ConsoleOut", "Red Team Position Two");
                    telemetry.update();

                }
            }



            while (total_eTime.time() < 30) {
                telemetry.addData("ConsoleOut", "Finished, Wait for end.");

            }
        }
    }


    public void encoderDrive(double speed,
                             double leftBackInches, double rightBackInches, double leftFrontInches, double rightFrontInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        //Ensures OpMode is running
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = DriveLeftRear.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = DriveRightRear.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
            newLeftFrontTarget = DriveLeftFront.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = DriveRightFront.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);


            DriveLeftRear.setTargetPosition(newLeftBackTarget);
            DriveRightRear.setTargetPosition(newRightBackTarget);
            DriveLeftFront.setTargetPosition(newLeftFrontTarget);
            DriveRightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("encoderDriveOut", "encoderDrive starting");
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();

            DriveLeftRear.setPower(speed * 1.05);
            DriveRightRear.setPower(speed);
            DriveLeftFront.setPower(speed * 1.05);
            DriveRightFront.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (DriveLeftRear.isBusy() || DriveRightRear.isBusy() || DriveLeftFront.isBusy() || DriveRightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        DriveLeftRear.getCurrentPosition(),
                        DriveRightRear.getCurrentPosition(),
                        DriveLeftFront.getCurrentPosition(),
                        DriveRightFront.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            runtime.reset();
            DriveLeftRear.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftFront.setPower(0);
            DriveRightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(2500);   // optional pause after each move

        }

    }

    public void encoderDriveStraight(double speed,
                                     double inches,
                                     double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        boolean rightAhead = false;
        boolean leftAhead = false;

        //Ensures OpMode is running & not out of time
        if (opModeIsActive() && (total_eTime.time() < 29)) {

            DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newLeftBackTarget = (int) (inches * COUNTS_PER_INCH);
            newRightBackTarget = (int) (inches * COUNTS_PER_INCH);
            newLeftFrontTarget = (int) (inches * COUNTS_PER_INCH);
            newRightFrontTarget = (int) (inches * COUNTS_PER_INCH);

            DriveLeftRear.setTargetPosition(-newLeftBackTarget);
            DriveRightRear.setTargetPosition(-newRightBackTarget);
            DriveLeftFront.setTargetPosition(-newLeftFrontTarget);
            DriveRightFront.setTargetPosition(-newRightFrontTarget);
            speed = - speed;

            // Turn On RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("encoderDriveOut", "encoderDriveStraight starting");
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                    DriveLeftRear.getCurrentPosition(),
                    DriveRightRear.getCurrentPosition(),
                    DriveLeftFront.getCurrentPosition(),
                    DriveRightFront.getCurrentPosition());
            telemetry.update();
//            sleep(2000);   // optional pause after each move

            // reset the timeout time and start motion.
            runtime.reset();
            DriveLeftRear.setPower(speed * 1.583);
            DriveRightRear.setPower(speed);
            DriveLeftFront.setPower(speed * 1.583);
            DriveRightFront.setPower(speed);


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (DriveLeftRear.isBusy() || DriveRightRear.isBusy())
                    && (total_eTime.time() < 29)) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        DriveLeftRear.getCurrentPosition(),
                        DriveRightRear.getCurrentPosition(),
                        DriveLeftFront.getCurrentPosition(),
                        DriveRightFront.getCurrentPosition());
                telemetry.update();

            }

            //Stop All Motion;
            runtime.reset();
            DriveLeftRear.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftFront.setPower(0);
            DriveRightFront.setPower(0);

            //Turn off RUN_TO_POSITION
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    }


    public void encoderDriveRotate(double speed,
                                   double degrees,
                                   double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        if (opModeIsActive() && (total_eTime.time() < 29)) {

            DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newLeftBackTarget =  - (int) (degrees * COUNTS_PER_DEGREE);
            newRightBackTarget =  + (int) (degrees * COUNTS_PER_DEGREE);
            newLeftFrontTarget = - (int) (degrees * COUNTS_PER_DEGREE);
            newRightFrontTarget = + (int) (degrees * COUNTS_PER_DEGREE);

            DriveLeftRear.setTargetPosition(newLeftBackTarget);
            DriveRightRear.setTargetPosition(newRightBackTarget);
            DriveLeftFront.setTargetPosition(newLeftFrontTarget);
            DriveRightFront.setTargetPosition(newRightFrontTarget);

            DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("encoderDriveOut", "encoderDriveRotate starting");
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                    DriveLeftRear.getCurrentPosition(),
                    DriveRightRear.getCurrentPosition(),
                    DriveLeftFront.getCurrentPosition(),
                    DriveRightFront.getCurrentPosition());
            telemetry.update();

            runtime.reset();
            if (speed > 0.0) {
                DriveLeftRear.setPower(-(speed * 1.00));
                DriveRightRear.setPower(speed * 1.583);
                DriveLeftFront.setPower(-(speed * 1.00));
                DriveRightFront.setPower(speed * 1.583);
            } else {
                DriveLeftRear.setPower(speed);
                DriveRightRear.setPower(-(speed * 1.00));
                DriveLeftFront.setPower(speed);
                DriveRightFront.setPower(-(speed * 1.00));
            }

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (DriveLeftRear.isBusy() || DriveRightRear.isBusy())
                    && (total_eTime.time() < 29)) {

                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        DriveLeftRear.getCurrentPosition(),
                        DriveRightRear.getCurrentPosition(),
                        DriveLeftFront.getCurrentPosition(),
                        DriveRightFront.getCurrentPosition());
                telemetry.update();

            }

            runtime.reset();
            DriveLeftRear.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftFront.setPower(0);
            DriveRightFront.setPower(0);

            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         }
    }

    public void RobotDescend(double speed,
                             double inches,
                             double timeoutS){

        int newDescendTarget;

        if (opModeIsActive() && (total_eTime.time() < 29)) {

            MotorRobotLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newDescendTarget = (int) (inches * COUNTS_PER_INCH);

            MotorRobotLifter.setTargetPosition(newDescendTarget);

            MotorRobotLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            MotorRobotLifter.setPower(speed);



            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (MotorRobotLifter.isBusy())
                    && (total_eTime.time() < 29)) {
            }

            runtime.reset();

            MotorRobotLifter.setPower(0);

            MotorRobotLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}