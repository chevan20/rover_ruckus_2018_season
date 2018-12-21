/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "craterTenser", group = "Concept")
@Disabled
public class craterTenser extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private DcMotor hookMotor = null;
    private DcMotor jBase = null;
    private DcMotor jMidd = null;

    private Servo D;
    double dPos = 0;

    private static final String VUFORIA_KEY = " AT0ySZn/////AAABGVDoF4gdNkZugx61WeftYqVhwz6Leeu2a1cWYQR+08xsATI6GQf3vvrvynP8JpemukCajxoFg32bkspzJx8g6uNBgHlQsFPxmFMJ8b4V/fDFTRSpy+vMOzIMoV2CHuitvtyrn/a6AsPUWczm5rsTqcCzAUEL6YD0xrzXkvaNJBzm3Jq5BkUW2ualta+LldpZ0ho/rdkDuyp6xOjsSvAbsjIDkQt807jlfYLBJAsaJNqRnQUU4mZMzl5aJsr+VnUbTfeev943zeK34ENEQzW1jCeCnLTsuWGmOd7QP+gF1YhxUr0A0s5Tr6+v8QntHPwYHp7QRpRErFqkst/OxbA4omIuLTsC41FFwYOr5YxDHhyl";
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;

    double distPerRot = (Math.PI * 3.8125);
    double stepsPerRot = 1120;
    double lengthOfField = 12 * 12;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private ModernRoboticsI2cGyro gyro;
    private boolean Left = false, Right = false, Center = false;

    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    //Set position to 0.025
    public void ServoUp(double POSITION) {
        dPos += POSITION;
        D.setPosition(dPos);

        if (dPos > 1) {
            dPos = 1;
        } else if (dPos < 0) {
            dPos = 0;
        }
    }

    //Set position to 0.025
    public void ServoDown(double POSITION) {
        dPos -= POSITION;
        D.setPosition(dPos);

        if (dPos > 1) {
            dPos = 1;
        } else if (dPos < 0) {
            dPos = 0;
        }
    }

    //----------------------------------------
    //Stop
    public void Stop() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    //Forward
    public void Forward(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);
    }

    public void Backward(double power) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackLeft.setPower(-power);
        BackRight.setPower(-power);
    }

    public void LTurn(double power) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(power);
        BackLeft.setPower(-power);
        BackRight.setPower(power);
    }

    public void RTurn(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }

    //----------------------------------------
    public void resetEncoder() {
        jMidd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        jMidd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//This method is for moving a distance in inches USING ENCODERS
//    public void armDistance(double power, double length)
//    {
//        //Total distance per 1 rotation is the product of pi * our wheel diameter
//        double distPerRot = Math.PI * 3.8125;
//        //There are this many steps per each rotation of NeverRest 20:1 dc motors
//        double stepsPerRot = 1120;
//
//        //This tells us exactly how many steps we need to travel
//        double totDistInSteps = ((length / distPerRot) * stepsPerRot);
//
//        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO FORWARD!
//        if (totDistInSteps > 0)
//        {
//            //Move forward until we over shoot
//            while (totDistInSteps >= BackLeft.getCurrentPosition())
//            {
//                Forward(power);
//                telemetry.addData("power", BackLeft.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//
//        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO BACKWARD!
//        else if (totDistInSteps < 0)
//        {
//            //Move backward until we over shoot
//            while (totDistInSteps <= BackLeft.getCurrentPosition())
//            {
//                Backward(power);
//                telemetry.addData("power", BackLeft.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//
//        //reset our encoder values because we may need to do more encoder driving
//        Stop();
//        resetEncoder();
//        sleep(1000);
//    }
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//This method is for moving a distance in inches USING ENCODERS
    public void moveDistance(double power, double length) {
        //Total distance per 1 rotation is the product of pi * our wheel diameter
        double distPerRot = Math.PI * 3.8125;
        //There are this many steps per each rotation of NeverRest 20:1 dc motors
        double stepsPerRot = 1120;

        //This tells us exactly how many steps we need to travel
        double totDistInSteps = ((length / distPerRot) * stepsPerRot);

        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO FORWARD!
        if (totDistInSteps > 0) {
            //Move forward until we over shoot
            while (totDistInSteps >= BackLeft.getCurrentPosition()) {
                Forward(power);
                telemetry.addData("power", BackLeft.getCurrentPosition());
                telemetry.update();
            }
        }

        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO BACKWARD!
        else if (totDistInSteps < 0) {
            //Move backward until we over shoot
            while (totDistInSteps <= BackLeft.getCurrentPosition()) {
                Backward(power);
                telemetry.addData("power", BackLeft.getCurrentPosition());
                telemetry.update();
            }
        }

        //reset our encoder values because we may need to do more encoder driving
        Stop();
        resetEncoder();
        sleep(1000);
    }

    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    public void runOpMode() {
        runtime.milliseconds();
        initVuforia();
//----------------------------------------
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
//----------------------------------------
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        runtime.reset();
        waitForStart();
//----------------------------------------
        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }

            //This is for our configuration in the robot
            FrontLeft = hardwareMap.get(DcMotor.class, "FL");
            FrontRight = hardwareMap.get(DcMotor.class, "FR");
            BackLeft = hardwareMap.get(DcMotor.class, "BL");
            BackRight = hardwareMap.get(DcMotor.class, "BR");
            hookMotor = hardwareMap.get(DcMotor.class, "Hook");
            jBase = hardwareMap.get(DcMotor.class, "one");
            jMidd = hardwareMap.get(DcMotor.class, "two");

            //Declaring the gyro and servo for end defector
            gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
            D = hardwareMap.get(Servo.class, "d");

            jBase.setDirection(DcMotorSimple.Direction.FORWARD);
            jMidd.setDirection(DcMotorSimple.Direction.FORWARD);

            //Set which direction is forward and backwards for our wheels
            FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
//----------------------------------------
            //Resetting all of the individual encoders to 0
            jMidd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            jBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Here, we tell the robot that we are using encoders and to pull data from them
            jMidd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            jBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//----------------------------------------
            //THis resets our gyro code and calibrates it to 0
            gyro.resetZAxisIntegrator();
            gyro.calibrate();

            //This updates our gyro heading post resetting it
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.update();

            //This if for COUNTING the number of encoder steps
            waitForStart();
            runtime.reset();
//--------------------------------------------------------------------------------------------------------------
            while (opModeIsActive()) {
                runtime.reset();
//----------------------------------------
                if (Left)
//----------------------------------------
                {
                    //Turn to face cube
                    while (!(gyro.getHeading() >= 30 && gyro.getHeading() <= 35)) {
                        LTurn(.1);
                    }

                    moveDistance(.5, -30);
                    sleep(1000);
                    Stop();

                    //THIS CODE IS FOR PLACING THE TEAM MARKER
//                    moveDistance(6);
//
//                    while(!(gyro.getHeading() >= 55 && gyro.getHeading() <= 65))
//                    {
//                        LTurn(.1);
//                    }
//
//                    //THIS CODE IS GOOD AND IS FOR TEAM MARKER
//                    sleep(1000);
//
//                    moveDistance(-20);
//
//                    while(!(gyro.getHeading() >= 28 && gyro.getHeading() <= 35))
//                    {
//                        LTurn(.1);
//                    }
//
//                    moveDistance(-60);
//
//                    gyro.resetZAxisIntegrator();
//                    gyro.calibrate();
//
//                    //THIS SLEEP IS FOR THE TEAM MARKER
//                    sleep(4000);
//
//                    moveDistance(140);
                    stop();
                }
//----------------------------------------
                else if (Right)
//----------------------------------------
                {
                    //Rotate towards the Gold Mineral
                    while (!(gyro.getHeading() >= 335 && gyro.getHeading() <= 340)) {
                        RTurn(.1);
                    }
                    //Move to knock of Cube
                    moveDistance(.5, -30);
                    sleep(2000);
                    Stop();

                    //THIS CODE IS FOR PLACING TEAM MARKER
//                    moveDistance(140);
                    stop();
                }
//----------------------------------------
                else if (Center)
//----------------------------------------
                {
                    //Move to knock of Cube
                    moveDistance(.5, -30);
                    sleep(1000);
                    Stop();

                    //THIS CODE IS FOR PLACING THE TEAM MARKER
//                    moveDistance(10);
//
//                    while(!(gyro.getHeading() >= 85 && gyro.getHeading() <= 95))
//                    {
//                        LTurn(.1);
//                    }
//
//                    moveDistance(-50);
//
//                    while(!(gyro.getHeading() >= 31 && gyro.getHeading() <= 36))
//                    {
//                        LTurn(.1);
//                    }
//
//                    moveDistance(-30);
//
//                    //THIS SLEEP IS FOR TEAM MARKER
//                    sleep(4000);
//
//                    moveDistance(140);
                    stop();
                }
//----------------------------------------
                else {
                    if (tfod != null) {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            int goldMineralX = -100;

                            sleep(2000);

                            if (updatedRecognitions.size() != 0) {
                                runtime.reset();
                                while (runtime.milliseconds() < 3000) {
                                    for (Recognition recognition : updatedRecognitions) {

                                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                            goldMineralX = (int) recognition.getTop();

                                            if (goldMineralX > -100 && goldMineralX < 600) {
                                                telemetry.addData("Pos:", "Right");
                                                telemetry.addData("MineralPos: ", goldMineralX);
                                                telemetry.update();
                                            } else if (goldMineralX > +600 && goldMineralX < 1100) {
                                                telemetry.addData("Pos:", "Center");
                                                telemetry.addData("MineralPos: ", goldMineralX);
                                                telemetry.update();
                                            } else {
                                                telemetry.addData("Pos:", "Left");
                                                telemetry.addData("MineralPos: ", goldMineralX);
                                                telemetry.update();
                                            }
                                        }
                                        telemetry.addData("MineralPos: ", goldMineralX);
                                        telemetry.update();
                                    }
                                }
                            } else {
                                runtime.reset();
                                if (runtime.milliseconds() < 3000) {
                                    telemetry.addData("Pos:", "Left");
                                    Left = true;
                                }
                            }

                            telemetry.addData("MineralPos: ", goldMineralX);

                            if (goldMineralX > -100 && goldMineralX < 600) {
                                telemetry.addData("Pos:", "Right");
                                Right = true;
                            } else if (goldMineralX > +600 && goldMineralX < 1200) {
                                telemetry.addData("Pos:", "Center");
                                Center = true;
                            } else {
                                telemetry.addData("Pos:", "Left");
                                Left = true;
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    //---------------------------------------
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    //---------------------------------------
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}