package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp", group = "Main")
public class LinearOp extends LinearOpMode {

    DcMotor FrontLeftMotor;
    DcMotor FrontRightMotor;
    DcMotor BackRightMotor;
    DcMotor BackLeftMotor;
    DcMotor HookMotor;
    Servo TeamMarkerServo;
    ModernRoboticsI2cGyro GyroSensor;

    Servo drawBridge;
    CRServo sweeper;
    DcMotor extend;
    DcMotor lift;
    DcMotor angle;


    //----------------------------------------------------------------------------------------------
    double teamMarkerServoPosition = 0;

    double FrontLeftMotorPower = 0;
    double FrontRightMotorPower = 0;
    double BackRightMotorPower = 0;
    double BackLeftMotorPower = 0;


    //----------------------------------------------------------------------------------------------
    private void FowardPosBackNeg(double pow) {
        FrontLeftMotor.setPower(pow);
        FrontRightMotor.setPower(pow);
        BackLeftMotor.setPower(pow);
        BackRightMotor.setPower(pow);
    }

    //----------------------------------------------------------------------------------------------
    private void LeftTurnPosRightTurnNeg(double pow) {
        FrontLeftMotor.setPower(pow);
        FrontRightMotor.setPower(-pow);
        BackLeftMotor.setPower(pow);
        BackRightMotor.setPower(-pow);
    }

    //----------------------------------------------------------------------------------------------
    private void StopWheels() {
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);
    }

    //----------------------------------------------------------------------------------------------
    private void servoUp(double POSITION) {
        teamMarkerServoPosition += POSITION;
        TeamMarkerServo.setPosition(teamMarkerServoPosition);

        if (teamMarkerServoPosition > 1) {
            teamMarkerServoPosition = 1;
        } else if (teamMarkerServoPosition < 0) {
            teamMarkerServoPosition = 0;
        }

    }

    //----------------------------------------------------------------------------------------------
    private void servoDown(double POSITION) {
        teamMarkerServoPosition -= POSITION;
        TeamMarkerServo.setPosition(teamMarkerServoPosition);

        if (teamMarkerServoPosition > 1) {
            teamMarkerServoPosition = 1;
        } else if (teamMarkerServoPosition < 0) {
            teamMarkerServoPosition = 0;
        }
    }

    //----------------------------------------------------------------------------------------------
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //------------------------------------------------------------------------------------------
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BR");
        HookMotor = hardwareMap.get(DcMotor.class, "Hook");
        TeamMarkerServo = hardwareMap.get(Servo.class, "d");
        GyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        drawBridge = hardwareMap.get(Servo.class, "drawBridge");
        sweeper = hardwareMap.get(CRServo.class, "sweeper");
        extend = hardwareMap.get(DcMotor.class, "extend");
        lift = hardwareMap.get(DcMotor.class, "lift");
        angle = hardwareMap.get(DcMotor.class, "angle");
        //------------------------------------------------------------------------------------------
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        //------------------------------------------------------------------------------------------
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //------------------------------------------------------------------------------------------
        HookMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        HookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //------------------------------------------------------------------------------------------
        GyroSensor.resetZAxisIntegrator();
        GyroSensor.calibrate();

        //------------------------------------------------------------------------------------------
        waitForStart();

        while (opModeIsActive()) {
            //things needed to code
            //
            //Drive Train
            //HookDown
            //HookUp
            //ExtendUp/Down
            //LiftUp/Down
            //AngleUp/Down
            //DrawBridgeUp/Down
            //SweeperForward/Backward

            //------------------------------------------------------------------------Controller-One

            double xPos = gamepad1.right_stick_x;
            double yPos = gamepad1.left_stick_x;

            if (gamepad1.right_stick_y != 0) {
                FowardPosBackNeg(-gamepad1.right_stick_y);
            } else if (gamepad1.left_stick_x != 0) {
                LeftTurnPosRightTurnNeg(gamepad1.left_stick_x);
            } else {
                StopWheels();
            }

            if (gamepad1.right_trigger != 0) {
                HookMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                HookMotor.setPower(-gamepad1.left_trigger);
            } else {
                HookMotor.setPower(0);
            }

//
//
//            //------------------------------------------------------------------------Controller-Two
//            //ExtendUp/Down
//            //LiftUp/Down
//            //AngleUp/Down
//            //DrawBridgeUp/Down
//            //SweeperForward/Backward
//
//
//            //drawBridge.setPosition(0);
//
//            if(gamepad2.right_stick_y!=0){
//                lift.setPower(gamepad2.right_stick_y);
//            }else{
//                lift.setPower(0);
//            }
//
//            if(gamepad2.left_stick_y != 0){
//                extend.setPower(gamepad2.left_stick_y);
//            }else{
//                extend.setPower(0);
//            }
//
//            if(gamepad2.y){
//                drawBridge.setPosition(180);
//            }
//            if(gamepad2.x){
//                drawBridge.setPosition(0);
//            }
//            if(gamepad2.a){
//                sweeper.setPower(1);
//            }else if(gamepad2.b){
//                sweeper.setPower(-1);
//            }else{
//                sweeper.setPower(0);
//
//            }
//            if(gamepad2.right_trigger != 0){
//                angle.setPower(gamepad2.right_trigger);
//            }else if(gamepad2.left_trigger != 0){
//                angle.setPower(gamepad2.left_trigger);
//            }else{
//                angle.setPower(0);
//            }
//

        }
    }
}
