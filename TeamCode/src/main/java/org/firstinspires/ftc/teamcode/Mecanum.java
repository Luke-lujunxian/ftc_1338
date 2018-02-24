package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;

/**
 * Created by yuying75618 on 18/2/21.
 */

public class Mecanum{
    protected double L1 = 0;//左前
    protected double L2 = 0;//左后
    protected double R1 = 0;//右前
    protected double R2 = 0;//右后

    protected DcMotor MotorL1,MotorL2,MotorR1,MotorR2;
    final static double RADIUS = 4.9;
    final static double selfRADIUS = 24.3;
    final static double r = 1500;

    private void setMacanum(){
        MotorL1.setPower(L1);
        MotorL2.setPower(L2);
        MotorR1.setPower(R1);
        MotorR2.setPower(R2);
        return;
    } //设置电机速度

    int i = 0;
    protected double lastL1 = 0;//左前
    protected double lastL2 = 0;//左后
    protected double lastR1 = 0;//右前
    protected double lastR2 = 0;//右后
    protected double changeL1 = 0;//左前
    protected double changeL2 = 0;//左后
    protected double changeR1 = 0;//右前
    protected double changeR2 = 0;//右后
    public void getPositionChange(){
        if(i >= 10){
            changeL1 = MotorL1.getCurrentPosition() - lastL1;
            changeL2 = MotorL2.getCurrentPosition() - lastL2;
            changeR1 = MotorR1.getCurrentPosition() - lastR1;
            changeR2 = MotorR2.getCurrentPosition() - lastR2;
            lastL1 = MotorL1.getCurrentPosition();
            lastL2 = MotorL2.getCurrentPosition();
            lastR1 = MotorR1.getCurrentPosition();
            lastR2 = MotorR2.getCurrentPosition();
            i = 0;
        }
        else
            i++;
    }//获取电机转速
    public double varSpeed(){
        double M1,M2,M3,M4;
        M4 = Math.abs(changeR2);
        M2 = Math.abs(changeL2);
        M3 = Math.abs(changeR1);
        M1 = Math.abs(changeL1);
        double aM = (M1 + M2 + M3 + M4) / 4;
        return (Math.pow((M1 - aM),2) + Math.pow((M2 - aM),2) + Math.pow((M3 - aM),2) + Math.pow((M4 - aM),2)) / 3;
    }//获取速度方差
    public double varPosition(){
        double P1 = Math.abs(MotorL1.getCurrentPosition());
        double P2 = Math.abs(MotorL2.getCurrentPosition());
        double P3 = Math.abs(MotorR1.getCurrentPosition());
        double P4 = Math.abs(MotorR2.getCurrentPosition());
        double aP = (P1 + P2 + P3 + P4) / 4;
        return (Math.pow((P1 - aP),2) + Math.pow((P2 - aP),2) + Math.pow((P3 - aP),2) + Math.pow((P4 - aP),2)) / 3;
    }//获取position方差

    double t1 = 1,t2 = 1,t3 = 1,t4 = 1;
    public double Targetspeed = 0;
    public double AM = 0;
    private void PIDMove(){
        double RF = Math.abs(changeR1);
        double LB = Math.abs(changeL2);
        double RB = Math.abs(changeR2);
        double LF = Math.abs(changeL1);


        double TargetPosition = Math.min(LF,Math.min(Math.min(RF,LB),RB));
        if(TargetPosition == 0){
            setMacanum();
            t1 = 1;t2 = 1;t3 = 1;t4 = 1;
            return;
        }

        if(IfinRange(RF,TargetPosition - 1,TargetPosition + 1) == false){
            double shift1 = RF - TargetPosition;
            double deltaPower1 = gain(shift1);
            AM = gain(shift1);
            t1 -= deltaPower1;
        }
        if(IfinRange(RB,TargetPosition - 1,TargetPosition + 1) == false){
            double shift2 = RB - TargetPosition;
            double deltaPower2 = gain(shift2);
            t2 -= deltaPower2;
        }
        if(IfinRange(LB,TargetPosition - 1,TargetPosition + 1) == false){
            double shift3 = LB - TargetPosition;
            double deltaPower3 = gain(shift3);
            t3 -= deltaPower3;
        }
        if(IfinRange(LF,TargetPosition - 1,TargetPosition + 1) == false){
            double shift4 = LF - TargetPosition;
            double deltaPower4 = gain(shift4);
            t4 -= deltaPower4;
        }
        Targetspeed = TargetPosition;
        MotorL1.setPower(L1 * t4);
        MotorL2.setPower(L2 * t3);
        MotorR1.setPower(R1 * t1);
        MotorR2.setPower(R2 * t2);
         return ;
    }

    double dt1 = 1,dt2 = 1,dt3 = 1,dt4 = 1;
    public boolean toCertainDistace(double v,double distance,int mode) {//1-向前，2-向后，3-向左，4-向右
        double position = r * distance / (2 * Math.PI * RADIUS);
        double PositionL1 = Math.abs(MotorL1.getCurrentPosition());
        double PositionL2 = Math.abs(MotorL2.getCurrentPosition());
        double PositionR1 = Math.abs(MotorR1.getCurrentPosition());
        double PositionR2 = Math.abs(MotorR2.getCurrentPosition());
        double TargetPosition = Math.min(PositionL1, Math.min(Math.min(PositionL2, PositionR1), PositionR2));


        switch (mode){
            case 1:
                L1 = v;
                L2 = v;
                R1 = -v;
                R2 = -v;
                break;
            case 2:
                L1 = -v;
                L2 = -v;
                R1 = v;
                R2 = v;
                break;
            case 3:
                L1 = v;
                L2 = -v;
                R1 = v;
                R2 = -v;
                break;
            case 4:
                L1 = -v;
                L2 = v;
                R1 = -v;
                R2 = v;
                break;
            default:
                L1 = 0;
                L2 = 0;
                R1 = 0;
                R2 = 0;
                break;
        }

        if (TargetPosition < position) {
            if(IfinRange(PositionL1,TargetPosition - 5,TargetPosition + 5) == false){
                double shift1 = PositionL1 - TargetPosition;
                double deltaPower1 = gain(shift1);
                AM = gain(shift1);
                dt1 -= deltaPower1;
            }
            if(IfinRange(PositionL2,TargetPosition - 5,TargetPosition + 5) == false){
                double shift2 = PositionL2 - TargetPosition;
                double deltaPower2 = gain(shift2);
                dt2 -= deltaPower2;
            }
            if(IfinRange(PositionR1,TargetPosition - 5,TargetPosition + 5) == false){
                double shift3 = PositionR1 - TargetPosition;
                double deltaPower3 = gain(shift3);
                dt3 -= deltaPower3;
            }
            if(IfinRange(PositionR2,TargetPosition - 5,TargetPosition + 5) == false){
                double shift4 = PositionR2 - TargetPosition;
                double deltaPower4 = gain(shift4);
                dt4 -= deltaPower4;
            }
            MotorL1.setPower(L1 * dt1);
            MotorL2.setPower(L2 * dt2);
            MotorR1.setPower(R1 * dt3);
            MotorR2.setPower(R2 * dt4);
            return false;
        }
        else{
            Stop();
            dt1 = 1;dt2 = 1;dt3 = 1;dt4 = 1;
            return true;
        }
    }//定距离平移

    double at1 = 1,at2 = 1,at3 = 1,at4 = 1;
    public boolean toCertainAngle(double v,double angle,int mode){//1-顺时针，2-逆时针
        double distance = angle / 180 * Math.PI * selfRADIUS;
        double position = r * distance / (2 * Math.PI * RADIUS);
        double PositionL1 = Math.abs(MotorL1.getCurrentPosition());
        double PositionL2 = Math.abs(MotorL2.getCurrentPosition());
        double PositionR1 = Math.abs(MotorR1.getCurrentPosition());
        double PositionR2 = Math.abs(MotorR2.getCurrentPosition());
        double TargetPosition = Math.min(PositionL1, Math.min(Math.min(PositionL2, PositionR1), PositionR2));


        switch (mode){
            case 1:
                L1 = v;
                L2 = v;
                R1 = v;
                R2 = v;
                break;
            case 2:
                L1 = -v;
                L2 = -v;
                R1 = -v;
                R2 = -v;
                break;
            default:
                L1 = 0;
                L2 = 0;
                R1 = 0;
                R2 = 0;
                break;
        }

        if (TargetPosition < position) {
            if(IfinRange(PositionL1,TargetPosition - 5,TargetPosition + 5) == false){
                double shift1 = PositionL1 - TargetPosition;
                double deltaPower1 = gain(shift1);
                AM = gain(shift1);
                at1 -= deltaPower1;
            }
            if(IfinRange(PositionL2,TargetPosition - 5,TargetPosition + 5) == false){
                double shift2 = PositionL2 - TargetPosition;
                double deltaPower2 = gain(shift2);
                at2 -= deltaPower2;
            }
            if(IfinRange(PositionR1,TargetPosition - 5,TargetPosition + 5) == false){
                double shift3 = PositionR1 - TargetPosition;
                double deltaPower3 = gain(shift3);
                at3 -= deltaPower3;
            }
            if(IfinRange(PositionR2,TargetPosition - 5,TargetPosition + 5) == false){
                double shift4 = PositionR2 - TargetPosition;
                double deltaPower4 = gain(shift4);
                at4 -= deltaPower4;
            }
            MotorL1.setPower(L1 * at1);
            MotorL2.setPower(L2 * at2);
            MotorR1.setPower(R1 * at3);
            MotorR2.setPower(R2 * at4);
            return false;
        }
        else{
            Stop();
            at1 = 1;at2 = 1;at3 = 1;at4 = 1;
            return true;
        }
    }
    public void Stop(){
        L1 = 0;
        L2 = 0;
        R1 = 0;
        R2 = 0;
        setMacanum();
        return;
    }    //停止
    public void Circle(double w){
        L1 = w;
        L2 = w;
        R1 = w;
        R2 = w;
        setMacanum();
        return;
    }//原地旋转
    public void AutoMove(double v,double aa){
        double a = aa / 180 * Math.PI;
        double vx = v * Math.cos(a);
        double vy = v * Math.sin(a);

        L1 = vy + vx;
        L2 = vy - vx;
        R1 = vy - vx;
        R2 = vy + vx;
        if(L1 == 0 && L2 == 0 ){
            Stop();
            return;
        }

        double p = 1 / Math.max(Math.abs(L1),Math.abs(L2));
        if(p < 1){
            L1 *= p;
            L2 *= p;
            R1 *= -p;
            R2 *= -p;
        }

        this.setMacanum();
        return;
    }//设置速度与角度移动
    public void Stick(double vx,double vy){
        if(Math.abs(vx) > Math.abs(vy)){
            L1 = vx;
            L2 = -vx;
            R1 = vx;
            R2 = -vx;
        }
        else if (Math.abs(vx) < Math.abs(vy)){
            L1 = vy;
            L2 = vy;
            R1 = -vy;
            R2 = -vy;
        }
        else if(Math.abs(vx) == Math.abs(vy)){
            L1 = 0;
            L2 = 0;
            R1 = 0;
            R2 = 0;
        }

        if(L1 == 0 && L2 == 0 ){
            Stop();
            return;
        }

        double p = 1 / Math.max(Math.abs(L1),Math.abs(L2));
        if(p < 1) {
            L1 *= p;
            L2 *= p;
            R1 *= p;
            R2 *= p;
        }
        PIDMove();
        return;
    }    //手柄控制
    public void Stick2(double vx,double vy){
        if(Math.abs(vx) > Math.abs(vy)){
            L1 = vx;
            L2 = -vx;
            R1 = vx;
            R2 = -vx;
        }
        else if (Math.abs(vx) < Math.abs(vy)){
            L1 = vy;
            L2 = vy;
            R1 = -vy;
            R2 = -vy;
        }
        else if(Math.abs(vx) == Math.abs(vy)){
            L1 = 0;
            L2 = 0;
            R1 = 0;
            R2 = 0;
        }

        if(L1 == 0 && L2 == 0 ){
            Stop();
            return;
        }

        double p = 1 / Math.max(Math.abs(L1),Math.abs(L2));
        if(p < 1) {
            L1 *= p;
            L2 *= p;
            R1 *= p;
            R2 *= p;
        }
        setMacanum();
        return;
    }

    static public boolean IfinRange(double value, double min, double max){
        if(value >= min && value <= max)
            return true;
        else
            return false;

    }
    static public double gain(double shift){
        double gain;
        shift += 1;
        gain = 0.05 * Math.log10(shift);
        return gain;
    }

    public void resetMecanum(){
        PIDClass.resetEncode(MotorL2);
        PIDClass.resetEncode(MotorL1);
        PIDClass.resetEncode(MotorR2);
        PIDClass.resetEncode(MotorR1);
        return;
    }
    static public void resetEncode(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForRefresh();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    static public void waitForRefresh(){
        for(int i = 0; i<= 20000;i++){

        }
    }
}