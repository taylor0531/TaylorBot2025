// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

//import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CandleSubsystem extends SubsystemBase {
    private final static CANdle m_candle = new CANdle(Constants.CANdleID, "Carnie");
    private final int LedCount = 68;
    //private Joystick joystick;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }
    private AnimationTypes m_currentAnimation;

    public CandleSubsystem() {

        //changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
        
    }

    public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(m_toAnimate == null) {
            singleFadePurple();
        } else {
            m_candle.animate(m_toAnimate);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    //  morse code dots half a sec flash, half a sec blank
    static void dotMethod(int r, int g, int b, int w, int startIdx, int count){
        m_candle.setLEDs(r,g,b,w,startIdx,count);//on
        try {
            Thread.sleep(500);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            System.out.println("morsecode exception "+e.toString());
        }
        m_candle.setLEDs(0,00,0,0,0,60);//off
        try {
            Thread.sleep(500);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            System.out.println("morsecode exception "+e.toString());
        }
    }

    //  morse code dashes sec and a half flash, half a sec blank
    static void dashMethod(int r, int g, int b, int w, int startIdx, int count){
        m_candle.setLEDs(r,g,b,w,startIdx,count);//on
        try {
            Thread.sleep(1500);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            System.out.println("morsecode exception "+e.toString());
        }
        m_candle.setLEDs(0,0,0,0,0,60);//off
        try {
            Thread.sleep(500);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            System.out.println("morsecode exception "+e.toString());
        }
    }

    public void morseCode(){
        //morse code for 4085

        //G = - - .
        //O = - - -

        //4 = . . . . -
        //0 = - - - - - 
        //8 = - - - . . 
        //5 = . . . . . 
        // space between units is 3 units
        // each dot is one unit
        // each line is 3 units
        try {
            //m_toAnimate = null;
            System.out.println("Inside morse code");
        
            //G = - - .
            dashMethod(252,226,5,0,0,60);
            dashMethod(252,226,5,0,0,60);
            dotMethod(186,85,211,0,0,60);

            //O = - - - 
            dashMethod(252,226,5,0,0,60);
            dashMethod(252,226,5,0,0,60);
            dashMethod(252,226,5,0,0,60);
        
            //4 = . . . . -
            dotMethod(0,100,255,0,0,60);
            dotMethod(0,100,255,0,0,60);
            dotMethod(0,100,255,0,0,60);
            dotMethod(0,100,255,0,0,60);
            dashMethod(0,100,255,0,0,60);
            
            //0 = - - - - - 
            dashMethod(159,35,247,0,0,60);
            dashMethod(159,35,247,0,0,60);
            dashMethod(159,35,247,0,0,60);
            dashMethod(159,35,247,0,0,60);
            dashMethod(159,35,247,0,0,60);
                  
            //8 = - - - . . 
            dashMethod(232,115,5,0,0,60);
            dashMethod(232,115,5,0,0,60);
            dashMethod(232,115,5,0,0,60);
            dotMethod(232,115,5,0,0,60);
            dotMethod(232,115,5,0,0,60);
        
            //5 = . . . . . 
            dotMethod(232,5,5,0,0,60);
            dotMethod(232,5,5,0,0,60);
            dotMethod(232,5,5,0,0,60);
            dotMethod(232,5,5,0,0,60);
            dotMethod(232,5,5,0,0,60);
            
          } catch (Exception e) {
            // TODO: handle exception
            System.out.println("morsecode exception "+e.toString());
        }
        
        
    }

    //flash blue, flash white
    //public void flash(){

    //    m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
    // }

    //public void singleFade() {
    //    m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
    // }

    // use this when elevator limit switch is triggered.
    //  flashing red
    public void singleFadeRed() {
        m_toAnimate = new SingleFadeAnimation(255, 0, 0, 0, 1, LedCount);
    }
    // use this as the default / works when driving too
    //  slow flashing purple
    public void singleFadePurple() {
        
        m_toAnimate = new SingleFadeAnimation(75, 0, 130, 0, .3, LedCount);
    }
    // use this when shooting
    //  mid speed 
    //public void singleFadeGreen() {
    //    m_toAnimate = new SingleFadeAnimation(0, 128, 0, 0, .7, LedCount);
    //}

    // use this when driving
       // slow flash white
    public void singleFadewhite() {
        
        m_toAnimate = new SingleFadeAnimation(0, 0, 0, 200, .7, LedCount);
    }

    //white for robot on before auto & tele
    public void whiteAnimation() {
        m_toAnimate =new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);  
    }

    public void shootAnimation(){
        m_toAnimate =new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
    }

    public void purpleAnimation(){
        m_toAnimate = new ColorFlowAnimation(128, 0,128, 0,0.5, LedCount, Direction.Forward);
    }

}