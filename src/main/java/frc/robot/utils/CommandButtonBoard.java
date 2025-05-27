// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandButtonBoard extends CommandGenericHID {
    /* Button IDs */
    static final int kCORAL_STOWED = 2; // Pin/Button 2
    static final int kCORAL_L2 = 3; // Pin/Button 3
    static final int kCORAL_L3 = 4; // Pin/Button 4
    static final int kCORAL_L4 = 5; // Pin/Button 5
    
    static final int kALGAE_STOWED_PROCESSOR = 6; // Pin/Button 6
    static final int kALGAE_LOW = 7; // Pin/Button 7
    static final int kALGAE_HIGH = 8; // Pin/Button 8
    static final int kALGAE_BARGE = 9; // Pin/Button 9
    
    static final int kINTAKE_PREP = 10; // Pin/Button 10
    static final int kINTAKE_RUN = 11; // Pin/Button 11
    static final int kINTAKE_REVERSE = 12; // Pin/Button 12
    
    static final int kOUTTAKE = 13; // Pin/Button 13
    
    // Buttons beyond button 13 cannot be directly assigned to a pin with UnoJoy,
    // so we need to resort to using POV directions instead.
    // The outtake reverse button is assigned to the left POV direction on pin 15.
    
    public CommandButtonBoard(int port) {
        super(port);
    }
    
    public Trigger coralStowed() {
      return this.coralStowed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger coralStowed(EventLoop loop) {
       return this.button(kCORAL_STOWED, loop);
    }
    
    public Trigger coralL2() {
      return this.coralL2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger coralL2(EventLoop loop) {
       return this.button(kCORAL_L2, loop);
    }
    
    public Trigger coralL3() {
      return this.coralL3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger coralL3(EventLoop loop) {
       return this.button(kCORAL_L3, loop);
    }
    
    public Trigger coralL4() {
      return this.coralL4(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger coralL4(EventLoop loop) {
       return this.button(kCORAL_L4, loop);
    }
    
    public Trigger algaeStowedProcessor() {
      return this.algaeStowedProcessor(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger algaeStowedProcessor(EventLoop loop) {
       return this.button(kALGAE_STOWED_PROCESSOR, loop);
    }
    
    public Trigger algaeLow() {
      return this.algaeLow(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger algaeLow(EventLoop loop) {
       return this.button(kALGAE_LOW, loop);
    }
    
    public Trigger algaeHigh() {
      return this.algaeHigh(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger algaeHigh(EventLoop loop) {
       return this.button(kALGAE_HIGH, loop);
    }
    
    public Trigger algaeBarge() {
      return this.algaeBarge(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger algaeBarge(EventLoop loop) {
       return this.button(kALGAE_BARGE, loop);
    }
    
    public Trigger intakePrep() {
      return this.intakePrep(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger intakePrep(EventLoop loop) {
       return this.button(kINTAKE_PREP, loop);
    }
    
    public Trigger intakeRun() {
      return this.intakeRun(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger intakeRun(EventLoop loop) {
       return this.button(kINTAKE_RUN, loop);
    }
    
    public Trigger intakeReverse() {
      return this.intakeReverse(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger intakeReverse(EventLoop loop) {
       return this.button(kINTAKE_REVERSE, loop);
    }
    
    public Trigger outtake() {
      return this.outtake(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    
    public Trigger outtake(EventLoop loop) {
       return this.button(kOUTTAKE, loop);
    }
    
    public Trigger outtakeReverse() {
       return this.povLeft(); // Left POV direction on pin 15
    }
}
