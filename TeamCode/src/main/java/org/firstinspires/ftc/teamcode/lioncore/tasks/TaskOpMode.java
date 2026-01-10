package org.firstinspires.ftc.teamcode.lioncore.tasks;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lioncore.control.Controller;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

import java.util.List;

public abstract class TaskOpMode extends OpMode {
    private Task task;
    private boolean endWhenTasksFinished;
    private boolean taskHasFinished;
    public Controller controller1;
    public Controller controller2;

    private long lastTime;
    private List<LynxModule> hubs;
    private List<SystemBase> systems;

    /**
     * Create all systems and tasks and return them. Do not initialise the systems.
     * @return Return a "Jobs" item containing task and systems
     */
    public abstract Jobs spawn();

    /**
     * Empty by default, allows users to override with something they want to happen once all commands in this iteration have been executed.
     * This could be used for something like bulk reading, updating a counter, etc.
     */
    public void mainloop() {

    }

    @Override
    public void init() {

        this.controller1 = new Controller(gamepad1);
        this.controller2 = new Controller(gamepad2);

        this.lastTime = System.nanoTime();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Jobs jobs = this.spawn();
        this.task = jobs.compileTask();
        this.systems = jobs.getSystems();
        this.endWhenTasksFinished = jobs.getEndWhenTasksFinished();
        this.taskHasFinished = false;

        for (SystemBase system : this.systems) {
            system.loadHardware(this.hardwareMap);
            system.init();
        }

        // Bulk hardware operations
        this.hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : this.hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
        }

        task.init();
    }

    @Override
    public void loop() {

        long time = System.nanoTime();
        long deltatime = time - lastTime;
        double deltatime_seconds = deltatime / 1_000_000_000.0;
        double frequency_hz = 1 / deltatime_seconds;
        double rounded_hz = Math.round(frequency_hz);

        this.controller1.update(gamepad1);
        this.controller2.update(gamepad2);

        if (!this.taskHasFinished) this.task.run();
        if (this.task.finished() && !this.taskHasFinished) {
            task.end(false);
            this.taskHasFinished = true;
            if (this.endWhenTasksFinished) this.requestOpModeStop();
        }

        for (SystemBase system : this.systems) {
            system.update(this.telemetry);
        }

        this.mainloop();
        this.telemetry.addData("Loopfreq", rounded_hz);
        this.telemetry.update();

        // Fully update all sensor values, motor positions, ect, once per loop cycle.
        for (LynxModule hub : this.hubs) {
           hub.clearBulkCache();
        }

        this.lastTime = time;
    }
}
