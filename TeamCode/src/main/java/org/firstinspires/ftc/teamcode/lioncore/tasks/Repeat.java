package org.firstinspires.ftc.teamcode.lioncore.tasks;

public class Repeat extends Task {
    private Task task;
    private boolean running;
    private int counter;

    /**
     * Execute a task N times.
     * @param task
     */
    public Repeat(Task task, int counter) {
        this.task = task;
        this.running = false;
        this.counter = counter;
    }

    public Repeat(Runnable runnable, int counter) {
        this.task = new Run(runnable);
        this.running = false;
        this.counter = counter;
    }

    @Override
    public void init() {
        this.running = true;
        this.task.init();
    }

    @Override
    public void run() {
        if (!this.running) return;
        this.task.run();
        if (this.task.finished() && counter > 0) {
            this.task.end(false);
            this.task.init();
        }
        this.counter -= 1;
    }

    @Override
    public boolean finished() {
        if (this.counter <= 0) {
            this.running = false;
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (this.running) this.task.end(interrupted);
    }
}
