package com.team2052.lib.motor;

public class CANItem {
    private int id;
    private String loop;

    public CANItem (int id, String loop) {
        this.id = id;
        this.loop = loop;
    }

    public int getId() {
        return id;
    }

    public String getLoop() {
        return loop;
    }
}
