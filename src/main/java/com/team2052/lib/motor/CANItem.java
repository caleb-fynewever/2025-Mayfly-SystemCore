package com.team2052.lib.motor;

import com.ctre.phoenix6.CANBus;

public class CANItem {
    private int id;
    private CANBus loop;

    public CANItem (int id, CANBus loop) {
        this.id = id;
        this.loop = loop;
    }

    public int getId() {
        return id;
    }

    public CANBus getLoop() {
        return loop;
    }
}
