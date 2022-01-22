package org.team199.robot2022;

public class WrappedObject<T> {

    public T obj;

    public WrappedObject(T obj) {
        this.obj = obj;
    }

    public T getObj() {
        return obj;
    }

    public void setObj(T obj) {
        this.obj = obj;
    }

}
