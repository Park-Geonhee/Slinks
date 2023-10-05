package com.nemo.neplan.model;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import javax.persistence.*;
import java.io.Serializable;

@Entity
public class PlanPlace implements Serializable {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "plan_id")
    @JsonIgnoreProperties({"hibernateLazyInitializer", "handler"})
    private Plan plan;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "place_id")
    @JsonIgnoreProperties({"hibernateLazyInitializer", "handler"})
    private Place place;


//    private int placeOrder=0;


//    public int getPlaceOrder() {
//        return placeOrder;
//    }
//
//    public void setPlaceOrder(int placeOrder) {
//        this.placeOrder = placeOrder;
//    }

    public long getId() {
        return id;
    }



    public void setId(long id) {
        this.id = id;
    }

    public Plan getPlan() {
        return plan;
    }

    public void setPlan(Plan plan) {
        this.plan = plan;
    }

    public Place getPlace() {
        return place;
    }

    public void setPlace(Place place) {
        this.place = place;
    }



}
