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


    private int placeOrder;



    public long getId() {
        return id;
    }

    public int getPlaceOrder() {
        return placeOrder;
    }

    public void setPlaceOrder(int placeOrder) {
        this.placeOrder = placeOrder;
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

    @PrePersist
    public void assignPlaceOrder() {
        if (plan != null) {
            // 플레이리스트에 속한 곡 중 가장 큰 placeOrder 값을 조회합니다.
            int maxPlaceOrder = plan.getMaxPlaceOrder();

            // maxPlaceOrder 값이 없는 경우(플레이리스트가 비어있는 경우) 1을 할당합니다.
            if (maxPlaceOrder == 0) {
                this.placeOrder = 1;
            } else {
                // maxPlaceOrder 값에 1을 더한 값을 할당합니다.
                this.placeOrder = maxPlaceOrder + 1;
            }
        }
    }

}
