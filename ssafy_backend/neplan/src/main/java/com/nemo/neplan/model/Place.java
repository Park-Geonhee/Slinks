package com.nemo.neplan.model;


import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import javax.persistence.*;

@Entity
public class Place {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private String x;
    private String y;

    @Column(length = 512)
    private String address;

    // File과 관련된 필드 선언
//    @ManyToOne(fetch = FetchType.LAZY)
//    @JoinColumn(name = "file_id")
//    @JsonIgnoreProperties({"hibernateLazyInitializer", "handler"})
//    private File file;

    @Enumerated(EnumType.STRING) // Enum 타입을 문자열로 저장
    private PlaceType placeType;

    public PlaceType getPlaceType() {
        return placeType;
    }

    public void setPlaceType(PlaceType placeType) {
        this.placeType = placeType;
    }

    public Long getId() {
        return id;
    }

    public void setId(Long id) {
        this.id = id;
    }

    public String getX() {
        return x;
    }

    public void setX(String x) {
        this.x = x;
    }

    public String getY() {
        return y;
    }

    public void setY(String y) {
        this.y = y;
    }

    public String getAddress() {
        return address;
    }

    public void setAddress(String address) {
        this.address = address;
    }

//    public File getFile() {
//        return file;
//    }
//
//    public void setFile(File file) {
//        this.file = file;
//    }
}
