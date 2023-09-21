package com.nemo.neplan.model;

import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import javax.persistence.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

@MappedSuperclass
@EntityListeners(AuditingEntityListener.class)
abstract class BaseTimeEntity {

    @Column(name = "created_date", nullable = false)
    @CreatedDate
    private String createdDate;

    @Column(name = "modified_date", nullable = false)
    @LastModifiedDate
    private String modifiedDate;


    //엔티티 저장 전에 실행 될 함수
    @PrePersist
    public void onPrePersist(){
        //생성일자, 수정일자 모두 현재시간으로 설정
        this.createdDate= LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMddHHmm"));
        this.modifiedDate=this.createdDate;
    }

    //엔티티 업데이트 전에 실행
    @PreUpdate
    public void onPreUpdate(){
        //수정일자를 현재 시간으로 설정
        this.modifiedDate=LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMddHHmm"));
    }

    public String getCreatedDate() {
        return createdDate;
    }

    public void setCreatedDate(String createdDate) {
        this.createdDate = createdDate;
    }

    public String getModifiedDate() {
        return modifiedDate;
    }

    public void setModifiedDate(String modifiedDate) {
        this.modifiedDate = modifiedDate;
    }
}
