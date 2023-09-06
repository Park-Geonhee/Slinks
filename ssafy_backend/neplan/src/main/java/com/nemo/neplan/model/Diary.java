package com.nemo.neplan.model;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.nemo.neplan.dto.DiaryDto;

import javax.persistence.*;

@Entity
public class Diary extends BaseTimeEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;
    private String content;

    // User와 관련된 필드 선언
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id")
    @JsonIgnoreProperties({"hibernateLazyInitializer", "handler"})
    private User user;

    // Place와 관련된 필드 선언
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "place_id")
    @JsonIgnoreProperties({"hibernateLazyInitializer", "handler"})
    private Place place;

    // File과 관련된 필드 선언
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "file_id")
    @JsonIgnoreProperties({"hibernateLazyInitializer", "handler"})
    private File file;

    public Diary(){}
    public Diary(Long id, String content, User user, Place place, File file) {
        this.id = id;
        this.content = content;
        this.user = user;
        this.place = place;
        this.file = file;
    }

    public Long getId() {
        return id;
    }

    public void setId(Long id) {
        this.id = id;
    }

    public String getContent() {
        return content;
    }

    public void setContent(String content) {
        this.content = content;
    }

    public User getUser() {
        return user;
    }

    public void setUser(User user) {
        this.user = user;
    }

    public Place getPlace() {
        return place;
    }

    public void setPlace(Place place) {
        this.place = place;
    }

    public File getFile() {
        return file;
    }

    public void setFile(File file) {
        this.file = file;
    }

    //일기 수정 메소드
    public void diaryPathch(Diary diary){
        if(diary.content !=null){
            this.content=diary.content;
        }

        if(diary.place!=null){
            this.place=diary.place;
        }
    }

    //다이어리를 찾을 때 사용함
    public DiaryDto toDto(){
        return new DiaryDto(id, content, user, place, file);
    }


}
