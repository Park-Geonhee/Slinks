package com.nemo.neplan.dto;

import com.nemo.neplan.model.Diary;
import com.nemo.neplan.model.User;
import com.nemo.neplan.model.Place;
import com.nemo.neplan.model.File;

import java.util.ArrayList;
import java.util.List;

public class DiaryDto {
    private Long id;
    private String content;
    private User user;
    private Place place;
//    private File file;
    //, File file

    public DiaryDto(Long id, String content, User user, Place place) {
        this.id = id;
        this.content = content;
        this.user = user;
        this.place = place;
//        this.file = file;
    }

    // Getter 및 Setter 메소드들

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

//    public File getFile() {
//        return file;
//    }
//
//    public void setFile(File file) {
//        this.file = file;
//    }


    //수정할 때 다이어리의 이미지 아이디를 저장하는 리스트
    private List<Long> diaryFileIdList=new ArrayList<>();

    //수정할 때 다이어리 안의 이미지 정보를 저장하는 리스트


//    //일기 수정시에 사용함
//    public Diary toEntity(){
//        return new Diary(id, content, user, place);
//    }

}
