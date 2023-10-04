package com.nemo.neplan.model;

import javax.persistence.*;

@Entity
public class File {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private long id;

    //실제로 로컬에 저장된 이미지 파일 이름
    private String imgName;
    //업로드 했던 이미지파일 초기이름
    private String originalFilename;
    //업로드 결과 로컬에 저장된 이미지 파일을 불러올 결과
    private String imgUrl;

    public File() {
    }

    public File(String imgName, String originalFilename, String imgUrl) {

        this.imgName = imgName;
        this.originalFilename = originalFilename;
        this.imgUrl = imgUrl;
    }



    //
//    //이미지 수정시 사용
//    public void updateFile(String imgName, String originalFilename, String imgUrl) {
//        this.imgName = imgName;
//        this.originalFilename = originalFilename;
//        this.imgUrl = imgUrl;
//    }



    public long getId() {
        return id;
    }

    public void setId(long id) {
        this.id = id;
    }

    public String getImgName() {
        return imgName;
    }

    public void setImgName(String imgName) {
        this.imgName = imgName;
    }

    public String getOriginalFilename() {
        return originalFilename;
    }

    public void setOriginalFilename(String originalFilename) {
        this.originalFilename = originalFilename;
    }

    public String getImgUrl() {
        return imgUrl;
    }

    public void setImgUrl(String imgUrl) {
        this.imgUrl = imgUrl;
    }
}