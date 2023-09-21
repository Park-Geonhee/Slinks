package com.nemo.neplan.dto;

public class FileDto {
    private long id;
    private String imgName;
    private String originalFilename;

    public FileDto(long id, String imgName, String originalFilename) {
        this.id = id;
        this.imgName = imgName;
        this.originalFilename = originalFilename;
    }

    // Getter 및 Setter 메소드들

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
}
