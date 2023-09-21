package com.nemo.neplan.service;

import com.nemo.neplan.model.File;
import org.springframework.web.multipart.MultipartFile;

import java.util.List;

public interface FileService {
  //파일업로드
    String uploadFiles(String uploadPath, String originalName, byte[] fileData) throws Exception;
    void deleteFile(String filePath) throws Exception;


}
