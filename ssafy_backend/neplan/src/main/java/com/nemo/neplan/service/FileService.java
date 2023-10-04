package com.nemo.neplan.service;

import com.nemo.neplan.model.File;
import org.springframework.web.multipart.MultipartFile;

import java.util.List;

public interface FileService {
  //파일업로드
   Long uploadFiles(MultipartFile file) throws Exception;
    void deleteFile(String filePath) throws Exception;

    File getFileById(long id);


}
