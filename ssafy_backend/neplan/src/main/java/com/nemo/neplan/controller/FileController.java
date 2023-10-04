package com.nemo.neplan.controller;

import com.nemo.neplan.model.File;
import com.nemo.neplan.service.FileService;
import io.swagger.annotations.Api;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.core.io.Resource;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

@RestController
@Api(tags = "File 관련 API", description = "파일 관리 API")
@RequestMapping("/file")
public class FileController {

    @Autowired
    private FileService fileService;

    @PostMapping("/upload")
    public ResponseEntity<Long> uploadFile(
            @RequestParam("file") MultipartFile file) {


        try {
            Long fileId = fileService.uploadFiles(file);
            return ResponseEntity.ok(fileId);
        } catch (Exception e) {
            // 예외 처리 코드 추가
            e.printStackTrace();
            return ResponseEntity.status(500).build(); // Internal Server Error
        }
    }
}
