package com.nemo.neplan.config;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.servlet.config.annotation.ResourceHandlerRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

@Configuration
public class WebMvcConfig implements WebMvcConfigurer {

    private String uploadDirectory="/usr/local/lib/upload-dir/";

    @Override
    public void addResourceHandlers(ResourceHandlerRegistry registry) {
        // uploads 디렉토리에 업로드된 파일에 대한 URL을 생성합니다.
        // 실제 파일 시스템의 경로를 설정하고 URL 패턴을 지정합니다.
        registry.addResourceHandler("/uploads/**")
                .addResourceLocations("file:" + uploadDirectory);
    }
}
