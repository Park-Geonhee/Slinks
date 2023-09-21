package com.nemo.neplan;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import springfox.documentation.swagger2.annotations.EnableSwagger2;

@SpringBootApplication
@EntityScan(basePackages = "com.nemo.neplan.model")
public class NeplanApplication {

	public static void main(String[] args) {
		SpringApplication.run(NeplanApplication.class, args);
	}

}
