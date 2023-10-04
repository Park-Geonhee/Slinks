package com.example.map

import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.view.View
import com.example.map.databinding.ActivityLoginBinding
import com.example.map.databinding.ActivityMainBinding

class LoginActivity : AppCompatActivity() {

    private lateinit var binding: ActivityLoginBinding
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding=ActivityLoginBinding.inflate(layoutInflater)
        setContentView(binding.root)
//        setContentView(R.layout.activity_login)





        //카카오 로그인 버튼
        binding.btnKakaoLogin.setOnClickListener(View.OnClickListener {
            var intent= Intent(this@LoginActivity,MainActivity::class.java )
            //ai diary페이지로 이동
            startActivity(intent)
        })
    }
}