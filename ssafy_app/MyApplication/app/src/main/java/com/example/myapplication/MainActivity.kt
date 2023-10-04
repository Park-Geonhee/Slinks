package com.example.myapplication

import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.GridView
import androidx.viewpager.widget.ViewPager
import com.example.myapplication.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    internal lateinit var viewPager: ViewPager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_slash)
//        setContentView(R.layout.splash_layout)
//        setContentView(R.layout.ac)

//
//        val img=arrayOf(
//            R.drawable.ai,
//            R.drawable.css,
//            R.drawable.html,
//            R.drawable.id,
//            R.drawable.jpg,
//            R.drawable.js,
//            R.drawable.mp4,
//            R.drawable.pdf,
//            R.drawable.png,
//            R.drawable.psd,
//            R.drawable.tiff,
//            R.drawable.php,
//        )

        var text=arrayOf(
            "ai",
            "css",
            "html",
            "id",
            "jpg",
            "js",
            "mp4",
            "pdf",
            "png",
            "psd",
            "tiff",
            "php"
        )

//        binding = ActivityMainBinding.inflate(layoutInflater)
//        setContentView(binding.root)

//        val gridViewAdapter = GridViewAdapter(this, img, text)
//        binding.gridview.adapter = gridViewAdapter
//
//        binding.gridview.setOnItemClickListener { adapterView, view, i, l ->
//            val intent = Intent(this, LectureActivity::class.java)
//            startActivity(intent)
//        }

//        //viewpager가 하면에 나타나도록 설정
//        viewPager=findViewById(R.id.viewpager) as ViewPager
//
//        val adapter=ViewPageAdapter(this)
//        viewPager.adapter=adapter








        //그리드 뷰가 화면에 나타나도록 설정, img, text도 같이 전달
//        val gridView = findViewById<GridView>(R.id.gridview)
//        val gridViewAdapter=GridViewAdapter(this, img, text)
//        gridView.adapter=gridViewAdapter

        //gridview를 누르면 lecture activity로 넘어가게 설정
//        gridView.setOnItemClickListener { adapterView, view, i, l ->
//
//            val intent=Intent(this, LectureActivity::class.java)
//            startActivity(intent)
//        }





    }
}