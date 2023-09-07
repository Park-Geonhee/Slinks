package com.example.myapplication

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.GridView
import androidx.viewpager.widget.ViewPager


class MainActivity : AppCompatActivity() {

    internal lateinit var viewPager: ViewPager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)


        val img=arrayOf(
            R.drawable.ai,
            R.drawable.css,
            R.drawable.html,
            R.drawable.id,
            R.drawable.jpg,
            R.drawable.js,
            R.drawable.mp4,
            R.drawable.pdf,
            R.drawable.png,
            R.drawable.psd,
            R.drawable.tiff,
            R.drawable.php,
        )

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

        val gridView = findViewById<GridView>(R.id.gridview)
        val gridViewAdapter=GridViewAdapter(this, img, text)
        gridView.adapter=gridViewAdapter



        viewPager=findViewById(R.id.viewpager) as ViewPager

        val adapter=ViewPageAdapter(this)
        viewPager.adapter=adapter


    }
}