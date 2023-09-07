package com.example.myapplication

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import androidx.viewpager.widget.PagerAdapter
import androidx.viewpager.widget.ViewPager

// 이미지 슬라이더
class ViewPageAdapter(private val context: Context) : PagerAdapter() {


    private var layoutInflater:LayoutInflater?=null

    //슬라이드에 들어갈 이미지 배열을 생성한다.
    val Image=arrayOf( //drawable file에 사진들을 보관한다,
        R.drawable.id,
        R.drawable.js,
        R.drawable.html,
        R.drawable.ai,
        R.drawable.css,
        R.drawable.php,
        R.drawable.jpg,
        R.drawable.mp4,
        R.drawable.pdf,
        R.drawable.png,
        R.drawable.psd,
        R.drawable.tiff
    )

    override fun isViewFromObject(view: View, `object`: Any): Boolean {
        return view === `object`
    }


    //슬라이드가 몇 개 넘어갈지 크기를 정해줌
    override fun getCount(): Int {
       return Image.size
    }

    // override 키워드를 사용하여 상위 클래스의 메서드를 재정의합니다.
    override fun instantiateItem(container: ViewGroup, position: Int): Any {
        // 레이아웃 인플레이터 객체를 생성합니다.
        layoutInflater = context.getSystemService(Context.LAYOUT_INFLATER_SERVICE) as LayoutInflater

        val v = layoutInflater!!.inflate(R.layout.viewpager_activity, null)  // 레이아웃 리소스(R.layout.viewpager_activity)를 이용하여 뷰 객체(v)를 인플레이트합니다.
        val image = v.findViewById<View>(R.id.imageview)  as ImageView// 뷰(v)에서 이미지뷰(R.id.imageview)를 찾아내고 image 변수에 할당합니다.

        image.setImageResource(Image[position])


        val vp=container as ViewPager
        vp.addView(v,0)

        return v
    }


    override fun destroyItem(container: ViewGroup, position: Int, `object`:Any){

        val vp=container as ViewPager
        val v=`object` as View
        vp.removeView(v)
    }



}