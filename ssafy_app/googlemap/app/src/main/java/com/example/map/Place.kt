package com.example.map



data class Place(
    val id: Long,
    val name: String,
    val address: String,
    val phone: String,
    val x: String,
    val y: String,
    val placeType: PlaceType  // 열거형을 사용한 데이터 타입으로 변경
)


