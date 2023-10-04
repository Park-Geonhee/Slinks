package com.example.map

data class PlanPlace(
    val id: Long,
    val plan: Plan?,
    val place: Place?,
    val placeOrder: Int
)