/**
 * @file small_dynamic_model.cpp
 * @Shuhang Tan (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 * 路口: 行人绿灯: 车: 1. 直行,等待; 2. 转弯,减速,有行人,等待
 * 行人红灯: 行人位置及速度: 1. 在道路边界之外: 车走; 2. 是否有进入道路的趋势(预测)
 *
 */