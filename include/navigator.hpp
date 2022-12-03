/**
 * Copyright (c) 2022 Aneesh Chodisetty, Bhargav Kumar Soothram, Joseph Pranadheer Reddy Katakam
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file navigator.hpp
 * @author Aneesh Chodisetty (aneeshc@umd.edu)
 * @author Bhargav Kumar Soothram (bsoothra@umd.edu)
 * @author Joseph Pranadheer Reddy Katakam (jkatak@umd.edu)
 * @brief Header file for navigator.cpp
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef INCLUDE_NAVIGATOR_HPP_
#define INCLUDE_NAVIGATOR_HPP_

#include <memory>

// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/static_transform_broadcaster.h"


class Navigator {
    public:
    Navigator();
    ~Navigator();
    /**
     * @brief Moves the bots and populates current_locations_ after reading the incoming data from an node
     * 
     */
    void moveIt() {}
    /**
     * @brief Broadcasts the current locations continuously
     * 
     */
    void broadcaster() {}

    private:
    const std::array<int, 20> final_locations_;
    std::array<int, 20> current_locations_;
    std::array<int, 20> transforms_;
};

#endif